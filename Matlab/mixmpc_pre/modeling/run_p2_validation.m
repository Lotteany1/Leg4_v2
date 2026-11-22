function report = run_p2_validation(urdf_file, output_dir)
% Run independent CAD-reference physics and contact regressions.
here = fileparts(mfilename('fullpath'));
matlab_dir = fileparts(fileparts(here));
if nargin<1 || isempty(urdf_file)
    urdf_file = 'E:/轮腿资料/URDF/chuanliantui_new_1/chuanliantui_new_1/urdf/chuanliantui.urdf';
end
if nargin<2 || isempty(output_dir)
    output_dir = 'E:/TEMP/mixmpc_modeling_20261117/p2';
end
previous_path = path;
cleanup = onCleanup(@() path(previous_path)); %#ok<NASGU>
addpath(here,fullfile(matlab_dir,'Tool'),fullfile(fileparts(here),'functions'));
if ~isfolder(output_dir), mkdir(output_dir); end
[~,hardware] = leg_config();
robot = mixmpc_robot_config(urdf_file);
model = mixmpc_reference_config(robot,hardware);
report.parameters = struct('urdf_sha256',tinympc14_sha256(urdf_file), ...
    'spring_source_sha256',tinympc14_sha256(which('leg_config')), ...
    'mass_kg',sum([robot.links.mass]),'spring',model.spring, ...
    'wheel_radius_m',model.wheel_radius,'mu',0.6,'gravity',model.gravity, ...
    'contact_tolerance',model.contact_tolerance);
sources = {'mixmpc_robot_config','mixmpc_robot_fk','mixmpc_closure','mixmpc_reference_config', ...
    'mixmpc_full_kinematics','mixmpc_full_dynamics','mixmpc_contact_model','mixmpc_contact_kkt', ...
    'mixmpc_reference_plant','mixmpc_reference_impact','mixmpc_simulate_reference','run_p2_validation'};
for i = 1:numel(sources)
    report.source_sha256.(sources{i}) = tinympc14_sha256(fullfile(here,[sources{i} '.m']));
end
% Freeze thresholds before running the tests.
tol = struct('loop_m',1e-6,'jacobian',1e-6,'equations',1e-6, ...
    'energy_J',2e-5,'momentum',2e-6,'gap_m',1e-5,'contact_speed_mps',2e-6, ...
    'negative_force_N',1e-6,'convergence_fraction',0.01);
report.tolerance = tol;
[report.dynamics,report.geometry] = verify_dynamics(model,tol);
[report.contact,traces] = verify_contact(model,tol);
report.status = 'CAD reference P2 numerical tests passed';
report.structural_status = 'CAD closure and limits require hardware confirmation';
report.scope = ['URDF CAD closure branch and local ideal wheel planes. ' ...
    'Hardware motor transmission, mechanical stops and spring calibration remain pending.'];
save(fullfile(output_dir,'p2_validation.mat'),'report','traces');
id = fopen(fullfile(output_dir,'p2_validation.json'),'w','n','UTF-8');
assert(id>=0,'Cannot write P2 validation report.');
close = onCleanup(@() fclose(id)); %#ok<NASGU>
fprintf(id,'%s\n',jsonencode(report,'PrettyPrint',true));
fprintf('%s. Metrics and trajectories: %s\n',report.status,output_dir);
end

function [errors,geometry] = verify_dynamics(model,tol)
robot = model.robot;
rbt = importrobot(robot.urdf,'DataFormat','struct');
home = homeConfiguration(rbt);
rev = robot.joints([robot.joints.q]>0);
[matched,order] = ismember({home.JointName},{rev.name});
assert(all(matched) && numel(order)==robot.nq,'CAD tree joint order changed.');
rbt.DataFormat = 'column';
tree_mass = sum(cellfun(@(b) b.Mass,rbt.Bodies));
base_body = robot.links(robot.base);
total_mass = sum([robot.links.mass]);
base_missing = ~ismember(base_body.name,rbt.BodyNames);
assert(abs(tree_mass+base_missing*base_body.mass-total_mass)<1e-10, ...
    'Imported rigid-body masses differ from CAD XML.');
zero = zeros(12,1);
asymmetric = [0.8;-0.35;0.52;0.07;-0.12;0.15;0.08;0.02;0.10;0.05;0.4;-0.6];
long = [0;0;0.45;-0.1;0.07;-0.15;0;-0.85;0;0.83;0.3;-0.1];
poses = [zero asymmetric long];
v = [0.1;-0.2;0.4;0.08;-0.05;0.15;0.4;-0.6;0.2;0.3;1;-2];
tau = [-2;0.8;1.4;-1.2;0.2;-0.15];
terrain = floor_planes();
terrain.normal(:,2) = [sin(0.12);0;cos(0.12)];
step = 2e-6;
records = cell(1,size(poses,2));
span = inf(1,size(poses,2));
for k = 1:size(poses,2)
    q = poses(:,k);
    d = mixmpc_full_dynamics(model,q,v);
    assert(all(d.joint_margin>=-tol.loop_m),'Audit pose outside exported CAD joint limits.');
    eigs_M = eig(d.M);
    assert(all(eigs_M>0) && norm(d.M-d.M',Inf)<1e-12,'Invalid CAD inertia.');
    span(k) = min(eigs_M);
    rbt_pos = d.kin.joint_position(order);
    R = d.kin.pose.R(:,:,robot.base);
    rbt.Gravity = (R'*model.gravity)';
    ids = 6+order;
    m_err = norm(d.open_M(ids,ids)-massMatrix(rbt,rbt_pos),'fro');
    g_err = norm(d.open_gravity(ids)-gravityTorque(rbt,rbt_pos),Inf);
    % rigidBodyTree's named fixed base is excluded from centerOfMass.
    com_local = centerOfMass(rbt,rbt_pos);
    com_local = (tree_mass*com_local(:)+base_missing*base_body.mass*base_body.com)/total_mass;
    com_err = norm(d.com-q(1:3)-R*com_local(:),Inf);

    % An independent redundant-coordinate KKT solve includes all loop
    % reactions. The production acceleration eliminates the passive angles.
    J = d.kin.loop_J;
    raw_tau = zeros(6+robot.nq,1);
    raw_tau(model.drive) = tau;
    z = [d.open_M -J'; J zeros(size(J,1))] \ ...
        [raw_tau+d.open_Qspring-d.open_h; -d.kin.loop_bias];
    a = d.M \ ([zeros(6,1);tau]+d.Qspring-d.h);
    kkt_err = norm(z(1:numel(raw_tau))-d.kin.N*a-d.kin.open_bias,Inf)/max(1,norm(z(1:numel(raw_tau)),Inf));
    loop_power = abs(z(numel(raw_tau)+1:end)'*(J*d.kin.open_v));

    % Check convective terms from the Lagrange identity and force terms
    % from potential differences, independently of the analytic recursion.
    grad_T = zero;
    grad_U = zeros(12,2);
    for i = 1:12
        dq = zero;
        dq(i) = step;
        plus = mixmpc_full_dynamics(model,q+dq,v);
        minus = mixmpc_full_dynamics(model,q-dq,v);
        delta = (plus.energy-minus.energy)/(2*step);
        grad_T(i) = delta(1);
        grad_U(i,:) = delta(2:3)';
    end
    p = mixmpc_full_dynamics(model,q+step*v,v);
    m = mixmpc_full_dynamics(model,q-step*v,v);
    lagrange = (p.M-m.M)*v/(2*step)-grad_T+grad_U(:,1);
    convective_error = norm(lagrange-d.h,Inf)/max(1,norm(d.h,Inf));
    spring_error = norm(d.Qspring+grad_U(:,2),Inf)/max(1,norm(d.Qspring,Inf));
    c = mixmpc_contact_model(model,d.kin,terrain);
    cp = mixmpc_contact_model(model,p.kin,terrain);
    cm = mixmpc_contact_model(model,m.kin,terrain);
    gap_error = norm((cp.gap-cm.gap)/(2*step)-c.velocity(3,:)',Inf);
    bias_error = norm((cp.J-cm.J)*v/(2*step)-c.bias,Inf);
    work_error = virtual_power(model,d,p,m,v,step);
    loop_pos = norm(d.kin.loop_position(:),Inf);
    loop_vel = norm(d.kin.loop_velocity(:),Inf);
    loop_accel = norm(d.kin.loop_acceleration(:),Inf);
    checks = [m_err g_err com_err kkt_err loop_power convective_error spring_error ...
        gap_error bias_error work_error loop_vel loop_accel];
    names = {'mass','gravity','COM','KKT','loop power','Lagrange bias','spring gradient', ...
        'gap derivative','contact bias','virtual power','loop velocity','loop acceleration'};
    [worst,i] = max(checks);
    assert(worst<tol.jacobian,'Independent %s check failed at pose %d (%.9g).',names{i},k,worst);
    assert(loop_pos<tol.loop_m,'Closed-chain position drift.');
    leg_length = zeros(2,1);
    for side = 1:2
        length_angle = polar_output(d.kin,robot.base,robot.side(side).frames([1 11]));
        leg_length(side) = length_angle(1);
    end
    records{k} = struct('q',q,'open_tree_mass_error',m_err,'open_tree_gravity_error',g_err, ...
        'open_tree_com_error_m',com_err,'reduced_vs_open_kkt_error',kkt_err, ...
        'loop_reaction_power_W',loop_power,'lagrange_bias_relative_error',convective_error, ...
        'spring_gradient_relative_error',spring_error,'gap_derivative_error_mps',gap_error, ...
        'contact_bias_error_mps2',bias_error,'virtual_power_relative_error',work_error, ...
        'loop_position_error_m',loop_pos,'loop_velocity_error_mps',loop_vel, ...
        'loop_acceleration_error_mps2',loop_accel,'spring_pin_distance_m',d.spring_length, ...
        'leg_length_m',leg_length,'min_joint_margin_rad',min(d.joint_margin));
end
errors = [records{:}];
geometry = struct('coordinates',12,'tree_coordinates',20,'independent_loop_constraints',8, ...
    'min_mass_eigenvalue',min(span),'spring_mount','CAD pin pair on thigh and shank');
end

function relative = virtual_power(model,d,p,m,v,step)
F = [37;-1.6];
relative = 0;
base = model.robot.base;
for side = 1:2
    frames = model.robot.side(side).frames([1 11]);
    R = d.kin.pose.R(:,:,base);
    displacement = R'*(d.kin.pose.p(:,frames(2))-d.kin.pose.p(:,frames(1)));
    idx = 6+(2*side-1:2*side);
    J = R'*(d.kin.Jv(:,idx,frames(2))-d.kin.Jv(:,idx,frames(1)));
    xz = displacement([1 3]);
    L = norm(xz);
    V = [xz'/L; [-xz(2) xz(1)]/L^2]*J([1 3],:);
    joint_power = (V'*F)'*v(idx);
    fd = (polar_output(p.kin,base,frames)-polar_output(m.kin,base,frames))/(2*step);
    task_power = F'*fd;
    relative = max(relative,abs(joint_power-task_power)/max(1,abs(task_power)));
end
end

function task = polar_output(kin,base,ids)
r = kin.pose.R(:,:,base)'*(kin.pose.p(:,ids(2))-kin.pose.p(:,ids(1)));
task = [hypot(r(1),r(3)); atan2(r(1),-r(3))];
end

function [metrics,traces] = verify_contact(model,tol)
terrain = floor_planes();
zero = zeros(12,1);
hip = fzero(@longitudinal_balance,[-0.25 0.25]);
q_stand = symmetric_stance(hip);
q_stand = ground_on(q_stand,1);
d_stand = mixmpc_full_dynamics(model,q_stand,zero);
c_stand = mixmpc_contact_model(model,d_stand.kin,terrain);
weight = -sum([model.robot.links.mass])*model.gravity(3);
tau0 = stand_torque(q_stand,terrain);
stand = mixmpc_reference_plant(model,q_stand,zero,tau0,terrain);
assert(strcmp(stand.mode,'DS') && strcmp(stand.status,'ok') && stand.contact_rank==5, ...
    'Symmetric double support must keep rank five with both wheels loaded.');
assert(norm(stand.qdd,Inf)<tol.equations, 'CAD static stance accelerates.');
static_moment = sum(cross(c_stand.point-d_stand.com,stand.reaction_world,1),2);
assert(norm(sum(stand.reaction_world,2)-[0;0;weight],Inf)<tol.equations && ...
    norm(static_moment,Inf)<tol.equations, 'Static force or moment balance failed.');
metrics.ds_static = struct('normal_force_N',stand.reaction(3,:), ...
    'acceleration_error',norm(stand.qdd,Inf),'moment_error_Nm',norm(static_moment,Inf));

v_roll = zero;
v_roll(1) = 0.22;
for side = 1:2
    r = 3*side-2;
    v_roll(10+side) = -c_stand.J(r,:)*v_roll/c_stand.J(r,10+side);
end
initial_roll = mixmpc_reference_plant(model,q_stand,v_roll,tau0,terrain);
assert(strcmp(initial_roll.status,'ok') && initial_roll.contact_rank==5, ...
    'The straight rolling velocity must be admissible at the symmetric pose.');
% Preserve the response when exact wheel inertias drive this configuration
% towards a change of rolling rank. Closely spaced constraints can demand
% a wrench outside the friction cones; that cannot be reported as rolling.
traces.ds_symmetric_roll = mixmpc_simulate_reference(model,q_stand,v_roll,tau0,terrain,0.04,0.001);
assert(any(strcmp(traces.ds_symmetric_roll.status, ...
    {'ok','sticking_feasibility_lost','contact_rank_incompatible'})), ...
    'The symmetric rolling diagnostic stopped for an unexpected reason.');
metrics.ds_symmetric_roll = check_constraints(traces.ds_symmetric_roll,tol);
metrics.ds_symmetric_roll.status = traces.ds_symmetric_roll.status;
metrics.ds_symmetric_roll.stop_time_s = traces.ds_symmetric_roll.stop_time;
metrics.ds_symmetric_roll.failure = traces.ds_symmetric_roll.failure;

% Keep a fore-aft wheel offset on level ground for the regular-rank check.
% The lateral rolling rows are independent at this pose.
q_roll = q_stand;
q_roll(9:10) = q_roll(9:10)+0.06;
pitch = fzero(@(p) rolling_balance(p,q_roll),[-0.25 0.25]);
[~,q_roll] = rolling_balance(pitch,q_roll);
d_roll = mixmpc_full_dynamics(model,q_roll,zero);
c_roll = mixmpc_contact_model(model,d_roll.kin,terrain);
tau_roll = stand_torque(q_roll,terrain);
v_roll(1:3) = 0.22*c_roll.basis(:,1,1);
for side = 1:2
    r = 3*side-2;
    v_roll(10+side) = -(c_roll.J(r,1:3)*v_roll(1:3))/c_roll.J(r,10+side);
end
start = mixmpc_reference_plant(model,q_roll,v_roll,tau_roll,terrain);
assert(strcmp(start.mode,'DS') && strcmp(start.status,'ok') && start.contact_rank==6, ...
    'The rolling test pose must have two admissible wheel contacts (%s, rank %d).',start.status,start.contact_rank);
traces.ds_roll = mixmpc_simulate_reference(model,q_roll,v_roll,tau_roll,terrain,0.025,0.001);
metrics.ds_roll = check_trace(traces.ds_roll,'DS',tol);
metrics.ds_roll.forward_travel_m = traces.ds_roll.q(1,end)-q_roll(1);
assert(metrics.ds_roll.forward_travel_m>0.004 && all(abs(traces.ds_roll.q(11:12,end))>0.05), ...
    'Wheel-center translation and wheel rotation must persist during rolling.');
traces.ds_roll_coarse = mixmpc_simulate_reference(model,q_roll,v_roll,tau_roll,terrain,0.025,0.002);
check_trace(traces.ds_roll_coarse,'DS',tol);
travel = traces.ds_roll_coarse.q(1,end)-q_roll(1);
metrics.ds_roll.forward_travel_step_delta_fraction = abs(travel-metrics.ds_roll.forward_travel_m)/ ...
    metrics.ds_roll.forward_travel_m;
assert(metrics.ds_roll.forward_travel_step_delta_fraction<tol.convergence_fraction, ...
    'Constrained rolling did not converge under step refinement.');

% Higher wheel rests on a separate horizontal tread, with both coordinates
% assigned independently as prescribed plane heights.
q_asym = q_roll;
q_asym([4 6 8 10]) = q_asym([4 6 8 10])+[0.026;0.11;-0.025;0.017];
q_asym(5) = fzero(@(p) pitch_offset(p,q_asym),[-0.25 0.25]);
q_asym = ground_on(q_asym,1);
d_offset = mixmpc_full_dynamics(model,q_asym,zero);
c_offset = mixmpc_contact_model(model,d_offset.kin,terrain);
staggered = terrain;
staggered.origin(3,2) = c_offset.gap(2);
assert(staggered.origin(3,2)>0.005, 'The offset case must have separate tread heights.');
tau_asym = stand_torque(q_asym,staggered)+[-0.10;0.06;0.08;-0.04;0.015;0.008];
sample = mixmpc_reference_plant(model,q_asym,zero,tau_asym,staggered);
assert(strcmp(sample.status,'ok') && sample.contact_rank==6 && strcmp(sample.mode,'DS'), ...
    'Offset dual support must retain six independent velocity constraints (%s).',sample.status);
traces.ds_offset = mixmpc_simulate_reference(model,q_asym,zero,tau_asym,staggered,0.02,0.001);
metrics.ds_offset = check_trace(traces.ds_offset,'DS',tol);
metrics.ds_offset.plane_offset_m = staggered.origin(3,2)-staggered.origin(3,1);

names = {'LS','RS'};
fields = {'left','right'};
for side = 1:2
    swing = 3-side;
    q = q_stand;
    q(4) = (-1)^(side+1)*0.055;
    q = ground_on(q,side);
    tau = tau0;
    tau(4+swing) = 0.07;
    sample = mixmpc_reference_plant(model,q,zero,tau,terrain);
    assert(sample.contact_rank==3 && strcmp(sample.mode,names{side}) && ...
        abs(sample.qdd(10+swing))>1 && all(sample.reaction(:,swing)==0), ...
        'The swing wheel lost its independent spin or carried a reaction.');
    one = mixmpc_simulate_reference(model,q,zero,tau,terrain,0.02,0.001);
    traces.(fields{side}) = one;
    metrics.(fields{side}) = check_trace(one,names{side},tol);
    assert(min(one.gap(swing,:))>0.005,'Swing wheel clearance was lost in the fixed-mode test.');
    assert(abs(one.v(10+swing,end))>0.1,'Torque did not accelerate the free wheel.');
end

% Agreement of step-halved RK4 and adaptive ode45 for the same mechanical
% configuration and forces, plus conservation in a true free-flight case.
q_free = q_asym;
q_free(3) = q_free(3)+0.15;
v_free = [0.2;-0.12;0.3;0.4;-0.2;0.5;0.6;-0.1;0.3;0.2;2;-1];
free = model;
free.gravity(:) = 0;
dt = 0.004;
traces.free_coarse = mixmpc_simulate_reference(free,q_free,v_free,zeros(6,1),terrain,0.048,dt);
traces.free_fine = mixmpc_simulate_reference(free,q_free,v_free,zeros(6,1),terrain,0.048,dt/2);
metrics.free_flight = check_trace(traces.free_fine,'FLIGHT',tol);
coarse = traces.free_coarse;
fine = traces.free_fine;
y_end = [fine.q(:,end);fine.v(:,end)];
ode_options = odeset('RelTol',1e-10,'AbsTol',1e-12);
[~,ref] = ode45(@(~,y) derivative(free,y,zeros(6,1),terrain,[false false]), ...
    [0 fine.t(end)],[q_free;v_free],ode_options);
coarse_error = norm([coarse.q(:,end);coarse.v(:,end)]-ref(end,:)',Inf);
fine_error = norm(y_end-ref(end,:)',Inf);
energy_drift = max(abs(sum(fine.energy)-sum(fine.energy(:,1))));
linear_drift = max(vecnorm(fine.linear_momentum-fine.linear_momentum(:,1)));
angular_drift = max(vecnorm(fine.angular_momentum_com-fine.angular_momentum_com(:,1)));
assert(strcmp(coarse.status,'ok') && fine_error<coarse_error/4 && fine_error<tol.convergence_fraction && ...
    energy_drift<tol.energy_J && max(linear_drift,angular_drift)<tol.momentum, ...
    'Free-flight conservation or step-size convergence failed.');
metrics.free_flight.rk4_endpoint_error = [coarse_error fine_error];
metrics.free_flight.energy_drift_J = energy_drift;
metrics.free_flight.linear_momentum_drift = linear_drift;
metrics.free_flight.angular_momentum_drift = angular_drift;

% Low-speed, event-detected touchdown including both stance sides. Construct
% an admissible incoming state from an independent prescribed impulse, then
% integrate the single-support dynamics backwards to seed the test.
q_land = zero;
q_land(7:10) = [0.35;-0.42;-0.43;0.34];
pitch = fzero(@(p) rolling_balance(p,q_land),[-0.25 0.25]);
[~,q_land] = rolling_balance(pitch,q_land);
tau_land = stand_torque(q_land,terrain);
d_land = mixmpc_full_dynamics(model,q_land,zero);
c_land = mixmpc_contact_model(model,d_land.kin,terrain);
for side = 1:2
    [incoming,impulse] = touchdown_fixture(d_land.M,c_land,side);
    support = (1:2)==side;
    [~,approach] = ode45(@(~,y) derivative(model,y,tau_land,terrain,support), ...
        [0 -0.00025],[q_land;incoming],ode_options);
    y = approach(end,:)';
    landed = mixmpc_simulate_reference(model,y(1:12),y(13:24),tau_land,terrain,0.008,0.0001);
    traces.(['land_' fields{side}]) = landed;
    metrics.(['land_' fields{side}]) = check_impact(landed,names{side},tol,terrain);
    impulse_error = norm(landed.events.impulse(:)-impulse,Inf);
    assert(impulse_error<tol.equations && norm(landed.events.v_after,Inf)<tol.equations, ...
        'Event impulse differs from independent touchdown fixture.');
    metrics.(['land_' fields{side}]).fixture_impulse_error_Ns = impulse_error;
end

% A torque change at a stationary double support pose isolates the release
% guard. Contact mode is determined by the required normal force.
w0 = stand.reaction(:);
response = zeros(6);
units = eye(6);
for i = 1:6
    sample = mixmpc_reference_plant(model,q_stand,zero,tau0+units(:,i),terrain,[true true]);
    response(:,i) = sample.reaction(:)-w0;
end
for side = 1:2
    swing = 3-side;
    wrench = zeros(3,2);
    wrench(3,side) = weight;
    wrench(3,swing) = -2;
    change = pinv(response,1e-8)*(wrench(:)-w0);
    pushed = mixmpc_reference_plant(model,q_stand,zero,tau0+change,terrain);
    assert(strcmp(pushed.status,'ok') && strcmp(pushed.mode,names{side}) && ...
        pushed.contact_acceleration(3,swing)>tol.equations, 'Unloading did not select unilateral support.');
    input = @(t,~,~) tau0+(t>=0.0065)*change;
    released = mixmpc_simulate_reference(model,q_stand,zero,input,terrain,0.014,0.001);
    field = ['lift_' fields{side}];
    traces.(field) = released;
    metrics.(field) = check_release(released,names{side},swing,tol);
end

low_mu = terrain;
low_mu.mu(:) = 0;
bad = mixmpc_reference_plant(model,q_stand,zero,tau0+[0;0;0;0;0.4;-0.4],low_mu);
assert(strcmp(bad.status,'no_admissible_rolling_mode'), 'Infeasible friction was treated as rolling.');
metrics.friction_reject = bad.status;

    function out = symmetric_stance(angle)
        out = zero;
        out(7:10) = angle*[1;1;-1;-1];
    end

    function offset = longitudinal_balance(angle)
        d = mixmpc_full_dynamics(model,symmetric_stance(angle),zero);
        offset = d.com(1)-mean(d.kin.pose.p(1,model.wheel_link));
    end

    function aligned = ground_on(q,side)
        d = mixmpc_full_dynamics(model,q,zero);
        c = mixmpc_contact_model(model,d.kin,terrain);
        aligned = q;
        aligned(3) = q(3)-c.gap(side);
    end

    function [offset,q] = rolling_balance(pitch,q)
        q(5) = pitch;
        q(4) = fzero(@(angle) roll_gap(angle,q),[-0.1 0.1]);
        q = ground_on(q,1);
        offset = moment_offset(q);
    end

    function gap = roll_gap(angle,q)
        q(4) = angle;
        d = mixmpc_full_dynamics(model,q,zero);
        c = mixmpc_contact_model(model,d.kin,terrain);
        gap = c.gap(2)-c.gap(1);
    end

    function offset = moment_offset(q)
        d = mixmpc_full_dynamics(model,q,zero);
        c = mixmpc_contact_model(model,d.kin,terrain);
        cross_track = cross([0;0;1],c.point(:,2)-c.point(:,1));
        offset = dot(cross_track,d.com-c.point(:,1))/norm(cross_track);
    end

    function offset = pitch_offset(pitch,q)
        q(5) = pitch;
        offset = moment_offset(q);
    end

    function tau = stand_torque(q,planes)
        d = mixmpc_full_dynamics(model,q,zero);
        c = mixmpc_contact_model(model,d.kin,planes);
        D = c.point(1:2,2)-c.point(1:2,1);
        right_load = weight*dot(D,d.com(1:2)-c.point(1:2,1))/dot(D,D);
        f = c.J'*[0;0;weight-right_load;0;0;right_load];
        tau = d.h(7:12)-d.Qspring(7:12)-f(7:12);
    end

end

function dy = derivative(model,y,tau,terrain,support)
p = mixmpc_reference_plant(model,y(1:12),y(13:24),tau,terrain,support);
dy = [y(13:24);p.qdd];
end

function [v,impulse] = touchdown_fixture(M,c,side)
stance = 3*(side-1)+(1:3);
swing = 3*(2-side)+(1:3);
projected_mass = c.J*(M\c.J');
coupling = -(projected_mass(stance,stance)\projected_mass(stance,swing));
% Fix the swing normal impulse to one unit and choose tangential impulses
% with interior Coulomb slack on both sides.
[x,y] = meshgrid(linspace(-c.mu(3-side),c.mu(3-side),41));
swing_impulses = [x(:)';y(:)';ones(1,numel(x))];
stance_impulses = coupling*swing_impulses;
margin = min([stance_impulses(3,:); ...
    c.mu(side)*stance_impulses(3,:)-vecnorm(stance_impulses(1:2,:)); ...
    c.mu(3-side)-vecnorm(swing_impulses(1:2,:))],[],1);
[best,index] = max(margin);
assert(best>0, 'Test pose has no interior touchdown impulse for side %d (margin %.6g).',side,best);
impulse = zeros(6,1);
impulse(stance) = stance_impulses(:,index);
impulse(swing) = swing_impulses(:,index);
v = -(M\(c.J'*impulse));
speed = -(c.J(swing(3),:)*v);
assert(speed>0 && norm(c.J(stance,:)*v,Inf)<1e-10, 'Invalid pre-touchdown velocity.');
v = v*(0.075/speed);
impulse = impulse*(0.075/speed);
end

function metrics = check_trace(sim,mode,tol)
assert(strcmp(sim.status,'ok'), '%s integration stopped at %.6f s (%s).',mode,sim.stop_time,sim.status);
assert(all(strcmp(sim.mode,mode)), 'Fixed-mode test switched contacts.');
metrics = check_constraints(sim,tol);
work = cumtrapz(sim.t,sum(sim.tau.*sim.v(7:12,:),1));
energy = sum(sim.energy,1);
metrics.max_drive_work_error_J = max(abs(energy-energy(1)-work));
assert(metrics.max_drive_work_error_J<tol.energy_J, 'Drive work and mechanical energy do not balance.');
end

function metrics = check_constraints(sim,tol)
reaction_normal = sim.reaction(3,:,:);
metrics = struct('max_equation_error',max(sim.equation_error),'max_accel_constraint_error',max(sim.constraint_error), ...
    'max_loop_error_m',max(sim.loop_error),'min_gap_m',min(sim.gap,[],'all'), ...
    'min_normal_force_N',min(reaction_normal,[],'all'),'min_friction_margin_N',min(sim.friction_margin), ...
    'min_cad_joint_margin_rad',min(sim.joint_margin),'contact_rank',unique(sim.contact_rank));
assert(metrics.max_equation_error<tol.equations && metrics.max_accel_constraint_error<tol.equations && ...
    metrics.max_loop_error_m<tol.loop_m && metrics.min_gap_m>=-tol.gap_m && ...
    metrics.min_normal_force_N>=-tol.negative_force_N && metrics.min_friction_margin_N>=-tol.negative_force_N, ...
    'Trajectory violates contact or dynamics checks.');
for i = 1:numel(sim.t)
    active = [strcmp(sim.mode{i},'LS') strcmp(sim.mode{i},'RS')] | strcmp(sim.mode{i},'DS');
    assert(all(abs(sim.gap(active,i))<=tol.gap_m) && all(sim.contact_speed(active,i)<=tol.contact_speed_mps), ...
        'Active contact drifts from its plane or rolls with slip.');
end
end

function metrics = check_impact(sim,from,tol,terrain)
assert(strcmp(sim.status,'ok'), 'Transition %s stopped at %.6f s (%s, gap %s m).', ...
    from,sim.stop_time,sim.status,mat2str(sim.gap(:,end),4));
assert(strcmp(sim.mode{1},from) && strcmp(sim.mode{end},'DS'), 'Touchdown mode trace is inconsistent.');
assert(numel(sim.events)==1, 'Expected one isolated contact event for %s.',from);
event = sim.events;
assert(strcmp(event.kind,'touchdown') && strcmp(event.before,from) && strcmp(event.after,'DS'), ...
    'Unexpected transition %s -> %s.',event.before,event.after);
impulse = event.impulse;
assert(event.kinetic_loss>=0 && event.balance_residual<tol.equations && ...
    all(impulse(3,:)>=0) && all(vecnorm(impulse(1:2,:)) <= terrain.mu'.*impulse(3,:)+1e-7), ...
    'Touchdown impulse violates momentum, dissipation, or the Coulomb cone.');
metrics = check_constraints(sim,tol);
metrics.event_time_s = event.time;
metrics.energy_loss_J = event.kinetic_loss;
metrics.impulse_Ns = impulse;
metrics.momentum_reset_residual = event.balance_residual;
end

function metrics = check_release(sim,to,swing,tol)
assert(strcmp(sim.status,'ok'), 'Release to %s stopped at %.6f s (%s).',to,sim.stop_time,sim.status);
assert(numel(sim.events)==1 && strcmp(sim.events.kind,'liftoff') && ...
    strcmp(sim.events.before,'DS') && strcmp(sim.events.after,to), ...
    'Expected a single release from DS to %s.',to);
event = sim.events;
released = sim.t>=event.time;
assert(all(event.v_before==event.v_after) && all(event.impulse==0,'all') && ...
    all(sim.reaction(:,swing,released)==0,'all') && sim.gap(swing,end)>0, ...
    'Liftoff should keep continuous velocity and have positive swing gap.');
metrics = check_constraints(sim,tol);
metrics.event_time_s = event.time;
metrics.final_swing_gap_m = sim.gap(swing,end);
end

function t = floor_planes()
t = struct('origin',zeros(3,2),'normal',[0 0;0 0;1 1],'mu',[0.6;0.6]);
end
