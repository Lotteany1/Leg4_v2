function audit = mixmpc_height_roll_audit(urdf_or_robot, heights, hardware)
% 完整 CAD 双支撑升降/横滚核算。H 是左右虚腿竖直投影的平均，roll 正向抬高逻辑左侧。
% 单独调用 mixmpc_height_roll_audit('.../chuanliantui.urdf', [0.27 0.30])。
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(fileparts(fileparts(mfilename('fullpath'))));
mixmpc_paths();
if isstruct(urdf_or_robot), robot = urdf_or_robot; else, robot = mixmpc_robot_config(urdf_or_robot); end
if nargin<3, [~,hw] = leg_config(); else, hw=hardware; end
if nargin<2, heights = [.27 0.30 0.32 hw.Len_max]; end
validateattributes(heights,{'double'},{'real','finite','positive','vector','nonempty'});
hip = arrayfun(@(s)s.frames(1),robot.side);
wheel = arrayfun(@(s)s.frames(11),robot.side);
hip_y = robot.zero.p(2,hip);
axle_y = robot.zero.p(2,wheel);
% 固连在 base 的标记按实际关节树归入机身。活动支链从两个同轴驱动开始。
group = ones(1,numel(robot.links));
for j = robot.joints
    group(j.child) = group(j.parent);
    for side = 1:2
        if ismember(j.child,robot.side(side).frames([1 3])), group(j.child) = side+1; end
        if j.child==wheel(side), group(j.child) = side+3; end
    end
end
mass = accumarray(group(:),[robot.links.mass]',[5 1]);
installed_mass = [hw.m_body;hw.m_leg;hw.m_leg;hw.m_wheel;hw.m_wheel];
scale = installed_mass./mass;
robot=mixmpc_phase_average_wheels(robot);
for side = 1:2
    % 轮相位在 14 态中不可观测。先平移到轮轴再作相位平均，最后移回平均质心。
    id = wheel(side);
    link = robot.links(id);
    joint = robot.joints([robot.joints.child]==id);
    axis = joint.axis;
    along = axis*axis';
    across = eye(3)-along;
    mesh = stlread(fullfile(robot.mesh_dir,link.mesh));
    vertex = link.mesh_xyz + link.mesh_R*mesh.Points';
    radius = vecnorm(across*vertex);
    tread = vertex(:,max(radius)-radius<=1e-4);
    tread = robot.zero.p(:,id)+robot.zero.R(:,:,id)*tread;
    tread_y = tread(2,:)-axle_y(side);
    tread_center(side) = axle_y(side)+(min(tread_y)+max(tread_y))/2; %#ok<AGROW>
    outer_radius(side) = max(radius); %#ok<AGROW>
end
cad = mixmpc_reference_config(robot,hw,false);
installed = cad;
installed.wheel_radius(:) = hw.wheel_radius;
for k = 1:numel(robot.links)
    factor = scale(group(k));
    installed.robot.links(k).mass = robot.links(k).mass*factor;
    installed.robot.links(k).inertia = robot.links(k).inertia*factor;
end
audit = struct('source_sha256',tinympc14_sha256(robot.urdf),'mass_groups', ...
    {{'body','left_leg','right_leg','left_wheel','right_wheel'}},'cad_mass_kg',mass, ...
    'installed_mass_kg',installed_mass,'hip_half_span_m',abs(diff(hip_y))/2, ...
    'axle_origin_half_span_m',abs(diff(axle_y))/2,'tread_center_half_span_m',abs(diff(tread_center))/2, ...
    'mesh_outer_radius_m',outer_radius,'cad_contact_radius_m',cad.wheel_radius(1), ...
    'installed_radius_m',hw.wheel_radius,'installed_half_track_m',hw.half_track);
for k = 1:numel(heights)
    q = vertical_pose(cad,hip,wheel,heights(k));
    dyn = mixmpc_full_dynamics(cad,q,zeros(12,1));
    knee = arrayfun(@(s)dyn.kin.joint_position(s.passive(1)),robot.side);
    limits = vertcat(robot.side.knee_limits);
    audit.points(k).height_m = heights(k);
    audit.points(k).knee_margin_rad = min(knee(:)-limits(:,1),limits(:,2)-knee(:));
    audit.points(k).cad = project(cad,dyn,hip,wheel,axle_y-hip_y,heights(k));
    audit.points(k).installed_estimate = project(installed,mixmpc_full_dynamics(installed,q,zeros(12,1)), ...
        hip,wheel,axle_y-hip_y,heights(k));
end
end

function q = vertical_pose(model,hip,wheel,height)
% 从 CAD 零位连续牛顿求解闭链轮心相对髋的矢状面位置；保留可达域外的膝限位余量。
q = zeros(12,1);
for i = 1:40
    kin = mixmpc_full_kinematics(model,q,zeros(12,1));
    offset = kin.pose.p([1 3],wheel)-kin.pose.p([1 3],hip);
    offset(2,:) = offset(2,:) + height;
    if norm(offset(:),Inf)<1e-12, return, end
    jacobian = kin.Jv([1 3],7:10,wheel)-kin.Jv([1 3],7:10,hip);
    jacobian = reshape(permute(jacobian,[1 3 2]),4,4);
    step = jacobian\offset(:);
    q(7:10) = q(7:10)-step/max(1,norm(step,Inf)/0.2);
end
error('The requested vertical pose did not converge at lengths %s m.',mat2str(height));
end

function result = project(model,dyn,hip,wheel,dy,height)
% 单位 Hdot / rolldot 对应的两个速度列 N：动能为 0.5 * v'*(N'*M*N)*v。
kin = dyn.kin;
axes = eye(12);
hip_J = mean(kin.Jv(:,:,hip),3);
relative = kin.Jv(:,:,wheel)-kin.Jv(:,:,hip);
% 虚腿不含 URDF 轮轴相对髋的侧向安装偏移。H 的竖直投影去掉此项。
height_J = -mean(relative(3,:,:),3) + mean(dy)*axes(4,:);
terrain = struct('origin',zeros(3,2),'normal',repmat([0;0;1],1,2),'mu',[0.6;0.6]);
contact = mixmpc_contact_model(model,kin,terrain);
% 两腿保持竖直，轮无前向滚动；左右在 CAD Y 轴的位置决定横滚映射。
constraints = [hip_J(1:2,:); axes(5:6,:); squeeze(relative(1,:,:))'; ...
    contact.J([3 6],:); squeeze(kin.Jw(2,:,wheel))'; height_J; axes(4,:)];
roll_sign = sign(diff(model.robot.zero.p(2,hip([2 1]))));
target = [zeros(10,2); 1 0; 0 roll_sign];
for boundary = ["fixed_hip_xy","no_lateral_slip"]
    if boundary=="no_lateral_slip"
        constraints(2,:) = mean(contact.J([2 5],:),1);
    end
    assert(rcond(constraints)>1e-9,'Singular height/roll tangent: %s',mat2str(svd(constraints)',4));
    N = constraints\target;
    if boundary=="no_lateral_slip"
        assert(norm(contact.J([2 5],:)*N,Inf)<1e-8,'Inconsistent bilateral lateral constraints.');
    end
    result.(boundary).mass_matrix = N'*dyn.M*N;
    result.(boundary).tangent = N;
    % 顺序 [H, roll]：M 的对角量为 kg / kg m²；重力项为 N / N m。
    result.(boundary).generalized_gravity = N'*dyn.gravity;
end
% 两种边界仅水平平移不同，势能相同。沿两轮贴地、固定 H 的曲线独立差分验证。
delta = 5e-4; % rad
U = zeros(1,3);
hip_mid = mean(model.robot.zero.p(:,hip),2);
axle_y = zeros(1,2);
for side = 1:2
    id = wheel(side);
    tire = model.robot.zero.p(:,id)+model.robot.zero.R(:,:,id)*model.wheel_center_offset(:,side);
    axle_y(side) = tire(2)-hip_mid(2);
end
for j = 1:3
    angle = (j-2)*delta;
    c = cos(angle);
    s = sin(roll_sign*angle);
    radius = model.wheel_radius(1);
    lengths = (height + (axle_y-mean(axle_y))*s)/c;
    q = vertical_pose(model,hip,wheel,lengths);
    R = [1 0 0;0 c -s;0 s c];
    q(4) = roll_sign*angle;
    q(1:3) = [0;0;height+radius*c-mean(axle_y)*s]-R*hip_mid;
    tilted = mixmpc_full_dynamics(model,q,zeros(12,1));
    U(j) = tilted.energy(2);
end
gravity = result.fixed_hip_xy.generalized_gravity;
assert(abs((U(3)-U(1))/(2*delta)-gravity(2))<1e-5,'Roll gravity finite difference failed.');
curvature = (U(1)-2*U(2)+U(3))/delta^2;
result.roll_potential_curvature_Nm_per_rad = curvature;
% F_i 为作功于虚腿长度的常值轴力时，L_L''=L_R''=H（roll=0）；此项与重力势能分列。
preload = height*gravity(1);
result.constant_axial_preload_torque_slope_Nm_per_rad = preload;
result.thin_wheel_tip_slope_Nm_per_rad = preload-curvature;
% 若轮轴保持固定离地高度，整车相对薄轮曲线增加 R*(1-cos(roll)) 的刚体平移。
delta_curvature = sum([model.robot.links.mass])*norm(model.gravity)*radius;
result.fixed_axle_height_tip_slope_Nm_per_rad = preload-curvature-delta_curvature;
end
