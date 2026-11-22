function result = verify_001_physics()
% 独立 CAD 物理校核：闭环孔位、惯量合法性、动能、重力虚功、静态整车质心。
mixmpc_paths(); p=tinympc14_config(); [~,hw]=leg_config();
repo=fileparts(fileparts(fileparts(mfilename('fullpath')))); repo=fileparts(repo);
r=mixmpc_robot_config(fullfile(repo,'001','001','urdf','001.urdf'));
m=mixmpc_reference_config(r,hw);
folder=fullfile(repo,'research','urdf_migration_20261122');
normalized=mixmpc_normalized_urdf(r,folder);
tree=importrobot(normalized,'DataFormat','struct');
home=homeConfiguration(tree); moving=r.joints([r.joints.q]>0);
[matched,order]=ismember({home.JointName},{moving.name});
assert(all(matched) && numel(order)==r.nq,'Independent normalized joint mapping failed');
tree.DataFormat='column';
tree_mass=sum(cellfun(@(b)b.Mass,tree.Bodies));
base=r.links(r.base); missing_base=~ismember(base.name,tree.BodyNames);
result.mechanism=mixmpc_mechanism_audit(r);
assert(abs(result.mechanism.total_mass_kg-hw.m_total)<1e-10,'001 total mass mismatch');
hip=arrayfun(@(s)s.frames(1),r.side);
wheel=arrayfun(@(s)s.frames(11),r.side);
energy_error=0; gravity_error=0; contact_error=0; cg_error=0;
tree_mass_error=0; tree_gravity_error=0; tree_com_error=0;
yaw_inertia_error=0; pitch_inertia_error=0; wheel_inertia_error=0;
yaw_table_error=0;
yaw_model=m; yaw_model.robot=mixmpc_phase_average_wheels(r);
heights=linspace(.17,hw.Len_max,9);
audit=mixmpc_height_roll_audit(r,heights);
matrix_error=0; gravity_table_error=0;
for k=1:numel(heights)
    H=heights(k); q=mixmpc_vertical_pose(m,H);
    tangent=audit.points(k).cad.no_lateral_slip.tangent;
    v=tangent*[.31;-.23];
    dyn=mixmpc_full_dynamics(m,q,v);
    R=dyn.kin.pose.R(:,:,r.base);
    tree.Gravity=(R'*m.gravity)';
    positions=dyn.kin.joint_position(order);
    ids=6+order;
    tree_mass_error=max(tree_mass_error,norm(dyn.open_M(ids,ids)-massMatrix(tree,positions),'fro'));
    tree_gravity_error=max(tree_gravity_error,norm(dyn.open_gravity(ids)-gravityTorque(tree,positions),Inf));
    tree_com=centerOfMass(tree,positions);
    tree_com=(tree_mass*tree_com(:)+missing_base*base.mass*base.com)/hw.m_total;
    tree_com_error=max(tree_com_error,norm(dyn.com-q(1:3)-R*tree_com,Inf));
    assert(min(eig(dyn.M))>0,'Full CAD mass matrix is not positive definite');
    step=1e-6;
    a=mixmpc_full_kinematics(m,q+step*v,zeros(12,1));
    b=mixmpc_full_kinematics(m,q-step*v,zeros(12,1));
    kinetic=0; Ua=0; Ub=0;
    for id=1:numel(r.links)
        link=r.links(id); R=dyn.kin.pose.R(:,:,id);
        ca=a.pose.p(:,id)+a.pose.R(:,:,id)*link.com;
        cb=b.pose.p(:,id)+b.pose.R(:,:,id)*link.com;
        speed=(ca-cb)/(2*step);
        rotation=(a.pose.R(:,:,id)-b.pose.R(:,:,id))/(2*step)*R';
        spin=[rotation(3,2);rotation(1,3);rotation(2,1)];
        kinetic=kinetic+.5*link.mass*(speed'*speed)+.5*spin'*(R*link.inertia*R')*spin;
        Ua=Ua-link.mass*m.gravity'*ca; Ub=Ub-link.mass*m.gravity'*cb;
    end
    energy_error=max(energy_error,abs(kinetic-.5*v'*dyn.M*v));
    gravity_error=max(gravity_error,abs((Ua-Ub)/(2*step)-v'*dyn.gravity));
    terrain=struct('origin',zeros(3,2),'normal',repmat([0;0;1],1,2),'mu',[.6;.6]);
    contact=mixmpc_contact_model(m,dyn.kin,terrain);
    contact_error=max(contact_error,norm(contact.J([2 3 5 6],:)*v,Inf));
    tire=zeros(3,2);
    for side=1:2
        id=wheel(side);
        tire(:,side)=dyn.kin.pose.p(:,id)+dyn.kin.pose.R(:,:,id)*m.wheel_center_offset(:,side);
    end
    center=mean(tire,2); yaw_velocity=zeros(12,1);
    yaw_velocity(1:3)=-cross([0;0;1],center); yaw_velocity(6)=1;
    for side=1:2
        row=3*side-2;
        yaw_velocity(10+side)=-contact.J(row,:)*yaw_velocity/contact.J(row,10+side);
    end
    yaw_dyn=mixmpc_full_dynamics(yaw_model,q,zeros(12,1));
    left=tinympc14_table_row(p.YAW_DATA,H); right=left;
    frozen=yaw_velocity; frozen(11:12)=0;
    locked=frozen'*yaw_dyn.M*frozen;
    expected_yaw=locked+p.I_w*sum(yaw_velocity(11:12).^2);
    yaw_inertia_error=max(yaw_inertia_error,abs(yaw_velocity'*yaw_dyn.M*yaw_velocity-expected_yaw));
    yaw_table_error=max(yaw_table_error,abs(locked-p.I_z_body-left(1)-right(2))/locked);
    pitch_velocity=zeros(12,1); midpoint=mean(dyn.kin.pose.p(:,hip),2);
    pitch_velocity(1:3)=-cross([0;1;0],midpoint);
    pitch_velocity(5)=1;
    pitch_velocity(7:10)=1;
    pitch_inertia_error=max(pitch_inertia_error,abs(pitch_velocity'*dyn.M*pitch_velocity-p.I_b));
    wheel_inertia_error=max(wheel_inertia_error,abs(mean(diag(dyn.M(11:12,11:12)))-p.I_w));
    [M,G]=tinympc14_hr_params(p,H);
    truth=audit.points(k).cad.no_lateral_slip;
    matrix_error=max(matrix_error,norm(M-truth.mass_matrix,'fro')/norm(truth.mass_matrix,'fro'));
    gravity_table_error=max(gravity_table_error,norm(G-truth.generalized_gravity)/norm(truth.generalized_gravity));
    % Check actual full-CAD CoM over the two contact points at the reduced
    % model equilibrium. This comparison is independent of its Newton rows.
    s=tinympc14_sched_pt(p,H,0,0);
    [xe,~,info]=tinympc14_fused_eq(p,p.sym7.sw_fused,s);
    assert(info.res<1e-10,'Reduced equilibrium failed');
    q(5)=xe(9); q(7:10)=q(7:10)-(xe(5)-xe(9));
    standing=mixmpc_full_dynamics(m,q,zeros(12,1));
    points=mixmpc_contact_model(m,standing.kin,terrain);
    cg_error=max(cg_error,abs(standing.com(1)-mean(points.point(1,:))));
end
assert(energy_error<1e-7 && gravity_error<1e-6 && contact_error<1e-9, ...
    '001 independent energy, gravity or no-slip work failed');
assert(matrix_error<1e-3 && gravity_table_error<1e-3,'CAD physical interpolation error exceeds 0.1%%');
assert(cg_error<1e-3,'Reduced standing equilibrium puts CAD CoM more than 1 mm off the contact line');
assert(max([tree_mass_error,tree_gravity_error,tree_com_error])<1e-9, ...
    'Independent rigidBodyTree mass, gravity or CoM comparison failed');
assert(max([yaw_inertia_error,pitch_inertia_error,wheel_inertia_error])<1e-8, ...
    'Yaw, body pitch or wheel spin inertia uses the wrong reference axis');
assert(yaw_table_error<1e-3,'Yaw inertia interpolation exceeds the 0.1%% physical table budget');
nominal_yaw=tinympc14_table_row(p.YAW_DATA,p.H_REF);
assert(abs(p.I_z-p.I_z_body-sum(nominal_yaw))<1e-10,'Nominal yaw inertia differs from its component table');
% Static ground reaction and free-flight CoM acceleration exercise the full
% contact and inertia equations, without identifying a spring force curve.
terrain=struct('origin',zeros(3,2),'normal',repmat([0;0;1],1,2),'mu',[.6;.6]);
q=mixmpc_vertical_pose(m,.27); base_q=q;
lean=fzero(@static_offset,[-.1,.1]);
q=static_pose(lean);
standing=mixmpc_full_dynamics(m,q,zeros(12,1));
c=mixmpc_contact_model(m,standing.kin,terrain);
q(3)=q(3)-mean(c.gap);
standing=mixmpc_full_dynamics(m,q,zeros(12,1));
c=mixmpc_contact_model(m,standing.kin,terrain);
drive=[zeros(6);eye(6)];
actuation=lsqminnorm([drive,c.J'],standing.h-standing.Qspring);
plant=mixmpc_reference_plant(m,q,zeros(12,1),actuation(1:6),terrain);
assert(strcmp(plant.status,'ok') && strcmp(plant.mode,'DS') && norm(plant.qdd,Inf)<1e-7, ...
    '001 static double support is not admissible');
reaction_error=norm(sum(plant.reaction_world,2)+hw.m_total*m.gravity,Inf);
assert(reaction_error<1e-7 && all(plant.reaction(3,:)>0),'001 ground reactions do not balance total weight');
free=standing.M\(standing.Qspring-standing.h);
com_acc=zeros(3,1);
for id=1:numel(r.links)
    link=r.links(id); offset=standing.kin.pose.R(:,:,id)*link.com;
    W=standing.kin.Jw(:,:,id);
    J=standing.kin.Jv(:,:,id)+cross(W,repmat(offset,1,12),1);
    com_acc=com_acc+link.mass*J*free;
end
flight_error=norm(com_acc/hw.m_total-m.gravity,Inf);
assert(flight_error<1e-9,'Internal forces changed the free-flight CoM acceleration');
result.energy_error_J=energy_error; result.gravity_virtual_work_error_W=gravity_error;
result.contact_velocity_error_m_s=contact_error;
result.max_mass_matrix_interpolation_relative_error=matrix_error;
result.max_gravity_interpolation_relative_error=gravity_table_error;
result.max_static_cad_com_offset_m=cg_error;
result.rigid_body_tree_errors=[tree_mass_error,tree_gravity_error,tree_com_error];
result.yaw_pitch_wheel_inertia_errors_kg_m2=[yaw_inertia_error,pitch_inertia_error,wheel_inertia_error];
result.max_yaw_inertia_interpolation_relative_error=yaw_table_error;
result.static_ground_reaction_N=plant.reaction(3,:);
result.static_ground_force_error_N=reaction_error;
result.free_flight_com_acceleration_error_m_s2=flight_error;
result.parameters=hw;
result.pending_identification={'gas-spring pin-force curve','vertical viscous damping', ...
    'roll tipping gain','motor torque constant and gearbox efficiency','installed encoder zeros and IMU frame'};
save(fullfile(folder,'physics_validation.mat'),'result');
fid=fopen(fullfile(folder,'physics_validation.json'),'w','n','UTF-8');
assert(fid>=0,'Cannot save physics validation'); cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s',jsonencode(result,'PrettyPrint',true));
fprintf('001 PHYSICS PASS: energy %.3e J, gravity work %.3e W, table %.3e, standing CoM %.3e m\n', ...
    energy_error,gravity_error,matrix_error,cg_error);

    function q=static_pose(psi)
        q=base_q;
        q(5)=p.sym7.pitch_ref;
        q(7:10)=q(7:10)-(psi-q(5));
    end

    function value=static_offset(psi)
        state=static_pose(psi);
        d=mixmpc_full_dynamics(m,state,zeros(12,1));
        contact=mixmpc_contact_model(m,d.kin,terrain);
        value=d.com(1)-mean(contact.point(1,:));
    end
end
