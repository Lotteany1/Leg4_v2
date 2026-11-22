function result = calibrate_001_urdf()
% 从工作区 001 URDF 显式重算 CAD 物理数据；不改 Q/R，不烧录。
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
repo = fileparts(root);
addpath(fullfile(root,'mixmpc_pre')); mixmpc_paths();
source = fullfile(repo,'001','001','urdf','001.urdf');
robot = mixmpc_robot_config(source);
[~,hardware] = leg_urdf_data();
model = mixmpc_reference_config(robot,hardware,false);
yaw_robot=mixmpc_phase_average_wheels(robot);
hip = arrayfun(@(s)s.frames(1),robot.side);
wheel = arrayfun(@(s)s.frames(11),robot.side);
group = ones(1,numel(robot.links));
for j = robot.joints
    group(j.child) = group(j.parent);
    for side = 1:2
        if ismember(j.child,robot.side(side).frames([1 3])), group(j.child)=side+1; end
        if j.child==wheel(side), group(j.child)=side+3; end
    end
end
mass = accumarray(group(:),[robot.links.mass]',[5 1]);
hip_mid = mean(robot.zero.p(:,hip),2);
[mb,cb,Ib] = aggregate(robot,robot.zero,find(group==1));
body_offset = cb-hip_mid;
limit = zeros(2,2); spin=zeros(2,1); radii=spin; centers=zeros(3,2);
for side = 1:2
    s = robot.side(side); ids=s.frames;
    upper_segment=robot.zero.p([1 3],ids(2))-robot.zero.p([1 3],ids(1));
    lower=robot.zero.p([1 3],ids(11))-robot.zero.p([1 3],ids(2));
    for k=1:2
        t=s.passive_sign(1)*s.knee_limits(k);
        R=[cos(t) -sin(t);sin(t) cos(t)];
        limit(side,k)=norm(upper_segment+R*lower);
    end
    link=robot.links(wheel(side)); axis=model.wheel_axis(:,side);
    spin(side)=axis'*link.inertia*axis+link.mass*(dot(link.com,link.com)-dot(axis,link.com)^2);
    mesh=stlread(fullfile(robot.mesh_dir,link.mesh));
    vertex=link.mesh_xyz+link.mesh_R*mesh.Points';
    radii(side)=max(vecnorm(vertex-axis*(axis'*vertex)));
    centers(:,side)=robot.zero.p(:,wheel(side))+robot.zero.R(:,:,wheel(side))*model.wheel_center_offset(:,side);
end
maximum=min(max(limit,[],2));
lengths=linspace(.15,maximum,20)';
height_roll=mixmpc_height_roll_audit(robot,lengths',hardware);
data=zeros(numel(lengths),4); balance=data; hr=zeros(numel(lengths),8); turn=zeros(numel(lengths),2);
support=zeros(numel(lengths),2); side_data=zeros(numel(lengths),7,2);
yaw=zeros(numel(lengths),3); yaw_axis=mean(centers,2);
body_yaw=Ib(3,3)+mb*sum((cb([1 2])-yaw_axis([1 2])).^2);
for k=1:numel(lengths)
    L=lengths(k); q=mixmpc_vertical_pose(model,L);
    kin=mixmpc_full_kinematics(model,q,zeros(12,1));
    offset=zeros(3,2); inertia=zeros(1,2); hip_inertia=inertia;
    for side=1:2
        [ml,c,I]=aggregate(robot,kin.pose,find(group==side+1));
        offset(:,side)=c-kin.pose.p(:,hip(side));
        inertia(side)=I(2,2);
        hip_inertia(side)=I(2,2)+ml*sum(offset([1 3],side).^2);
        side_data(k,:,side)=[L,ml,offset(:,side)',inertia(side),hip_inertia(side)];
    end
    c=offset*mass(2:3)/sum(mass(2:3));
    ld=norm(c([1 3])); theta=atan2(-c(1),-c(3));
    data(k,:)=[L,theta,ld,mean(hip_inertia)];
    balance(k,:)=[L,L+c(3),-c(3),mean(inertia)];
    point=height_roll.points(k).cad;
    M=point.no_lateral_slip.mass_matrix; G=point.no_lateral_slip.generalized_gravity;
    hr(k,:)=[L,M(1,1),M(1,2),M(2,2),G',point.thin_wheel_tip_slope_Nm_per_rad,point.roll_potential_curvature_Nm_per_rad];
    [mt,ct,~]=aggregate(robot,kin.pose,1:numel(robot.links));
    [ms,cs,~]=aggregate(robot,kin.pose,find(group<=3));
    ground=mean(kin.pose.p(3,wheel))-mean(radii);
    turn(k,:)=[L,mt*(ct(3)-ground)];
    support(k,:)=[L,cs(3)-mean(kin.pose.p(3,hip))];
    yaw(k,1)=L;
    for side=1:2
        [part_mass,part_center,part_inertia]=aggregate(yaw_robot,kin.pose,find(group==side+1 | group==side+3));
        yaw(k,side+1)=part_inertia(3,3)+part_mass*sum((part_center([1 2])-yaw_axis([1 2])).^2);
    end
end
nominal=mixmpc_height_roll_audit(robot,.27,hardware);
point=nominal.points.cad.no_lateral_slip;
M=point.mass_matrix; G=point.generalized_gravity;
hr=sortrows([hr;.27,M(1,1),M(1,2),M(2,2),G', ...
    nominal.points.cad.thin_wheel_tip_slope_Nm_per_rad, ...
    nominal.points.cad.roll_potential_curvature_Nm_per_rad],1);
qn=mixmpc_vertical_pose(model,.27); kn=mixmpc_full_kinematics(model,qn,zeros(12,1));
yaw_nominal=zeros(1,2);
for side=1:2
    [part_mass,part_center,part_inertia]=aggregate(yaw_robot,kn.pose,find(group==side+1 | group==side+3));
    yaw_nominal(side)=part_inertia(3,3)+part_mass*sum((part_center([1 2])-yaw_axis([1 2])).^2);
end
yaw=sortrows([yaw;.27,yaw_nominal],1);
par=struct('g',hardware.g,'m_total',sum(mass),'m_body',mb,'m_leg',mean(mass(2:3)), ...
    'm_wheel',mean(mass(4:5)),'M_SUP',sum(mass(1:3)),'half_track',abs(diff(centers(2,:)))/2, ...
    'wheel_radius',mean(radii),'body_x',body_offset(1),'body_y',body_offset(2),'l_c',body_offset(3), ...
    'I_w',mean(spin),'I_b',Ib(2,2)+mb*sum(body_offset([1 3]).^2), ...
    'I_z_body',body_yaw,'I_z',body_yaw+sum(yaw_nominal), ...
    'Len_min',.17,'Len_max',maximum,'LPV_min',.16,'M_V',point.mass_matrix(1,1), ...
    'I_ROLL',point.mass_matrix(2,2),'M_LOAD',point.generalized_gravity(1)/hardware.g);
frames=robot.side(1).frames([1 2 3 4 5 6 7 8 11]);
geometry=robot.zero.p([1 3],frames)-robot.zero.p([1 3],frames(1));
tool=fullfile(root,'Tool');
write_text(fullfile(tool,'leg_mechanism_data.m'),sprintf(['function g = leg_mechanism_data()\n%%#codegen\n' ...
    '%% 001 CAD zero XZ points relative to hip; generated by calibrate_001_urdf.\ng = %s;\nend\n'],mat2str(geometry,17)));
body=sprintf('function [data,p,balance,hr,turn,support,yaw] = leg_urdf_data()\n%%#codegen\n%% 001 CAD inertial data; generated by calibrate_001_urdf.\n');
body=[body sprintf('data = %s;\nbalance = %s;\nhr = %s;\nturn = %s;\nsupport = %s;\n', ...
    mat2str(data,17),mat2str(balance,17),mat2str(hr,17),mat2str(turn,17),mat2str(support,17))];
body=[body sprintf('yaw = %s;\n',mat2str(yaw,17))];
fields=fieldnames(par);
for k=1:numel(fields), body=[body sprintf('p.%s = %.17g;\n',fields{k},par.(fields{k}))]; end %#ok<AGROW>
write_text(fullfile(tool,'leg_urdf_data.m'),[body 'end' newline]);
header=sprintf('/** @file  physical_config.h  @brief 001 URDF physical constants, generated by MATLAB. */\n#ifndef PHYSICAL_CONFIG_H\n#define PHYSICAL_CONFIG_H\n');
keys={'m_total','m_body','m_leg','m_wheel','M_SUP','half_track','wheel_radius','l_c','body_x','Len_min','Len_max','I_w','I_b','I_z','I_z_body'};
units={'kg, whole robot','kg, body and fixed IMU','kg, mean non-wheel leg', ...
    'kg, mean wheel','kg, wheel-excluded support','m, half tire-center track', ...
    'm, STL outer radius','m, body CoM signed height above hip','m, body CoM forward offset', ...
    'm, operational lower length','m, exported knee upper length', ...
    'kg m^2, mean axle spin','kg m^2, body about hip pitch axis', ...
    'kg m^2, whole locked yaw about tire midpoint, excluding wheel spin', ...
    'kg m^2, body contribution about the same yaw axis'};
for k=1:numel(keys)
    header=[header sprintf('#define CAD_%s %.12gf // %s\n',upper(keys{k}),par.(keys{k}),units{k})]; %#ok<AGROW>
end
header=[header sprintf('#define CAD_URDF_SHA256 "%s"\n#endif\n',tinympc14_sha256(source))];
write_text(fullfile(repo,'Wheel_Leg_Code','Code','Config','physical_config.h'),header);
result=struct('source',source,'sha256',tinympc14_sha256(source),'parameters',par, ...
    'mass_groups',mass,'body_com',cb,'body_inertia_com',Ib,'leg_data',data,'balance',balance, ...
    'height_roll',height_roll,'side_data',side_data,'turn',turn,'support',support,'geometry',geometry, ...
    'yaw',yaw,'yaw_axis_point_m',yaw_axis, ...
    'provenance','URDF CAD calculation, not a physical inertia measurement');
folder=fullfile(repo,'research','urdf_migration_20261122');
if ~isfolder(folder), mkdir(folder); end
save(fullfile(folder,'cad_measurements.mat'),'result');
write_text(fullfile(folder,'cad_measurements.json'),jsonencode(result,'PrettyPrint',true));
disp(par);
fprintf('001 CAD measurement and parameter export PASS\n');
end

function [mass,center,I]=aggregate(robot,pose,ids)
mass=sum([robot.links(ids).mass]); center=zeros(3,1); positions=zeros(3,numel(ids));
for k=1:numel(ids)
    id=ids(k); positions(:,k)=pose.p(:,id)+pose.R(:,:,id)*robot.links(id).com;
    center=center+robot.links(id).mass*positions(:,k);
end
center=center/mass; I=zeros(3);
for k=1:numel(ids)
    id=ids(k); link=robot.links(id); R=pose.R(:,:,id); d=positions(:,k)-center;
    I=I+R*link.inertia*R'+link.mass*(dot(d,d)*eye(3)-d*d');
end
end

function write_text(file,text)
fid=fopen(file,'wb'); assert(fid>=0,'Cannot write %s.',file);
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
bytes=unicode2native(text,'UTF-8'); assert(fwrite(fid,bytes,'uint8')==numel(bytes),'Incomplete file.');
end
