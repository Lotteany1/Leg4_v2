function audit = mixmpc_mechanism_audit(robot)
% Verify candidate planar loops in the full URDF tree and check mesh evidence.
links = robot.links;
mass = [links.mass];
inertia_eigs = cell2mat(arrayfun(@(link) sort(eig(link.inertia)), links, 'UniformOutput', false));
assert(all(mass>0) && all(inertia_eigs(1,:)>0), 'Invalid mass or inertia.');
triangle_margin = sum(inertia_eigs(1:2,:),1)-inertia_eigs(3,:);
assert(all(triangle_margin >= -1e-12), 'Inertia triangle inequality violated.');
com = [links.com];
for k = 1:numel(links)
    com(:,k) = robot.zero.p(:,k) + robot.zero.R(:,:,k)*links(k).com;
end
audit.total_mass_kg = sum(mass);
audit.com_at_cad_zero_m = com*mass'/sum(mass);
audit.min_principal_inertia_kg_m2 = min(inertia_eigs,[],'all');
audit.min_inertia_triangle_margin_kg_m2 = min(triangle_margin);
I = zeros(3);
for k = 1:numel(links)
    d = com(:,k)-audit.com_at_cad_zero_m;
    R = robot.zero.R(:,:,k);
    I = I + R*links(k).inertia*R' + mass(k)*(dot(d,d)*eye(3)-d*d');
end
audit.inertia_about_cad_zero_com_kg_m2 = I;
audit.revolute_joint_names = {robot.joints([robot.joints.q]>0).name};
audit.missing_effort_or_speed = {robot.joints([robot.joints.q]>0 & ...
    arrayfun(@(j) any(j.limits(3:4)==0),robot.joints)).name};
audit.link_mass_kg = struct('name',{links.name},'mass_kg',num2cell(mass));

for k = 1:2
    s = robot.side(k);
    pins = [s.closure_spec s.spring_pins];
    for p = 1:numel(pins)
        pin = pins(p);
        for side_link = 1:numel(pin.links)
            id = pin.links(side_link);
            mesh = stlread(fullfile(robot.mesh_dir,links(id).mesh));
            vertices = links(id).mesh_xyz + links(id).mesh_R*mesh.Points';
            [count, radius, error,fit_error,coverage] = ring(vertices,pin.point(:,side_link),pin.axis(:,side_link));
            % Some 001 bearing cutouts retain only seven independent arc
            % vertices. Test geometric fit and angular coverage explicitly.
            assert(count>=6 && error<5e-5 && fit_error<2e-6 && coverage>pi, ...
                '%s: missing bore in %s (%d vertices, center %.3g m, fit %.3g m, arc %.3g rad).', ...
                pin.marker,links(id).name,count,error,fit_error,coverage);
            evidence = struct('link',links(id).name,'radius_m',radius, ...
                'vertices',count,'fitted_center_error_m',error,'fit_error_m',fit_error,'arc_coverage_rad',coverage);
            audit.leg(k).pin(p).evidence(side_link) = evidence;
        end
        audit.leg(k).pin(p).marker = pin.marker;
        audit.leg(k).pin(p).point_m = pin.point;
        audit.leg(k).pin(p).axis = pin.axis;
    end
    audit.leg(k).name = s.name;
    audit.leg(k).knee_limit_rad = s.knee_limits;
    audit.leg(k).active_candidates = audit.revolute_joint_names(s.active);
    audit.leg(k).passive_joints = audit.revolute_joint_names(s.passive);
    wheel_id = s.frames(11);
    wheel = stlread(fullfile(robot.mesh_dir,links(wheel_id).mesh));
    axis = robot.joints([robot.joints.child]==wheel_id).axis;
    audit.leg(k).wheel_mesh_outer_radius_m = max(vecnorm(wheel.Points'-axis*(axis'*wheel.Points')));
end

pose_count = 0;
max_error = 0;
max_mirror = 0;
min_sigma = Inf;
step = 1e-5;
% Exercise the CAD branch near zero independently of the knee-limit scan.
for hip = linspace(-0.3,0.3,5)
    for relative = linspace(-0.1,0.1,5)
        active = zeros(2,2);
        for side=1:2, active(:,side)=[hip;hip+relative]./robot.side(side).active_sign(:); end
        [q,valid] = mixmpc_closure(robot,active);
        assert(valid, 'Nominal CAD branch has disconnected unexpectedly.');
        pose_count = pose_count+1;
        max_error = max(max_error,norm(loop_residual(robot,q),Inf));
        pose = mixmpc_robot_fk(robot,q);
        left = pose.p(:,robot.side(1).frames(11))-pose.p(:,robot.side(1).frames(1));
        right = pose.p(:,robot.side(2).frames(11))-pose.p(:,robot.side(2).frames(1));
        max_mirror = max(max_mirror,norm(left([1 3])-right([1 3]),Inf));
        active_id = [robot.side.active];
        passive_id = [robot.side.passive];
        J = zeros(8,numel(passive_id));
        for j = 1:numel(passive_id)
            direction = zeros(size(q));
            direction(passive_id(j)) = step;
            J(:,j) = (loop_residual(robot,q+direction)-loop_residual(robot,q-direction))/(2*step);
        end
        min_sigma = min(min_sigma,min(svd(J)));
        active_rate = [0.6 -0.4; 0.1 0.25];
        qplus = mixmpc_closure(robot, active+step*active_rate);
        qminus = mixmpc_closure(robot, active-step*active_rate);
        qdot = (qplus-qminus)/(2*step);
        assert(norm(qdot(active_id)-active_rate(:),Inf)<1e-8, 'Drive coordinate mismatch.');
        assert(norm((loop_residual(robot,q+step*qdot)-loop_residual(robot,q-step*qdot))/(2*step),Inf)<1e-7, ...
            'Closed-chain velocity is inconsistent with the URDF tree.');
    end
end
assert(max_error<1e-6 && max_mirror<1e-6 && min_sigma>1e-4, 'CAD loop checks failed.');
audit.pose_count = pose_count;
audit.max_position_residual_m = max_error;
audit.max_sagittal_mirror_error_m = max_mirror;
audit.min_passive_jacobian_sigma_m_per_rad = min_sigma;

% Sweep the connected, same-assembly branch around the CAD pose using the full
% export's knee bounds. Hip range, interference and motor stops are separate.
relative = linspace(-pi,pi,2001);
for k = 1:2
    s = robot.side(k);
    knee = nan(size(relative));
    length_m = knee;
    angle_rad = knee;
    for index = 1:numel(relative)
        drive = zeros(2,2);
        drive(2,k) = relative(index);
        [q,feasible] = mixmpc_closure(robot, drive);
        if ~feasible, continue, end
        pose = mixmpc_robot_fk(robot,q);
        delta = pose.p([1 3],s.frames(11))-pose.p([1 3],s.frames(1));
        knee(index) = q(s.passive(1));
        length_m(index) = norm(delta);
        angle_rad(index) = atan2(delta(1),-delta(2));
    end
    allowed = isfinite(knee) & knee>=s.knee_limits(1) & knee<=s.knee_limits(2);
    center = (numel(relative)+1)/2;
    first = find(~allowed(1:center),1,'last')+1;
    last = center+find(~allowed(center+1:end),1,'first')-1;
    assert(allowed(center) && ~isempty(first) && ~isempty(last), 'No bounded connected knee range around CAD zero.');
    span = first:last;
    audit.leg(k).knee_only_length_range_m = [min(length_m(span)) max(length_m(span))];
    audit.leg(k).knee_only_drive_delta_rad = relative([first last]);
    audit.leg(k).scan = struct('drive_delta_rad',relative,'length_m',length_m, ...
        'knee_rad',knee,'leg_angle_rad',angle_rad,'within_exported_knee_limits',allowed);
end
end

function residual = loop_residual(robot,q)
pose = mixmpc_robot_fk(robot,q);
residual = zeros(8,1);
for k = 1:2
    pins = robot.side(k).closure_spec;
    for p = 1:2
        ids = pins(p).links;
        delta = pose.p(:,ids(1)) + pose.R(:,:,ids(1))*pins(p).point(:,1) ...
              - pose.p(:,ids(2)) - pose.R(:,:,ids(2))*pins(p).point(:,2);
        idx = 4*(k-1)+2*(p-1);
        residual(idx+(1:2)) = delta([1 3]);
    end
end
end

function [count, radius, center_error,fit_error,coverage] = ring(vertices, marker, axis)
% Project onto the actual bore plane; 001 does not align every frame to Y.
basis=null(axis');
% A revolute-axis origin may be anywhere along the same bore line; the
% exported marker can sit outside the bearing face's axial span.
points = unique((basis'*vertices)','rows');
offset = points-(basis'*marker)';
distance = round(hypot(offset(:,1),offset(:,2))*1e5)*1e-5;
mask = distance>=0.002 & distance<=0.015;
if nnz(mask)<3
    count = 0;
    radius = NaN;
    center_error = Inf;
    fit_error = Inf; coverage=0;
    return
end
radius = mode(distance(mask));
samples = unique(round(offset(distance==radius,:)*1e6)*1e-6,'rows');
count = size(samples,1);
circle = [2*samples, ones(count,1)] \ sum(samples.^2,2);
center_error = norm(circle(1:2));
fitted_radius=sqrt(circle(3)+sum(circle(1:2).^2));
fit_error=max(abs(vecnorm(samples-circle(1:2)',2,2)-fitted_radius));
angles=sort(atan2(samples(:,2)-circle(2),samples(:,1)-circle(1)));
coverage=2*pi-max(diff([angles;angles(1)+2*pi]));
end
