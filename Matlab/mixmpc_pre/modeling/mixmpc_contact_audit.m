function audit = mixmpc_contact_audit(robot, radius)
% Kinematic point-contact audit with ideal circular wheels and a floating base.
% Coordinates are [xyz rpy lf0 lf00 rf0 rf00 lfwheel rfwheel] in the URDF frame.
nominal = zeros(12,1);
asymmetric = [0;0;0; deg2rad(-5);0.1;0.25; 0.08;0.02;0.10;0.05; 0.4;-0.6];
states = [nominal asymmetric];
wheel_link = arrayfun(@(side) side.frames(11), robot.side);
axis_local = [[0;-1;0] [0;1;0]];
mass = [robot.links.mass];
normal = [0;0;1];
h = 1e-6;
for k = 1:size(states,2)
    state = states(:,k);
    pose = fk(robot,state);
    point = lowest_points(pose);
    offset = point-pose.p(:,wheel_link);
    J = zeros(6,12);
    for j = 1:numel(state)
        direction = zeros(size(state));
        direction(j) = h;
        pp = fk(robot,state+direction);
        pm = fk(robot,state-direction);
        for i = 1:2
            id = wheel_link(i);
            linear = (pp.p(:,id)-pm.p(:,id))/(2*h);
            skew = (pp.R(:,:,id)-pm.R(:,:,id))/(2*h)*pose.R(:,:,id)';
            skew = (skew-skew')/2;
            angular = [skew(3,2);skew(1,3);skew(2,1)];
            J(3*(i-1)+(1:3),j) = linear + cross(angular,offset(:,i));
        end
    end
    % Normal velocity of the material contact point must match gap derivative.
    direction = [0.1;-0.2;0.4; 0.08;-0.05;0.15; 0.4;-0.6;0.2;0.3; 1;-2];
    pp = fk(robot,state+h*direction);
    pm = fk(robot,state-h*direction);
    gap_velocity = (lowest_points(pp)-lowest_points(pm))/(2*h);
    normal_fd = gap_velocity(3,:)';
    error = norm(normal_fd - J([3 6],:)*direction,Inf);
    assert(error<1e-7, 'Contact velocity is inconsistent with gap geometry.');
    audit.pose(k) = struct('q',state,'wheel_plane_z_m',point(3,:), ... %#ok<AGROW>
        'contact_rank_DS',rank(J,1e-8),'contact_rank_LS',rank(J(1:3,:),1e-8), ...
        'contact_rank_RS',rank(J(4:6,:),1e-8),'normal_rank_DS',rank(J([3 6],:),1e-8), ...
        'normal_velocity_error_m_s',error);

    if k==1
        com = pose.p;
        for body = 1:numel(mass)
            com(:,body) = com(:,body)+pose.R(:,:,body)*robot.links(body).com;
        end
        com = com*mass'/sum(mass);
        audit.cad_zero_single_support_static_offset_m = vecnorm(point(1:2,:)-com(1:2));
        audit.cad_zero_single_support_gravity_moment_Nm = ...
            sum(mass)*9.81*audit.cad_zero_single_support_static_offset_m;
    end
end

    function points = lowest_points(pose)
        points = pose.p(:,wheel_link);
        for i = 1:2
            axle = pose.R(:,:,wheel_link(i))*axis_local(:,i);
            radial = normal - axle*dot(axle,normal);
            points(:,i) = points(:,i)-radius*radial/norm(radial);
        end
    end
end

function pose = fk(robot,state)
[q, valid] = mixmpc_closure(robot,reshape(state(7:10),2,2),state(11:12));
assert(valid, 'Audit pose leaves the selected closure branch.');
c = cos(state(4:6));
s = sin(state(4:6));
R = [c(3) -s(3) 0; s(3) c(3) 0; 0 0 1] * ...
    [c(2) 0 s(2); 0 1 0; -s(2) 0 c(2)] * ...
    [1 0 0; 0 c(1) -s(1); 0 s(1) c(1)];
pose = mixmpc_robot_fk(robot,q,state(1:3),R);
end
