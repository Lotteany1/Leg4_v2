function dyn = mixmpc_full_dynamics(model, q, v)
% M*qdd + h = [zeros(6,1); tau] + Qspring + Jcontact'*lambda.
% tau is ordered lf0, lf00, rf0, rf00, lfwheel, rfwheel in CAD joint signs.
kin = mixmpc_full_kinematics(model,q,v);
v = kin.v;
links = model.robot.links;
mass = [links.mass];
M = zeros(size(kin.N,1));
h = zeros(size(M,1),1);
gravity = h;
com = zeros(3,numel(links));
velocity = com;
spin = com;
for j = 1:numel(links)
    R = kin.pose.R(:,:,j);
    offset = R*links(j).com;
    I = R*links(j).inertia*R';
    W = kin.open_Jw(:,:,j);
    J = kin.open_Jv(:,:,j) + cross(W,repmat(offset,1,size(W,2)),1);
    omega = kin.omega(:,j);
    a_bias = kin.open_bv(:,j)+cross(kin.open_bw(:,j),offset)+cross(omega,cross(omega,offset));
    M = M + mass(j)*(J'*J) + W'*I*W;
    gravity = gravity - mass(j)*J'*model.gravity;
    h = h + mass(j)*J'*a_bias + W'*(I*kin.open_bw(:,j)+cross(omega,I*omega));
    com(:,j) = kin.pose.p(:,j)+offset;
    velocity(:,j) = J*kin.open_v;
    spin(:,j) = I*omega;
end
h = h+gravity;
M = (M+M')/2;
[spring, potential, spring_length, spring_load] = spring_loads(model,kin);
center = com*mass'/sum(mass);
momentum = velocity.*mass;
reduced_M = kin.N'*M*kin.N;
limits = model.joint_limits(model.check_limit,1:2);
angles = kin.joint_position(model.check_limit);
dyn = struct('kin',kin,'M',(reduced_M+reduced_M')/2, ...
    'h',kin.N'*(h+M*kin.open_bias),'gravity',kin.N'*gravity,'Qspring',kin.N'*spring, ...
    'com',center,'linear_momentum',sum(momentum,2), ...
    'angular_momentum_com',sum(spin+cross(com-center,momentum,1),2), ...
    'energy',[v'*reduced_M*v/2; -sum(mass.*(model.gravity'*com)); potential], ...
    'spring_length',spring_length,'spring_force',spring_load, ...
    'joint_margin',min(angles-limits(:,1),limits(:,2)-angles), ...
    'open_M',M,'open_h',h,'open_gravity',gravity,'open_Qspring',spring);
end

function [Q,U,len,force] = spring_loads(model,kin)
Q = zeros(size(kin.N,1),1);
U = 0;
len = zeros(2,1);
force = len;
s = model.spring;
for side = 1:2
    pins = model.robot.side(side).spring_pins;
    p = zeros(3,2);
    J = zeros(3,numel(Q),2);
    for end_id = 1:2
        id = pins(end_id).links;
        offset = kin.pose.R(:,:,id)*pins(end_id).point;
        p(:,end_id) = kin.pose.p(:,id)+offset;
        J(:,:,end_id) = kin.open_Jv(:,:,id)+cross(kin.open_Jw(:,:,id),repmat(offset,1,numel(Q)),1);
    end
    delta = p(:,2)-p(:,1);
    len(side) = norm(delta);
    assert(len(side)>1e-9,'Coincident gas-spring pins.');
    gap = max(s.long-len(side),0);
    stroke = s.long-s.short;
    compression = min(gap,stroke);
    force(side) = s.peak*compression/stroke;
    U = U + s.peak*(compression^2/(2*stroke)+max(gap-stroke,0));
    Q = Q + (J(:,:,2)-J(:,:,1))'*(delta*(force(side)/len(side)));
end
end
