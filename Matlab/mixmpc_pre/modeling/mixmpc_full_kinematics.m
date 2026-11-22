function kin = mixmpc_full_kinematics(model, q, v)
% Closed-chain coordinates [p_WB, rpy, lf0, lf00, rf0, rf00, lfwheel, rfwheel].
% v = dq/dt. E below maps ZYX Euler rates to world angular velocity.
% Jv/Jw map to world origin/angular velocities. bv/bw give Jdot(q,v)*v.
robot = model.robot;
q = q(:);
v = v(:);
validateattributes(q,{'double'},{'numel',12,'real','finite'});
validateattributes(v,{'double'},{'numel',12,'real','finite'});
a = q(4:6);
c = cos(a);
s = sin(a);
assert(abs(c(2))>0.02,'Floating-base pitch leaves the ZYX chart.');
Rz = [c(3) -s(3) 0; s(3) c(3) 0; 0 0 1];
Ry = [c(2) 0 s(2); 0 1 0; -s(2) 0 c(2)];
Rx = [1 0 0; 0 c(1) -s(1); 0 s(1) c(1)];
R = Rz*Ry*Rx;
E = [Rz*Ry(:,1) Rz(:,2) [0;0;1]];
[joint_position,feasible] = mixmpc_closure(robot,reshape(q(7:10),2,2),q(11:12));
assert(feasible,'Actuated coordinates leave the CAD closure branch.');
pose = mixmpc_robot_fk(robot,joint_position,q(1:3),R);
n = numel(robot.links);
columns = 6+robot.nq;
linear = zeros(3,columns,n);
angular = zeros(3,columns,n);
axes = zeros(3,n);
base = robot.base;
linear(:,1:3,base) = eye(3);
angular(:,4:6,base) = E;
for joint = robot.joints
    i = joint.parent;
    j = joint.child;
    offset = pose.p(:,j)-pose.p(:,i);
    linear(:,:,j) = linear(:,:,i)-skew(offset)*angular(:,:,i);
    angular(:,:,j) = angular(:,:,i);
    if joint.q
        axes(:,j) = pose.R(:,:,i)*joint.rotation*joint.axis;
        angular(:,6+joint.q,j) = axes(:,j);
    end
end

% Use only the two independent translational closure equations per pin.
% The revolute axes and their lateral offsets are fixed by the planar tree.
plane = R(:,[1 3])';
pins = model.closure_pins;
offsets = zeros(3,2,numel(pins));
position_error = zeros(3,numel(pins));
relative_velocity = zeros(3,columns,numel(pins));
loop_J = zeros(2*numel(pins),columns);
for i = 1:numel(pins)
    ids = pins(i).links;
    for side = 1:2
        offsets(:,side,i) = pose.R(:,:,ids(side))*pins(i).point(:,side);
    end
    position_error(:,i) = pose.p(:,ids(1))+offsets(:,1,i)-pose.p(:,ids(2))-offsets(:,2,i);
    relative_velocity(:,:,i) = linear(:,:,ids(1))-skew(offsets(:,1,i))*angular(:,:,ids(1)) ...
                             -linear(:,:,ids(2))+skew(offsets(:,2,i))*angular(:,:,ids(2));
    loop_J(2*i-1:2*i,:) = plane*relative_velocity(:,:,i);
end
passive_J = loop_J(:,model.passive);
assert(rcond(passive_J)>1e-9,'CAD closure Jacobian is singular on this branch.');
N = zeros(columns,12);
N(model.keep,:) = eye(12);
N(model.passive,:) = -(passive_J\loop_J(:,model.keep));
open_v = N*v;

% Convective bias of the tree at fixed generalized velocities.
omega = reshape(pagemtimes(angular,open_v),3,n);
bv = zeros(3,n);
bw = zeros(3,n);
yaw_omega = E(:,3)*v(6);
pitch_omega = E(:,2)*v(5);
bw(:,base) = cross(yaw_omega,pitch_omega)+cross(yaw_omega+pitch_omega,E(:,1)*v(4));
for joint = robot.joints
    i = joint.parent;
    j = joint.child;
    offset = pose.p(:,j)-pose.p(:,i);
    bv(:,j) = bv(:,i)+cross(bw(:,i),offset)+cross(omega(:,i),cross(omega(:,i),offset));
    bw(:,j) = bw(:,i);
    if joint.q
        bw(:,j) = bw(:,j)+open_v(6+joint.q)*cross(omega(:,i),axes(:,j));
    end
end

% Second derivative of each closure equation gives Ndot*v. At closure the
% extra derivatives of the moving projection plane multiply zero residuals.
loop_bias = zeros(size(loop_J,1),1);
pin_bias = zeros(3,numel(pins));
for i = 1:numel(pins)
    ids = pins(i).links;
    pin_bias(:,i) = point_bias(ids(1),offsets(:,1,i))-point_bias(ids(2),offsets(:,2,i));
    loop_bias(2*i-1:2*i) = plane*pin_bias(:,i);
end
open_bias = zeros(columns,1);
open_bias(model.passive) = -(passive_J\loop_bias);
kin = struct('pose',pose,'q',q,'v',v,'joint_position',joint_position,'N',N, ...
    'open_v',open_v,'open_bias',open_bias,'open_Jv',linear,'open_Jw',angular, ...
    'open_bv',bv,'open_bw',bw,'omega',omega, ...
    'Jv',pagemtimes(linear,N),'Jw',pagemtimes(angular,N), ...
    'bv',bv+reshape(pagemtimes(linear,open_bias),3,n), ...
    'bw',bw+reshape(pagemtimes(angular,open_bias),3,n), ...
    'loop_J',loop_J,'loop_bias',loop_bias,'loop_position',position_error, ...
    'loop_velocity',reshape(pagemtimes(relative_velocity,open_v),3,[]), ...
    'loop_acceleration',pin_bias+reshape(pagemtimes(relative_velocity,open_bias),3,[]));

    function a = point_bias(id,r)
        a = bv(:,id)+cross(bw(:,id),r)+cross(omega(:,id),cross(omega(:,id),r));
    end
end

function C = skew(a)
C = [0 -a(3) a(2); a(3) 0 -a(1); -a(2) a(1) 0];
end
