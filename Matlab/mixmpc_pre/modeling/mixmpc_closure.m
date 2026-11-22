function [q, feasible] = mixmpc_closure(robot, active, wheel)
% Solve the two planar loops of each leg on the branch of the zero CAD pose.
% active(:,side) contains the two coaxial candidate drive angles in URDF radians.
% Side order follows the export names lf then rf. Wheel spin is independent.
if nargin < 3, wheel = [0 0]; end
q = nan(robot.nq,1);
feasible = false;
for side = 1:2
    s = robot.side(side);
    zero = robot.zero.p([1 3],s.frames);
    hip = zero(:,1);
    a = s.q_sign*active(:,side);
    knee = hip + rotate(a(1),zero(:,2)-hip);
    crank_tip = zero(:,3) + rotate(a(2),zero(:,4)-zero(:,3));
    rocker_pin = hip + rotate(a(1),zero(:,7)-hip);

    hinge = intersection(crank_tip,rocker_pin,zero(:,4),zero(:,7),zero(:,5));
    if any(~isfinite(hinge)), return, end
    coupler = angle_from(zero(:,5)-zero(:,4),hinge-crank_tip);
    rocker = angle_from(zero(:,5)-zero(:,7),hinge-rocker_pin);
    rod_pin = hinge + rotate(rocker,zero(:,6)-zero(:,5));

    shank_pin = intersection(knee,rod_pin,zero(:,2),zero(:,6),zero(:,8));
    if any(~isfinite(shank_pin)), return, end
    shank = angle_from(zero(:,8)-zero(:,2),shank_pin-knee);
    rod = angle_from(zero(:,8)-zero(:,6),shank_pin-rod_pin);
    passive = [shank-a(1), coupler-a(2), rocker-coupler, rod-rocker];
    q(s.active) = active(:,side);
    q(s.passive) = s.q_sign*atan2(sin(passive),cos(passive));
    q(s.wheel) = wheel(side);
end
feasible = true;
end

function result = rotate(a,v)
result = [cos(a) -sin(a); sin(a) cos(a)] * v;
end

function angle = angle_from(v,w)
angle = atan2(det([v w]),dot(v,w));
end

function p = intersection(a,b,a0,b0,p0)
radius_a = norm(p0-a0);
radius_b = norm(p0-b0);
branch = sign(det([b0-a0 p0-a0]));
distance = norm(b-a);
if distance <= abs(radius_a-radius_b) || distance >= radius_a+radius_b
    p = [NaN;NaN];
    return
end
unit = (b-a)/distance;
along = (radius_a^2-radius_b^2+distance^2)/(2*distance);
perp = branch*sqrt(max(0,radius_a^2-along^2));
p = a + along*unit + perp*[-unit(2);unit(1)];
end
