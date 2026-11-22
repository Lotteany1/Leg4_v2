function cfg = tinympc14_urdf_config(urdf_file, wheel_radius)
% Build the reduced MPC parameters from the six-joint training URDF.
% The wheel collision geometry is a mesh. Pass the measured radius explicitly.
validateattributes(wheel_radius, {'numeric'}, {'scalar','real','finite','positive'});
robot = importrobot(urdf_file, 'DataFormat', 'struct');
home = homeConfiguration(robot);
names = string({home.JointName});
assert(numel(home) == 6 && all(ismember(["lf0","lf1","lfwheel","rf0","rf1","rfwheel"], names)), ...
    'Use the six-joint training URDF (chuanliantui_train.urdf).');
for k = 1:numel(home)
    home(k).JointPosition = 0;
end

cfg = tinympc14_config();
cfg.R_w = wheel_radius;
% Give heading error more weight against yaw-rate damping in the CAD model.
cfg.Q_DIAG(3) = 70000;
prefix = ["lf", "rf"];
legs = cell(1,2);
wheels = cell(1,2);
hip = zeros(3,2);
axle = zeros(3,2);
links = cell(1,2);
range = zeros(2,2);
for side = 1:2
    tag = prefix(side);
    legs{side} = cellstr(tag + ["0","1","00","01","02","03"]);
    wheels{side} = {char(tag + "wheel")};
    links{side} = leg_geometry(robot, home, tag);
    geom = links{side};
    hip(:,side) = geom.hip;
    axle(:,side) = geom.axle;
    bends = geom.bend - geom.knee_sign * geom.knee_limits;
    assert(all(bends > 0 & bends < pi), 'Expected the rear-knee assembly branch.');
    lengths = sqrt(sum(geom.lengths.^2) + 2*prod(geom.lengths)*cos(bends));
    range(side,:) = [min(lengths),max(lengths)];
end
cfg.LEG_RANGE = [max(range(:,1)),min(range(:,2))];
assert(cfg.LEG_RANGE(1) < cfg.H_REF && cfg.H_REF < cfg.LEG_RANGE(2), ...
    'Nominal height falls outside the common leg workspace.');

doc = xmlread(urdf_file);
base = base_inertial(doc,robot.BaseName);
body_names = setdiff(robot.BodyNames, [legs{:}, wheels{:}]);
[cfg.m_b, body_com, body_inertia] = aggregate(robot, home, body_names, base);
hip_mid = mean(hip,2);
cfg.l_c = body_com(3) - hip_mid(3);
cfg.I_b = body_inertia(2,2) + cfg.m_b*sum((body_com([1,3])-hip_mid([1,3])).^2);
cfg.I_z = body_inertia(3,3);
cfg.R_l = abs(diff(axle(2,:)))/2;
cfg.R_TRK = cfg.R_l;

len = linspace(cfg.LEG_RANGE(1), cfg.LEG_RANGE(2), 41)';
data = zeros(numel(len),3,2);
comp = zeros(numel(len),2,2);
leg_mass = zeros(1,2);
wheel_mass = zeros(1,2);
wheel_inertia = zeros(1,2);
nominal = home;
for side = 1:2
    geom = links{side};
    [wheel_mass(side), ~, inertia] = aggregate(robot, home, wheels{side});
    wheel_inertia(side) = inertia(2,2);
    nominal = align_leg(robot, nominal, geom, cfg.H_REF);
    for k = 1:numel(len)
        q = align_leg(robot, home, geom, len(k));
        [leg_mass(side), center, inertia] = aggregate(robot, q, legs{side});
        center = center - geom.hip;
        hip_to_com = -center(3);
        data(k,:,side) = [len(k)-hip_to_com, hip_to_com, inertia(2,2)];
        comp(k,:,side) = [atan2(-center(1), hip_to_com), hypot(center(1), hip_to_com)];
    end
end

% The deployed reduced equations use symmetric left/right inertial parameters.
cfg.Leg_data = [len, mean(data,3)];
cfg.Leg_comp = [len, mean(comp,3)];
cfg.m_l = mean(leg_mass);
cfg.M_LEG = cfg.m_l;
cfg.m_w = mean(wheel_mass);
cfg.I_w = mean(wheel_inertia);
cfg.M_SUP = cfg.m_b + sum(leg_mass);
cfg.M_V = cfg.M_SUP;  % wheels remain at ground height in vertical motion
cfg.M_LOAD = cfg.M_SUP; % preserve the six-joint training model's lumped support load
[~, ~, inertia] = aggregate(robot, nominal, robot.BodyNames, base);
cfg.I_ROLL = inertia(1,1); % nominal locked assembly about its COM
cfg.H_OFF = cfg.l_c;
cfg.COMP.LEG_GRAV_FF = cfg.m_l;
cfg.COMP.ROLL_TRACK = 2*cfg.R_TRK;
cfg.COMP.ROLL_LEN_CLAMP = cfg.LEG_RANGE;
cfg.URDF_SOURCE = char(urdf_file);

joints = doc.getElementsByTagName('joint');
for k = 0:joints.getLength()-1
    joint = joints.item(k);
    side = find(prefix + "wheel" == string(char(joint.getAttribute('name'))), 1);
    if isempty(side), continue; end
    limit = joint.getElementsByTagName('limit').item(0);
    effort = str2double(char(limit.getAttribute('effort')));
    validateattributes(effort, {'numeric'}, {'finite','positive','scalar'});
    cfg.u_min(side) = -effort;
    cfg.u_max(side) = effort;
end
end

function base = base_inertial(doc, name)
links = doc.getElementsByTagName('link');
for k = 0:links.getLength()-1
    link = links.item(k);
    if strcmp(char(link.getAttribute('name')),name)
        inertial = link.getElementsByTagName('inertial').item(0);
        origin = inertial.getElementsByTagName('origin').item(0);
        base.CenterOfMass = sscanf(char(origin.getAttribute('xyz')),'%f');
        rpy = sscanf(char(origin.getAttribute('rpy')),'%f');
        assert(numel(base.CenterOfMass)==3 && isequal(rpy,zeros(3,1)), ...
            'Expected a base inertial frame with zero RPY.');
        mass = inertial.getElementsByTagName('mass').item(0);
        base.Mass = str2double(char(mass.getAttribute('value')));
        I = inertial.getElementsByTagName('inertia').item(0);
        fields = {'ixx','iyy','izz','iyz','ixz','ixy'};
        base.Inertia = cellfun(@(field) str2double(char(I.getAttribute(field))), fields);
        return;
    end
end
error('No inertial data found for the URDF root %s.', name);
end

function geom = leg_geometry(robot, q, tag)
names = string({q.JointName});
geom.hip_index = find(names == tag+"0");
geom.knee_index = find(names == tag+"1");
hip = getTransform(robot, q, char(tag+"0"));
knee = getTransform(robot, q, char(tag+"1"));
wheel = getTransform(robot, q, char(tag+"wheel"));
geom.hip = hip(1:3,4);
geom.axle = wheel(1:3,4);
geom.wheel = char(tag+"wheel");
segments = [knee([1,3],4)-hip([1,3],4), wheel([1,3],4)-knee([1,3],4)];
geom.lengths = vecnorm(segments);
geom.bend = diff(atan2(segments(1,:),-segments(2,:)));
hip_joint = getBody(robot,char(tag+"0")).Joint;
knee_joint = getBody(robot,char(tag+"1")).Joint;
hip_axis = hip(1:3,1:3)*hip_joint.JointAxis(:);
knee_axis = knee(1:3,1:3)*knee_joint.JointAxis(:);
assert(abs(abs(hip_axis(2))-1) < 1e-8 && abs(abs(knee_axis(2))-1) < 1e-8, ...
    'Expected sagittal-plane revolute joints.');
geom.hip_sign = sign(hip_axis(2));
geom.knee_sign = sign(knee_axis(2));
geom.knee_limits = knee_joint.PositionLimits;
end

function q = align_leg(robot, q, geom, len)
bend_cos = (len^2 - sum(geom.lengths.^2))/(2*prod(geom.lengths));
q(geom.knee_index).JointPosition = (geom.bend - acos(bend_cos))/geom.knee_sign;
q(geom.hip_index).JointPosition = 0;
axle = getTransform(robot, q, geom.wheel);
direction = axle(1:3,4) - geom.hip;
q(geom.hip_index).JointPosition = atan2(direction(1),-direction(3))/geom.hip_sign;
end

function [mass, center, inertia] = aggregate(robot, q, names, base)
mass = 0;
moment = zeros(3,1);
inertia = zeros(3);
first = 1;
if nargin == 4
    first = 0; % The rigid body tree stores the root separately from BodyNames.
end
for k = first:numel(names)
    if k == 0
        body = base;
        pose = eye(4);
    else
        body = getBody(robot, names{k});
        pose = getTransform(robot, q, names{k});
    end
    R = pose(1:3,1:3);
    pos = R*body.CenterOfMass(:) + pose(1:3,4);
    v = body.Inertia;
    I = [v(1), v(6), v(5); v(6), v(2), v(4); v(5), v(4), v(3)];
    mass = mass + body.Mass;
    moment = moment + body.Mass*pos;
    inertia = inertia + R*I*R' + body.Mass*((pos'*pos)*eye(3)-pos*pos');
end
center = moment/mass;
inertia = inertia - mass*((center'*center)*eye(3)-center*center');
end
