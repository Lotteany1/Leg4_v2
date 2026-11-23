function robot = mixmpc_robot_config(urdf_file)
% Load the full CAD export. Joint coordinates use the URDF axes and zero pose.
doc = xmlread(urdf_file);
root = doc.getDocumentElement();
nodes = root.getElementsByTagName('link');
for k = nodes.getLength():-1:1
    item = nodes.item(k-1);
    inertial = element(item, 'inertial');
    [com, frame] = origin(inertial);
    i = element(inertial, 'inertia');
    d = attrs(i, {'ixx','ixy','ixz','iyy','iyz','izz'});
    I = [d(1) d(2) d(3); d(2) d(4) d(5); d(3) d(5) d(6)];
    visual = element(item, 'visual');
    [mesh_xyz, mesh_R] = origin(visual);
    mesh = char(element(element(visual, 'geometry'), 'mesh').getAttribute('filename'));
    [~, mesh_name, extension] = fileparts(mesh);
    links(k) = struct('name', char(item.getAttribute('name')), ... %#ok<AGROW>
        'mass', attrs(element(inertial, 'mass'), {'value'}), 'com', com, ...
        'inertia', frame*I*frame', 'mesh', [mesh_name extension], ...
        'mesh_xyz', mesh_xyz, 'mesh_R', mesh_R);
end
link_names = {links.name};
nodes = root.getElementsByTagName('joint');
moving = 0;
for k = 1:nodes.getLength()
    item = nodes.item(k-1);
    type = char(item.getAttribute('type'));
    [translation, rotation] = origin(item);
    child = find(strcmp(link_names, char(element(item,'child').getAttribute('link'))));
    parent = find(strcmp(link_names, char(element(item,'parent').getAttribute('link'))));
    axis = zeros(3,1);
    limits = nan(1,4);
    coordinate = 0;
    if strcmp(type,'revolute')
        moving = moving+1;
        coordinate = moving;
        axis = vector(element(item,'axis'), 'xyz');
        assert(abs(norm(axis)-1) < 1e-12, 'Non-unit joint axis.');
        limits = attrs(element(item,'limit'), {'lower','upper','effort','velocity'});
    else
        assert(strcmp(type,'fixed'), 'Unexpected joint type %s.',type);
    end
    joints(k) = struct('name', char(item.getAttribute('name')), 'type',type, ... %#ok<AGROW>
        'parent',parent, 'child',child, 'translation',translation, 'rotation',rotation, ...
        'axis',axis, 'q',coordinate, 'limits',limits);
end
joint_names = {joints.name};
robot = struct('links',links, 'joints',joints, 'nq',moving, 'urdf',urdf_file, ...
    'base',find(strcmp(link_names,'base_link')));
assert(moving==14, 'Expected the full 14-joint CAD export.');
robot.mesh_dir = fullfile(fileparts(fileparts(urdf_file)), 'meshes');
robot.zero = mixmpc_robot_fk(robot, zeros(moving,1));
for k = 1:2
    prefix = {'lf','rf'};
    names = strcat(prefix{k},{'0','1','00','01','02','03','04','05','001','002','wheel'});
    [found, ids] = ismember(names,joint_names);
    assert(all(found), 'Missing joint in %s leg.',prefix{k});
    joint = joints(ids);
    frames = [joint.child];
    assert(max(abs(cat(3,joint.rotation)-repmat(eye(3),1,1,numel(joint))), [], 'all') < 1e-12, ...
        'The planar closure requires aligned export frames.');
    axes = [joint.axis];
    active = [1 3];
    passive = [2 4 5 6];
    rotating = [active passive 11];
    sign_y = axes(2,rotating);
    assert(all(abs(sign_y)==1) && all(sign_y==sign_y(1)), 'Leg axes are not parallel.');
    pins = [make_pin(frames(7),frames([1 5])), make_pin(frames(8),frames([2 6]))];
    spring = [make_pin(frames(9),frames(1)), make_pin(frames(10),frames(2))];
    robot.side(k) = struct('name',prefix{k}, 'active',[joint(active).q], ...
        'passive',[joint(passive).q], 'wheel',joint(11).q, 'q_sign',-sign_y(1), ...
        'frames',frames, 'closure_spec',pins, 'spring_pins',spring, ...
        'knee_limits',joint(2).limits(1:2));
end

    function pin = make_pin(marker, body_links)
        position = robot.zero.p(:,marker);
        axis = [0;1;0];
        local = zeros(3,numel(body_links));
        local_axis = local;
        for i_link = 1:numel(body_links)
            id = body_links(i_link);
            R = robot.zero.R(:,:,id);
            local(:,i_link) = R'*(position-robot.zero.p(:,id));
            local_axis(:,i_link) = R'*axis;
        end
        pin = struct('marker',links(marker).name, 'links',body_links, ...
            'point',local, 'axis',local_axis);
    end
end

function node = element(parent, label)
node = parent.getElementsByTagName(label).item(0);
assert(~isempty(node), 'Missing URDF element %s.',label);
end

function values = attrs(node, names)
values = cellfun(@(n) str2double(char(node.getAttribute(n))), names);
assert(all(isfinite(values)), 'Missing or invalid numeric URDF attribute.');
end

function v = vector(node, name)
v = sscanf(char(node.getAttribute(name)), '%f');
assert(numel(v)==3 && all(isfinite(v)), 'Expected a finite URDF 3-vector.');
end

function [p,R] = origin(node)
o = element(node,'origin');
p = vector(o,'xyz');
a = vector(o,'rpy');
c = cos(a);
s = sin(a);
R = [c(3) -s(3) 0; s(3) c(3) 0; 0 0 1] * ...
    [c(2) 0 s(2); 0 1 0; -s(2) 0 c(2)] * ...
    [1 0 0; 0 c(1) -s(1); 0 s(1) c(1)];
end
