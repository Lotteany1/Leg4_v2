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
    if strcmp(type,'revolute') || strcmp(type,'continuous')
        moving = moving+1;
        coordinate = moving;
        axis = vector(element(item,'axis'), 'xyz');
        assert(abs(norm(axis)-1) < 1e-12, 'Non-unit joint axis.');
        if strcmp(type,'continuous')
            limits = [-Inf Inf 0 0];
        else
            limits = attrs(element(item,'limit'), {'lower','upper','effort','velocity'});
        end
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
robot.mesh_dir = fullfile(fileparts(fileparts(urdf_file)), 'meshes');
robot.zero = mixmpc_robot_fk(robot, zeros(moving,1));
is001 = any(strcmp(link_names,'right-front'));
if is001
    % The export gives eight pin-marker bodies independent root revolutes.
    % They are material on the moving links, rather than extra mechanism DOFs.
    attachments = {'right-fix-01','right-front'; 'right-fix-02','right-front-01'; ...
        'right-spring-01','right-front-01'; 'right-spring-02','right-front'; ...
        'lefr-fix-01','lefr-front-00'; 'lefr-fix-02','lefr-front-01'; ...
        'lefr-spring-01','lefr-front-01'; 'lefr-spring-02','lefr-front-00'};
    for j = 1:size(attachments,1)
        id = find(strcmp(link_names,attachments{j,1}));
        parent = find(strcmp(link_names,attachments{j,2}));
        index = find([joints.child]==id);
        R = robot.zero.R(:,:,parent);
        joints(index).parent = parent;
        joints(index).translation = R'*(robot.zero.p(:,id)-robot.zero.p(:,parent));
        joints(index).rotation = R'*robot.zero.R(:,:,id);
        joints(index).type = 'fixed';
        joints(index).q = 0;
    end
    moving = 0;
    for j = 1:numel(joints)
        if ~strcmp(joints(j).type,'fixed')
            moving = moving+1;
            joints(j).q = moving;
        end
    end
    robot.joints = joints;
    robot.nq = moving;
    robot.zero = mixmpc_robot_fk(robot,zeros(moving,1));
end
assert(moving==14, 'Expected four actuators, eight passive hinges and two wheels.');
for k = 1:2
    prefix = {'lf','rf'};
    if is001
        if k==1
            names = {'lefr-front-00','lefr-front-01','lefr-rear-00','lefr-rear-01', ...
                'lefr-rear-02','lefr-rear-03','lefr-spring-01','lefr-fix-02', ...
                'lefr-fix-01','lefr-spring-02','lefr-front-wheel'};
        else
            names = {'right-front','right-front-01','right-rear-00','right-rear-01', ...
                'right-rear-02','right-rear-03','right-fix-01','right-fix-02', ...
                'right-spring-01','right-spring-02','right-front-wheel'};
        end
        [found, frames] = ismember(names,link_names);
        ids = arrayfun(@(id)find([joints.child]==id),frames);
    else
        names = strcat(prefix{k},{'0','1','00','01','02','03','04','05','001','002','wheel'});
        [found, ids] = ismember(names,joint_names);
        frames = [joints(ids).child];
    end
    assert(all(found), 'Missing joint in %s leg.',prefix{k});
    joint = joints(ids);
    axes = zeros(3,numel(joint));
    for j = 1:numel(joint)
        axes(:,j) = robot.zero.R(:,:,frames(j))*joint(j).axis;
    end
    active = [1 3];
    passive = [2 4 5 6];
    rotating = [active passive 11];
    sign_y = axes(2,rotating);
    assert(all(abs(abs(sign_y)-1)<1e-10), 'Leg axes are not parallel.');
    pins = [make_pin(frames(7),frames([1 5])), make_pin(frames(8),frames([2 6]))];
    if is001
        spring = [make_pin(frames(9),frames(2)), make_pin(frames(10),frames(1))];
        if k==1
            % The left marker's exported origin is displaced from its bore.
            % The upper-link STL bore agrees with the right pin's XZ center.
            right_pin = find(strcmp(link_names,'right-spring-02'));
            point = robot.zero.p(:,frames(10));
            point([1 3]) = robot.zero.p([1 3],right_pin);
            parent = frames(1);
            spring(2).point = robot.zero.R(:,:,parent)'*(point-robot.zero.p(:,parent));
        end
    else
        spring = [make_pin(frames(9),frames(1)), make_pin(frames(10),frames(2))];
    end
    robot.side(k) = struct('name',prefix{k}, 'active',[joint(active).q], ...
        'passive',[joint(passive).q], 'wheel',joint(11).q, 'q_sign',-sign_y(1), ...
        'active_sign',-axes(2,active), 'passive_sign',-axes(2,passive), ...
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
