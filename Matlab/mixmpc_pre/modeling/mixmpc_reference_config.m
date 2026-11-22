function model = mixmpc_reference_config(robot, hardware, candidate_spring)
% Parameters for the CAD reference, in left/right URDF order.
% Hardware supplies the candidate pin-distance force curve only.
model.robot = robot;
model.gravity = [0;0;-hardware.g];
if nargin<3 || candidate_spring
    model.spring = struct('peak',hardware.SprF,'short',hardware.Spr_pin_min, ...
        'long',hardware.Spr_pin_max);
    assert(model.spring.long>model.spring.short && model.spring.peak>=0, ...
        'Invalid gas-spring force curve.');
else
    % Inertia and gravity projections require no assumed spring force curve.
    model.spring=struct('disabled',true);
end
model.wheel_radius = [0.060;0.060];
model.contact_tolerance = struct('gap',2e-7,'velocity',2e-6,'impact_speed',1e-8, ...
    'normal_acceleration',1e-6,'release_force',1e-7);
model.wheel_link = arrayfun(@(s) s.frames(11),robot.side);
[~,idx] = ismember([robot.side.wheel],[robot.joints.q]);
model.wheel_axis = [robot.joints(idx).axis];
model.wheel_forward_sign = ones(1,2);
model.wheel_center_offset = zeros(3,2);
for side = 1:2
    id = model.wheel_link(side);
    axis = model.wheel_axis(:,side);
    world_axis = robot.zero.R(:,:,id)*axis;
    model.wheel_forward_sign(side) = sign(world_axis(2));
    mesh = stlread(fullfile(robot.mesh_dir,robot.links(id).mesh));
    vertex = robot.links(id).mesh_xyz+robot.links(id).mesh_R*mesh.Points';
    axial = axis'*vertex;
    radius = vecnorm(vertex-axis*axial);
    tread = axial(max(radius)-radius<1e-4);
    model.wheel_center_offset(:,side) = axis*(min(tread)+max(tread))/2;
end

% Tree coordinates start with the floating base followed by the 14 CAD joints.
model.drive = 6 + [[robot.side.active] [robot.side.wheel]];
model.keep = [1:6 model.drive];
model.passive = 6 + [robot.side.passive];
model.closure_pins = [robot.side.closure_spec];
rev = robot.joints([robot.joints.q]>0);
model.joint_limits = reshape([rev.limits],4,[])';
model.check_limit = setdiff(1:robot.nq,[robot.side.wheel],'stable');
end
