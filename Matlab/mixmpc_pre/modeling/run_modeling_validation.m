function result = run_modeling_validation(urdf_dir, output_dir)
% Reproduce the current offline model audit and write MATLAB and JSON results.
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
if nargin<1 || isempty(urdf_dir)
    urdf_dir = 'E:/轮腿资料/URDF/chuanliantui_new_1/chuanliantui_new_1/urdf';
end
if nargin<2, output_dir = fullfile('E:/TEMP','mixmpc_modeling_20261117','validation'); end
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(root,here);
mixmpc_paths();
if ~isfolder(output_dir), mkdir(output_dir); end
full_urdf = fullfile(urdf_dir,'chuanliantui.urdf');
train_urdf = fullfile(urdf_dir,'chuanliantui_train.urdf');
result.urdf = struct('full',full_urdf,'training',train_urdf, ...
    'full_sha256',tinympc14_sha256(full_urdf),'training_sha256',tinympc14_sha256(train_urdf));
robot = mixmpc_robot_config(full_urdf);
sources = unique([robot.side.frames]);
for k = 1:numel(sources)
    mesh = robot.links(sources(k)).mesh;
    result.urdf.leg_mesh(k) = struct('file',mesh, ... %#ok<AGROW>
        'sha256',tinympc14_sha256(fullfile(robot.mesh_dir,mesh)));
end
result.mechanism = mixmpc_mechanism_audit(robot);
result.contact = mixmpc_contact_audit(robot,0.060);
[~, hardware] = leg_config();
result.parameters = struct('cad_mass_kg',result.mechanism.total_mass_kg, ...
    'hardware_reference',hardware, 'hardware_wheel_radius_m',0.0525, ...
    'hardware_source_sha256',tinympc14_sha256(which('leg_config')));
result.parameters.height_roll = mixmpc_height_roll_audit(robot);
result.nominal = verify_mixmpc_urdf(train_urdf,0.060);
verify_mixmpc_vlwip;
p = tinympc14_urdf_config(train_urdf,0.060);
p.URDF_USE_SPRING = true;
p.R_DIAG(1:2) = 8100; % Replay the published pre-migration C baseline.
result.roll_baseline_weights = struct('Q',p.Q_DIAG,'R',p.R_DIAG);
[result.terminal_grid, terminal] = verify_mixmpc_terminal_grid(p);
[result.roll_cases, traces] = mixmpc_roll_baseline(p,terminal);
result.scope = ['CAD loop and contact kinematics, and training-model roll diagnostics. ' ...
    'Closed-chain dynamics and hardware calibration are pending.'];
save(fullfile(output_dir,'modeling_validation.mat'),'result','traces');
fid = fopen(fullfile(output_dir,'modeling_validation.json'),'w','n','UTF-8');
assert(fid>=0, 'Could not open validation metrics for writing.');
close = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(result,'PrettyPrint',true));
for k = 1:numel(result.roll_cases)
    c = result.roll_cases(k);
    fprintf('roll %+.0f deg, %+.1f rad/s, margin %.0f mm: %s at %.3f s\n', ...
        c.initial_roll_deg,c.initial_roll_rate_rad_s,1000*c.reference_margin_m,c.status,c.end_time_s);
end
fprintf('Modeling validation results: %s\n',output_dir);
end
