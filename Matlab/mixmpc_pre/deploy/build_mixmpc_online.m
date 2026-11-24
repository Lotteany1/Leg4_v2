function report = build_mixmpc_online(output_dir, firmware_model_dir, model, wheel_radius)
% Build the TinyMPC package with the online scheduler, then run C comparison.
% model can be a training URDF path or an explicit TinyMPC configuration.
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
D = mixmpc_paths();
out = char(java.io.File(output_dir).getCanonicalPath());
solver_dir = fullfile(out,'solver');
generated_dir = fullfile(out,'generated');
host_dir = fullfile(out,'host');
if ~isfolder(solver_dir), mkdir(solver_dir); end

if nargin < 3
    p = tinympc14_config();
elseif isstruct(model)
    p = model;
else
    p = tinympc14_urdf_config(model,wheel_radius);
    p.URDF_USE_SPRING = true; % Real spring calibration is a hardware input.
end
[report,table] = build_mixmpc_online_model(generated_dir,p);
tinympc14_cache_generate(false,solver_dir,p);
tinympc14_zoh_generate(false,solver_dir,p);
schema = sprintf(['#define TINYMPC14_ONLINE_IFACE_VER 1\n' ...
    '#define TINYMPC14_LPV_PSI_MIN %.2ff\n#define TINYMPC14_LPV_PSI_MAX %.2ff\n' ...
    '#ifndef TINYMPC14_H_REF\n#define TINYMPC14_H_REF %.6ff\n#endif\n' ...
    '#ifndef TINYMPC14_PITCH_REF\n#define TINYMPC14_PITCH_REF %.6ff\n#endif\n'], ...
    table.angle_grid(1),table.angle_grid(end),p.H_REF,p.sym7.pitch_ref);
if isfield(p,'roll_height') && ~isfield(p,'URDF_SOURCE')
    schema = [schema sprintf('#define TINYMPC14_ROLL_H_MIN %.9ef\n#define TINYMPC14_ROLL_H_MAX %.9ef\n', ...
        p.roll_height(1),p.roll_height(end))];
end
if isfield(p,'URDF_SOURCE')
    schema = [schema sprintf(['#define TINYMPC14_URDF_MODEL 1\n' ...
        '#define TINYMPC14_L_MIN %.9ff\n#define TINYMPC14_L_MAX %.9ff\n' ...
        '#define TINYMPC14_M_TOTAL %.9ff\n#define TINYMPC14_M_WHEEL %.9ff\n'], ...
        table.length_grid(1),table.length_grid(end),tinympc14_m_tot(p),p.m_w)];
end
tinympc14_section(fullfile(solver_dir,'tinympc14_data.h'),'online',schema);
copyfile(fullfile(D.SOLVER_DIR,'tinympc14.c'),solver_dir,'f');
copyfile(fullfile(D.SOLVER_DIR,'tinympc14.h'),solver_dir,'f');
if isfield(p,'URDF_SOURCE')
    generate_urdf_comp(solver_dir,p.Leg_comp);
end
tinympc14_bundle_online(generated_dir,solver_dir);

p.ONLINE_TERMINAL = table;
tinympc14_online_runtime_check(p);
s = tinympc14_sched_pt(p,p.H_REF,0,0);
[xb,uss] = tinympc14_fused_eq(p,p.sym7.sw_fused,s);
tinympc14_generate_vectors(xb,uss,p,host_dir);
script = fullfile(D.VERIFY_DIR,'host','run_host_msvc.cmd');
[status,detail] = system(sprintf('call "%s" online "%s"',script,out));
fprintf('%s',detail);
assert(status==0 && contains(detail,'RESULT PASS'),'Online TinyMPC host comparison failed.');
write_manifest(out);
save(fullfile(out,'tinympc14_package_config.mat'),'p','report');
if nargin >= 2 && ~isempty(firmware_model_dir)
    tinympc14_install_online(out,firmware_model_dir);
end
fprintf('Online TinyMPC package verified: %s\n',out);
end

function generate_urdf_comp(folder,comp)
declaration = sprintf('extern const float tinympc14_urdf_comp[%d][3];',size(comp,1));
tinympc14_section(fullfile(folder,'tinympc14_data.h'),'urdf_comp',declaration);
fid = fopen(fullfile(folder,'tinympc14_data.c'),'a');
assert(fid>=0,'Could not write URDF compensation table.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
fprintf(fid,'\n/* URDF-derived leg length, CoM angle and radial distance. */\n');
fprintf(fid,'const float tinympc14_urdf_comp[%d][3] = {\n',size(comp,1));
fprintf(fid,'    {%.9ef, %.9ef, %.9ef},\n',comp.');
fprintf(fid,'};\n');
end

function write_manifest(root)
fid = fopen(fullfile(root,'tinympc14_online.sha256'),'w');
assert(fid>=0,'Could not write the online package manifest.');
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
for file = {'tinympc14.c','tinympc14.h','tinympc14_data.c','tinympc14_data.h'}
    name = ['solver/' file{1}];
    fprintf(fid,'%s *%s\n',tinympc14_sha256(fullfile(root,name)),name);
end
end
