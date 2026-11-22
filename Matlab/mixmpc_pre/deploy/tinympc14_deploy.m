function tinympc14_deploy(output_dir, rebuild)
%TINYMPC14_DEPLOY 混合 MPC 独立部署入口。
% 默认目标只改 tinympc14_config.m 的 cfg.OUT_DIR = '绝对路径';。
% tinympc14_deploy                         完整生成/验证，再写默认目标。
% tinympc14_deploy('E:\TEMP\mixmpc')       临时替换目标，不改配置。
% tinympc14_deploy('E:\TEMP\mixmpc',false) 校验现有表及来源哈希后复制，不重跑仿真。
% 补偿/VMC 的 Leg_* 函数及固件 port 实现仍按 solver/README.md 接入。
if nargin < 1, output_dir = ''; end
if nargin < 2, rebuild = true; end
here = fileparts(mfilename('fullpath'));
root = fileparts(here);
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(here); addpath(fileparts(here)); mixmpc_paths();
p = tinympc14_config();
if isempty(output_dir), output_dir = p.OUT_DIR; end
output_dir = mpc_output_path(output_dir);
if rebuild
    tinympc14_build_and_verify();
else
    run_check();
end
% Assemble and verify the split package under build before touching the target.
package_dir=fullfile(root,'build','polynomial_package');
if ~isfolder(package_dir), mkdir(package_dir); end
for item={'tinympc14.c','tinympc14.h'}
    copy_changed(fullfile(root,'deploy','solver',item{1}),fullfile(package_dir,item{1}));
end
for item={'tinympc14_data.c','tinympc14_data.h'}
    copy_changed(fullfile(p.STAGING_DIR,item{1}),fullfile(package_dir,item{1}));
end
split=tinympc14_export_split(package_dir);
tinympc14_deployment_check(split,false);
% Verify all six files before publication; preserve timestamps for unchanged units.
names={'tinympc14.c','tinympc14.h','tinympc14_data.c','tinympc14_data.h', ...
    'tinympc14_schedule.c','tinympc14_cost.c'};
hashes=cellfun(@(name) tinympc14_sha256(fullfile(split,name)),names,'UniformOutput',false);
if ~isfolder(output_dir), mkdir(output_dir); end
for k=1:numel(names)
    destination=fullfile(output_dir,names{k});
    copy_changed(fullfile(split,names{k}),destination);
    assert(strcmp(tinympc14_sha256(destination),hashes{k}),'Deployment hash mismatch: %s',names{k});
end
copy_changed(fullfile(split,'tinympc14_deployment.sha256'),fullfile(output_dir,'tinympc14_deployment.sha256'));
tinympc14_deployment_check(output_dir);
fprintf('Leg4 mixed MPC deployment: PASS (%d split files) -> %s\n',numel(names),output_dir);

end

function copy_changed(source,destination)
% Windows 上编辑器可能映射旧源文件。同目录写完再换名，避免直接截断正在读取的内容。
if isfile(destination) && strcmp(tinympc14_sha256(source),tinympc14_sha256(destination)), return; end
temporary = [tempname(fileparts(destination)) '.tmp'];
cleanup = onCleanup(@() remove_temporary(temporary)); %#ok<NASGU>
copyfile(source,temporary);
tinympc14_replace_generated(temporary,destination);
end

function remove_temporary(file)
if isfile(file), delete(file); end
end

function run_check()
run(which('s07_tinympc14_static_check'));
% Existing and fast-built tables must pass the same off-grid model gate as
% a full build before any bytes are copied to the firmware target.
run(which('s09_tinympc14_fused_verify'));
end
