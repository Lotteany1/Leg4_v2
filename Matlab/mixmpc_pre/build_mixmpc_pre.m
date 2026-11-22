function build_mixmpc_pre(output_dir, do_verify, mode)
% 混合 MPC 的完整生成与部署；只改 Q/R 时用 build_mixmpc_pre_fast。
% Build/validate and deploy mixed MPC; false checks and deploys existing tables.
if nargin < 1, output_dir = ''; end
if nargin < 2, do_verify = true; end
if nargin < 3, mode = 'full'; end
old_path = path; old_dir = pwd;
cleanup = onCleanup(@() restore_state(old_path, old_dir)); %#ok<NASGU>
root = fileparts(mfilename('fullpath'));
addpath(root);
D = mixmpc_paths();
leg_source_check();
if isempty(output_dir), output_dir = tinympc14_config().OUT_DIR; end
output_dir = mpc_output_path(output_dir);
if strcmp(mode, 'fast')
    qr_build_run(D, output_dir, do_verify, '', {}, @fast_generate, @fast_verify, @publish);
    return;
end
tinympc14_deploy(output_dir, do_verify);
end

function fast_generate()
% 复用标称/LPV/ZOH 缓存重算增益与拟合表，再核对生成表与当前源码清单一致。
D = mixmpc_paths();
cache = qr_build_cache('read', D.ROOT);
p = tinympc14_config();
timer = tic;
tinympc14_cache_generate(true,p.STAGING_DIR,p,cache);
fprintf('FAST nominal: %.2f s\n',toc(timer));
timer = tic;
tinympc14_lpv_generate(true,cache);
fprintf('FAST terminal/codegen: %.2f s\n',toc(timer));
tinympc14_zoh_generate(true,p.STAGING_DIR,p,cache);
tinympc14_data_manifest('tinympc14_data','deploy/s07_tinympc14_zoh_data.m');
clear tinympc14_lpv_eval tinympc14_zoh_apply
run_step(which('s07_tinympc14_static_check'));
end

function fast_verify()
% 生成本次权重对应的固定/随机/补偿向量，再跑主机 C 对拍。
tinympc14_roll_contract_check();
tinympc14_scheduler_c_check();
D = mixmpc_paths();
model = qr_build_cache('read', D.ROOT);
tinympc14_generate_vectors(model.nominal.vector_xb0, model.nominal.vector_uss0);
script = fullfile(D.ROOT, 'verify', 'host', 'run_host_msvc.cmd');
[status, detail] = system(['"' script '"']);
fprintf('%s', detail);
assert(status == 0 && contains(detail, 'RESULT PASS'), 'Fast host C comparison failed');
end

function publish(output_dir)
% 混合 MPC 的发布是哈希核对后的整体部署，不是从 staging 复制具名文件。
tinympc14_deploy(output_dir, false);
end

function run_step(file)
% clear 开头的原始脚本在独立工作区运行，不清除入口函数的清理器。
run(file);
end

function restore_state(old_path, old_dir)
path(old_path); cd(old_dir);
end
