function qr_build_run(D, output_dir, do_verify, env_name, files, gen_fcn, verify_fcn, publish_fcn)
%QR_BUILD_RUN Q/R-only 构建事务：产物先落 build/，核对通过后才发布到输出目录。
% 模型缓存缺失、损坏或与源码不符时立即停止，不会悄悄退回完整重建。
%
% 输入:
%   D           - 模块路径结构，需要 D.ROOT / D.OUT_DIR / D.BUILD_DIR
%   output_dir  - 目标目录；空则取 D.OUT_DIR
%   do_verify   - 是否执行 C 对拍；false 只跳过对拍，模型守卫与数值检查照跑
%   env_name    - 生成脚本读取输出目录用的环境变量名；空表示不走环境变量
%   files       - 通过后要发布的产物文件名（cell）；给了 publish_fcn 时忽略
%   gen_fcn     - 无参生成句柄，把产物写进 staging
%   verify_fcn  - 无参核对句柄，不合格必须抛错
%   publish_fcn - 可选。自定义发布句柄，签名为 publish_fcn(output_dir)，
%                 用于发布方式不是「从 staging 复制具名文件」的模块（如混合 MPC）。
if nargin < 2, output_dir = ''; end
if nargin < 3, do_verify = true; end
if nargin < 4, env_name = ''; end
if nargin < 8, publish_fcn = []; end
validateattributes(do_verify, {'logical','numeric'}, {'scalar','binary'});
[~, project] = fileparts(D.ROOT);
tool_root = fullfile(fileparts(D.ROOT), 'Tool');
old_path = path; old_dir = pwd; old_env = '';
if ~isempty(env_name), old_env = getenv(env_name); end
cleanup = onCleanup(@() restore_state(old_path, old_dir, env_name, old_env)); %#ok<NASGU>
addpath(D.ROOT, fullfile(tool_root, 'shared'), fullfile(tool_root, 'model'));
if isempty(output_dir), output_dir = D.OUT_DIR; end
output_dir = mpc_output_path(output_dir);
qr_build_cache('read', D.ROOT);
stage = fullfile(D.BUILD_DIR, 'fast', 'output');
if strcmpi(output_dir, stage), error('Output directory must differ from fast staging'); end
if ~isempty(env_name), setenv(env_name, stage); end
if ~isfolder(stage), mkdir(stage); end
cd(D.ROOT);
timer = tic;
fprintf('FAST %s: 只重算 Q/R 增益、拟合和代码；完整仿真未执行。\n', project);
gen_fcn();
if do_verify
    verify_fcn();
end

% Only verified, explicitly named files are published. Cached/raw C is never copied.
if ~isempty(publish_fcn)
    publish_fcn(output_dir);
else
    for k = 1:numel(files)
        assert(isfile(fullfile(stage, files{k})), 'Missing generated file: %s', files{k});
    end
    if ~isempty(files)
        if ~isfolder(output_dir), mkdir(output_dir); end
        for k = 1:numel(files)
            [ok, message] = copyfile(fullfile(stage, files{k}), fullfile(output_dir, files{k}), 'f');
            assert(ok, 'Publish failed: %s', message);
        end
    end
end
report = struct('project',project, 'cache_hit',true, 'c_verified',logical(do_verify), ...
    'full_validation',false, 'elapsed',toc(timer), 'output_dir',output_dir);
save(fullfile(D.BUILD_DIR, 'fast', 'report.mat'), 'report');
fprintf('FAST PASS: %.2f s -> %s（必要检查；非完整验证）\n', report.elapsed, output_dir);
end

function restore_state(old_path, old_dir, env_name, old_env)
path(old_path); cd(old_dir);
if ~isempty(env_name), setenv(env_name, old_env); end
end