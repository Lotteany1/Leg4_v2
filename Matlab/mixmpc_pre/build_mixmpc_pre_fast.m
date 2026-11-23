    function build_mixmpc_pre_fast(output_dir, do_verify)
% Q/R-only: 改 tinympc14_config.m 的 Q/R-BLOCK 块后运行本入口。
% 需要先跑过一次完整构建；模型缓存缺失或与源码不符会立即停止，不会退回完整重建。
% build_mixmpc_pre_fast('E:\TEMP\output') 在 build/ 内生成并核对后再发布。
% 默认跳过横滚合同和主机 C 对拍；传入 (..., true) 可启用。
if nargin < 1, output_dir = ''; end
if nargin < 2, do_verify = false; end
build_mixmpc_pre(output_dir, do_verify, 'fast');
end