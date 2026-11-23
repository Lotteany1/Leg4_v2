function report = verify_mixmpc_pre(mode, varargin)
% 统一验证入口；默认检查日常模型，URDF 只用于显式选择的物理对照。
% verify_mixmpc_pre                  模型不变量及 VL-WIP 动量/功率检查。
% verify_mixmpc_pre('urdf', ...)      完整/训练 URDF 交叉校核，参数见 run_modeling_validation。
% verify_mixmpc_pre('p2', ...)        闭链动力学与接触验证，参数见 run_p2_validation。
if nargin < 1, mode = 'model'; end
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
root = fileparts(mfilename('fullpath'));
addpath(root);
mixmpc_paths();
switch mode
    case 'model'
        tinympc14_audit_regression();
        verify_mixmpc_vlwip();
        report = struct('model', true, 'vlwip', true);
    case 'urdf'
        addpath(fullfile(root, 'modeling'));
        report = run_modeling_validation(varargin{:});
    case 'p2'
        addpath(fullfile(root, 'modeling'));
        report = run_p2_validation(varargin{:});
    otherwise
        error('mixmpc:VerificationMode', 'Unknown verification mode: %s', mode);
end
end
