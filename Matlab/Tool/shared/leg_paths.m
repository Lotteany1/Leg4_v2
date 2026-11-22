function D = leg_paths()
%LEG_PATHS Tool 的目录与部署目标；换输出位置只改 D.OUT_DIR 这一行。
% 本文件只放路径。物理量在 leg_config.m —— 那个文件带 %#codegen 且被 4 个 Coder 输入函数调用，
% 里面不能出现 fileparts(mfilename('fullpath')) 这类 codegen 不支持的构造。
D.ROOT = fileparts(fileparts(mfilename('fullpath')));
% ══ 部署：生成文件的输出位置（要改输出位置就改这一行）══
D.OUT_DIR = 'E:\DevelopLotteany1\0_Code\Leg4_v2_ultimate\Wheel_Leg_Code\Code\Math';
% 本地验证时可用 build_leg_tool('目标目录') 临时覆盖；不修改上述默认配置。
override = getenv('LEG4_TOOL_OUT_DIR');
if ~isempty(override), D.OUT_DIR = override; end
D.BUILD_DIR = fullfile(D.ROOT, 'build');
D.MODEL_DIR = fullfile(D.ROOT, 'model');
D.SHARED_DIR = fullfile(D.ROOT, 'shared');
D.SOLVER_DIR = fullfile(D.ROOT, 'solver');
D.CMSIS_DIR = fullfile(D.BUILD_DIR, 'cmsis');
D.CODEGEN_DIR = fullfile(D.BUILD_DIR, 'codegen_current', 'lib');
D.VERIFY_BUILD_DIR = fullfile(D.BUILD_DIR, 'verify', sprintf('process_%d', feature('getpid')));
% 可选：单独与已部署的固件目录比对，留空跳过（避免对同一目录自比较）。
D.FIRMWARE_COMPARE_DIR = '';
end
