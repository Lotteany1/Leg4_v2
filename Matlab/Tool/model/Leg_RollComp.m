%#codegen
function [dF_lft, dF_rgt, Fc_raw] = Leg_RollComp(L0, omega, v)
%LEG_ROLLCOMP  转弯向心力前馈: 腿长/yaw 角速度/车速 → 左右腿推力补偿  (codegen 版)
%
%   [dF_lft, dF_rgt, Fc_raw] = Leg_RollComp(L0, omega, v)
%
%   模型 (前馈, 与侧倾角 PID 反馈叠加使用, 参考 Leg_SpringF 的用法):
%   转弯向心加速度 a_c = v*omega 作用在质心高度 h 处, 产生绕纵轴的滚转
%   惯性力矩 tau = M_SUP*a_c*h, 由左右腿竖直推力差平衡:
%       (dF_rgt - dF_lft) * (TRACK/2) = tau   (对称分配: 一腿 +Fc, 另一腿 -Fc)
%       =>  Fc = M_SUP * v * omega * h / TRACK                          [N]
%   质心高度用腿长近似: h = L0 + H_OFF (机身支撑质心在 pitch 轴上方的偏置,
%   腿/机身质量的质心差折进 H_OFF 一并标定)。
%
%   输出用法 (与固件 balance.c 腿长环输出叠加):
%       leg_control[lft].F = PID_out + ... + dF_lft;
%       leg_control[rgt].F = PID_out + ... + dF_rgt;
%
%   极性: omega > 0 = 左转 (俯视逆时针), v > 0 = 前进。前进左转时车身向外(右)
%   倾 → 右腿(外侧)加载 dF_rgt = +Fc, 左腿(内侧)卸载 dF_lft = -Fc; 倒车/右转
%   时 v*omega 变号, 补偿方向自动翻转, 无需显式判内外侧。
%
%   输入 (single/float 标量):
%       L0    - 等效腿长 [m], 传两腿平均值即可 (左右腿不等长对 h 的影响是二阶小量)
%       omega - yaw 角速度 [rad/s]
%       v     - 车身前进速度 [m/s]
%
%   输出 (固定 single，对应固件 float):
%       dF_lft - 左腿推力补偿量 [N], 直接加进左腿竖直推力指令 F
%       dF_rgt - 右腿推力补偿量 [N]
%       Fc_raw - 限幅前的单侧等效力 [N]；模型外部扰动为 [Fc_raw; -Fc_raw]，不下发电机
%
%   保护:
%     1) L0 钳位 [0.17, 0.34] m: 腿长标称工作区间 (与固件 LEG_LEN_CLAMP 一致)
%     2) Fc 限幅 ±F_MAX: 内侧腿卸载过度会把腿顶离地面, 参考实现缺这道限幅
%        (其 TURN_RATE 限幅是死代码), 高速转弯时 Fc 无界 —— 本实现补上
%
%   See also: Leg_SpringF, Leg_Comp, Leg_Trq

% ==================== 标定参数 (唯一来源: config/leg_config.m 的 par) ====================
% 单位: 质量 kg, 长度 m, 力 N。固定 single，对应固件 float。
[~, par] = leg_config();
M_SUP  = single(par.M_SUP);          % 轮以上支撑质量 = 整车总质量减两轮质量 [kg]
                                     % (轮与轮毂电机在地面高度, 不产生滚转力矩, 不计入)
H_OFF  = single(par.l_c);            % 支撑质心在 pitch 轴上方的偏置高度 h = L0 + H_OFF [m]
                                     % //! TODO: 占位值, 须按整车质心实测标定
TRACK  = single(par.RollComp_TRACK); % 两轮接地点间距 (全轮距) [m]
                                     % 用户确认全轮距 0.42551 m，半轮距为 0.212755 m
F_MAX  = single(par.RollComp_FMAX);  % 单侧补偿力限幅 [N], 防内侧腿被卸载到离地

% ==================== 输入钳位 (标称区间内是恒等映射) ====================
L0 = min(max(L0, single(par.Len_min)), single(par.Len_max));

% ==================== 向心力前馈 ====================
h  = L0 + H_OFF;                                   % 支撑质心离地高度 [m]
Fc_raw = M_SUP .* v .* omega .* h / TRACK;     % 转弯力矩的单侧等效力 [N]，保留饱和后的残余扰动
Fc = min(max(Fc_raw, -F_MAX), F_MAX);

dF_rgt =  Fc;                                      % 外侧(转弯外沿)加载
dF_lft = -Fc;                                      % 内侧卸载

end
