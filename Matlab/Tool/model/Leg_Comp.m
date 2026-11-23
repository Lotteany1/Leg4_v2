%#codegen
function [theta0, l_d] = Leg_Comp(l)
%LEG_COMP  腿部质心偏移模型: 腿长 → 质心偏置角 / 质心到 pitch 轴力臂  (codegen 版)
%
%   [theta0, l_d] = Leg_Comp(l)
%
%   数据唯一来源: leg_config.m (与本函数同目录, 零额外路径依赖) 的腿长参数
%   查找表 (当前 0.15:0.01:0.32 m, 共 18 个均匀节点; 行数/范围以 leg_config.m 为准),
%   本函数只做分段线性插值, 超出表范围钳位到端点值, 不外推。
%   codegen 会把表常量折叠进生成的 C (static const), 固件侧无运行时依赖。
%
%   输入 (double/single 标量):
%       l      - 腿长 (pitch 转轴到轮轴的等效腿长)  [m]
%
%   输出 (类型随输入):
%       theta0 - 腿质心相对腿长向量的偏置角             [rad]
%       l_d    - 腿质心到 pitch 转轴的半径（重力力臂需投影） [m]
%
%   极性 (与 lqr/推导-v2.md §1.2/§8 一致, 表列 2/3 与模型 theta_l0/l_l_d
%   同号, 直接代入即可):
%       theta0 > 0 表示质心偏向 theta 正方向一侧 (后摆方向, 从左看逆时针)。
%       腿自重对 pitch 轴的力矩 (模型 §3.3/§3.4 重力项):
%           tau_g = m_leg * g * l_d * sin(theta + theta0)   % g = -9.81
%       重力补偿前馈力矩与之等大反向: tau_ff = -tau_g。
%       其中 theta 为模型腿角 (腿与竖直向下方向夹角, 正 = 后摆)。
%
%   See also: leg_data, Leg_Pos, Leg_Spd, Leg_Trq, Leg_InvTrq

% ==================== 查找表 (唯一来源: leg_config.m) ====================
% 列 = [l_leg, theta0, l_d, I_leg], 腿长均匀步长 → 直接定位, 无需搜索。
% cast(...,'like',l): 表类型随输入 (single 输入 → 固件侧为 float 常量)。
D = leg_config();
th0_tab = cast(D(:, 2), 'like', l);
ld_tab  = cast(D(:, 3), 'like', l);

% ==================== 分段线性插值 ====================
L0 = D(1, 1);                       % 表起点腿长 (m)
DL = D(2, 1) - D(1, 1);             % 表步长 (m)
NT = size(D, 1);                    % 表行数

u  = (l - L0) / DL;                 % 归一化表坐标 (第 k 个节点处 u = k-1)
u  = min(max(u, 0), NT - 1);        % 钳位到 [0, NT-1], 表外取端点
k0 = min(floor(u), NT - 2);         % 段左节点 (0-based), 保证 k+1 不越界
s  = u - k0;                        % 段内比例 ∈ [0, 1]
k  = int32(k0) + 1;                 % 1-based 索引

theta0 = th0_tab(k) + s .* (th0_tab(k + 1) - th0_tab(k));
l_d    = ld_tab(k)  + s .* (ld_tab(k + 1)  - ld_tab(k));

end
