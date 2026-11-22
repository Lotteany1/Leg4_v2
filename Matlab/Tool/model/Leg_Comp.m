%#codegen
function [theta0,l_d] = Leg_Comp(l)
% 001 非轮腿机构的质心表。l 为髋到轮轴距离 [m]；输出质心偏角 [rad]、
% 髋到质心的矢状面半径 [m]。CAD 偏角定义为 atan2(-x_com,-z_com)，
% 与输出层 MgF 的同一角坐标配套。均匀表外保持端点，不作外推。
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
