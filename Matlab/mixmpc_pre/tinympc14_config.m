function cfg = tinympc14_config()
% 14 态 MPC 的实车标定与部署配置。共享质量、半轮距来自 Tool/leg_config。
% x=[s ds yaw dyaw psiL dpsiL psiR dpsiR pitch dpitch roll droll H-H_REF dH]。
% s 为左右髋平均前进位置 [m]，H 为髋中点相对轮心平均高度 [m]。
% 角度用 rad，角速度用 rad/s，线速度用 m/s。psi 为含俯仰的矢状面腿角。
% 正 roll 对应左侧升高。u=[Twl Twr TBl TBr FL FR]，轮髋力矩 [N·m]，单腿轴力 [N]。
% 输出：Tw_motor=-Tw，Tp=TB+MgF，Fmotor=clip(F+Fc)-Fs。

%% 模型与求解
cfg.NX = 14;
cfg.NU = 6;
cfg.NH = 8;          % 7 个控制区间，共 14 ms。
cfg.Ts = 0.002;      % ZOH 节点 2 ms；控制 1 ms，后台求解 4 ms。
cfg.rho      = 0.001;
cfg.max_iter = 100;
cfg.pri_tol  = 1e-4;
cfg.dua_tol  = 1e-7; % 随代价缩放的对偶残差阈值。

cfg.CACHE_REF = '';
addpath(fileparts(mfilename('fullpath')));
D = mixmpc_paths();
cfg.OUT_DIR = fullfile(fileparts(fileparts(D.ROOT)),'Wheel_Leg_Code','Code','Model');
cfg.STAGING_DIR = D.STAGING_DIR;
cfg.WRITE = false;

%% Q/R：改动后重建增益和 data
% >>> Q/R-BLOCK-BEGIN >>>
%              s   ds   yaw  dyaw  psiL  dpsiL  psiR  dpsiR    pitch   dpitch  roll  droll  H-H_REF dH
cfg.Q_DIAG = [10000000, 212469, 2000000, 2500000, 45446271, 532251, 45446271, 532251, 183776190, 55166, 336000000, 115008, 1454724232, 444785] / 10000;
%              左轮  右轮  左髋  右髋  左腿力  右腿力
cfg.R_DIAG = [305120, 305120, 25716, 25716, 594, 594] / 10000;
% R 惩罚相对承重平衡输入的偏差；最后两项不是关节力矩或力变化率。
% Q(11:12) 分别惩罚 roll 角和角速度，Q(13:14) 分别惩罚高度误差和升降速度。
% Q 进入状态误差的二次代价，R 进入输入偏差的二次代价。两者都需重建终端表。
% <<< Q/R-BLOCK-END <<<

%% 平衡 10 态块参数（方程 (19)-(23) 口径）

% Chassis-only CAD export as supplied. Left/right leg and wheel masses are averaged.
[~, hardware] = leg_config();
cfg.g      = hardware.g;   % 重力加速度                          [m/s^2]
cfg.R_w    = 0.06;       % 轮半径                              [m]
cfg.R_l    = hardware.half_track;  % 轮中心线到机体对称面的实测半距 [m]
cfg.l_c    = hardware.l_c;         % 机身质心高出髋转轴的标定偏置 [m]
cfg.m_w    = hardware.m_wheel;     % 单个轮与轮毂电机 [kg]
cfg.m_l    = hardware.m_leg;       % 单侧轮上腿机构 [kg]
cfg.m_b    = hardware.m_body;      % 机身与装车载荷，扣除两腿两轮 [kg]
cfg.M_SUP  = hardware.M_SUP;       % 轮上总质量，m_b+2*m_l，用于横向载荷转移 [kg]
cfg.I_w = 0.002878686969512;  % 轮 + 轮毂电机绕轮轴转动惯量         [kg*m^2]
cfg.I_b = 0.02655031277436;  % 机身俯仰转动惯量                    [kg*m^2]
cfg.I_z = 0.01603872210918;   % 机身偏航转动惯量                    [kg*m^2]

% 平衡参数表：[腿长L, 轮侧力臂lw, 机体侧力臂lb, 腿惯量Il]，单位 m/m/m/kg*m^2。
cfg.Leg_data = [
    0.15, 0.1334830273321, 0.01651697266786, 0.01227550664342;
    0.16, 0.1390120640826, 0.02098793591744, 0.01241816832162;
    0.17, 0.1447331387935, 0.02526686120654, 0.01256546029749;
    0.18, 0.150613986338, 0.02938601366199, 0.01271801723798;
    0.19, 0.1566291846452, 0.03337081535477, 0.01287628898487;
    0.2, 0.1627584300802, 0.03724156991976, 0.01304059909027;
    0.21, 0.1689853093918, 0.04101469060817, 0.01321118254519;
    0.22, 0.1752964090584, 0.04470359094162, 0.01338821070608;
    0.23, 0.1816806585833, 0.0483193414167, 0.01357180811152;
    0.24, 0.188128839005, 0.05187116099502, 0.01376206402062;
    0.25, 0.1946332100268, 0.05536678997323, 0.01395904042558;
    0.26, 0.2011872235994, 0.05881277640061, 0.01416277764571;
    0.27, 0.2077853013723, 0.06221469862773, 0.01437329821342;
    0.28, 0.2144226599114, 0.0655773400886, 0.01459060951039;
    0.29, 0.2210951720328, 0.0689048279672, 0.01481470544553;
    0.3, 0.2277992557011, 0.07220074429895, 0.01504556734982;
    0.31, 0.2345317841246, 0.07546821587542, 0.01528316417429;
    0.32, 0.241290012224, 0.07870998777598, 0.01552745199878;
    0.33, 0.2480715157378, 0.0819284842622, 0.01577837277693;
    0.34, 0.2548741399687, 0.08512586003125, 0.01603585213749;
];

% 重力补偿表：[腿长, 质心角, 质心半径]，单位 m/rad/m。
cfg.Leg_comp = [
    0.15, 1.372649007992, 0.08397760011694;
    0.16, 1.320220844194, 0.0847021984867;
    0.17, 1.270545357477, 0.08548256740754;
    0.18, 1.223201943256, 0.08631428702563;
    0.19, 1.177858147902, 0.08719381096631;
    0.2, 1.134246062124, 0.0881181917153;
    0.21, 1.092145689775, 0.08908490457207;
    0.22, 1.051372979482, 0.09009173211467;
    0.23, 1.011771035213, 0.09113668678752;
    0.24, 0.9732035284583, 0.092217958058;
    0.25, 0.9355496517135, 0.09333387572222;
    0.26, 0.8987001550964, 0.09448288400979;
    0.27, 0.8625541382851, 0.09566352301978;
    0.28, 0.8270163538097, 0.09687441520207;
    0.29, 0.7919948299811, 0.09811425535841;
    0.3, 0.7573986507776, 0.09938180313777;
    0.31, 0.7231357398032, 0.1006758773413;
    0.32, 0.6891104862522, 0.1019953515963;
    0.33, 0.6552210189331, 0.1033391511516;
    0.34, 0.6213558704893, 0.1047062507323;
];

cfg.M_LEG = cfg.m_l;
cfg.H_BAR = 0.25; % 推导自检工作点 [m]
cfg.H_REF = 0.27; % 状态 x(13) 的高度坐标原点 [m]；目标高度由控制器传入

%% 双轮接地时的升降与横滚
% CAD closure at H_REF, with wheel heights fixed. This point is outside the export knee limits.
% M_V comes from kinetic energy. M_LOAD is the slope of gravity potential divided by g.
cfg.M_V = 9.072155406593;
cfg.M_LOAD = 9.314671533857;
cfg.R_TRK   = cfg.R_l; % 双轮支撑闭链的实测半距 [m]；直立时左右腿力矩臂分别为 ±R_TRK
cfg.I_ROLL = 0.310399869075; % locked CAD about assembly COM at H_REF [kg*m^2]
cfg.B_DAMP  = 8.0;     % 总竖直黏性阻尼 [N·s/m]，经验参数
cfg.K_EFF   = 0.0;     % 净竖直刚度 [N/m]，气弹簧由输出层补偿
cfg.G_ROLL  = 0.0;     % A(12,11) 的附加项 [s^-2]，外部倾覆预测单独计算
cfg.ROLL_TIP_GAIN = 0.5; % 静载下的合成倾覆力矩系数 [1]；腿力虚功已扣除对应的静载几何矩

%% 力箱（总支持力口径）与速率
cfg.F_MIN      = 20.0;  % 单腿总支持力下限 [N]，含气弹簧
cfg.F_MAX      = 150.0; % 单腿总支持力上限 [N]
cfg.H_CEIL_REF = 0.27;  % 力箱自检用参考高度 [m]
cfg.SPRING_IN_BOX = false;
cfg.DF_RATE    = 6.0;  % 每个 Ts 预测区间、每侧支持力最大增量 [N]
cfg.A_MAX      = 0.60; % 状态箱允许的横滚角绝对值 [rad]
cfg.TAU_JOINT_CEIL = 54.0; % 运动学自检中的关节力矩上限假设 [N·m]

%% 补偿插件（模型外三笔：MgF / Fs / 向心）
cfg.COMP.LEG_GRAV_FF = cfg.m_l;
cfg.COMP.ROLL_TRACK  = 2*cfg.R_TRK;
cfg.COMP.ROLL_CLAMP  = 60.0;
cfg.COMP.ROLL_LEN_CLAMP = [0.16, 0.34];
cfg.K_CF  = 0.0;
cfg.H_OFF = cfg.l_c;

%% 状态箱（14 态）与输入箱（6 路）
cfg.x_min = [-1e6, -1e6, -1e6, -1e6, -1e6, -1e6, -1e6, -1e6, -1e6, -1e6, -cfg.A_MAX, -1e6, -1e6, -1e6];
cfg.x_max = [ 1e6,  1e6,  1e6,  1e6,  1e6,  1e6,  1e6,  1e6,  1e6,  1e6,  cfg.A_MAX,  1e6,  1e6,  1e6];

wheel_limit = 16384 / 3326.29; % code_config.h: M3508_OUT_MAX / M3508_TO_TORQUE
cfg.u_min = [-wheel_limit, -wheel_limit, -40.00, -40.00, cfg.F_MIN, cfg.F_MIN];
cfg.u_max = [ wheel_limit,  wheel_limit,  40.00,  40.00, cfg.F_MAX, cfg.F_MAX];

%% 7 自由度支线专用（本支线独有）

cfg.sym7.Mv_mode    = 'locked';
cfg.sym7.m_leg_near = 0.55;

cfg.sym7.affine_mode = 'B';

cfg.sym7.psi_nom = 0.1;

cfg.sym7.psi_sign      = +1;
cfg.sym7.psi_inc_pitch = 0;

cfg.sym7.k_eff_alt = 149.0;

cfg.sym7.k_eff_mode = 'fixed';
cfg.sym7.k_eff_df_h = 0.01;

cfg.sym7.theta_star_guess = 0.05;
cfg.sym7.ik_bracket = [0.4, 1.7];

cfg.sym7.sched_l = 0.27;
cfg.sym7.sched_psi = 0.00;
cfg.sym7.pitch_ref = -0.05; % rad, current firmware BALANCE_PITCH_REF
cfg.sym7.psi_scan = [0, 0.05, 0.10, 0.15];

cfg.sym7.sw = struct('kin_ldot', 1, 'F_pitch', 0, 'F_H_proj', 0, 'roll_closure', 1, ...
                     'Ts_H', 0, 'Ts_alpha', 0);
% sw：s01 等推导自检的配置；不代表当前部署使用的开关组合。
cfg.sym7.sw_none = struct('kin_ldot', 0, 'F_pitch', 0, 'F_H_proj', 0, 'roll_closure', 0, ...
                          'Ts_H', 0, 'Ts_alpha', 0);
% sw_none：关闭增量耦合的对照配置；腿长参数调度本身仍然存在。
cfg.sym7.sw_fused = struct('kin_ldot', 1, 'F_pitch', 0, 'F_H_proj', 1, 'roll_closure', 1, ...
                           'Ts_H', 1, 'Ts_alpha', 1);
% sw_fused：s03/s04d 生成部署表及当前运行时验证使用的配置。
% F_pitch 保持关闭，避免水平内力重复计入；Ts_H/Ts_alpha 是轮矩投影。
% kin_ldot/roll_closure 仅用于推导检查，不代表完整变腿长多体动力学。
end
