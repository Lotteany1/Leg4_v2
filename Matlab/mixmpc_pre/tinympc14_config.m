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

% 001 full CAD export. Left/right leg and wheel parameters use symmetric averages.
[~, hardware] = leg_config();
cfg.g      = hardware.g;   % 重力加速度                          [m/s^2]
cfg.R_w    = hardware.wheel_radius;       % 轮半径                              [m]
cfg.R_l    = hardware.half_track;  % 轮中心线到机体对称面的实测半距 [m]
cfg.l_c    = hardware.l_c;         % 机身质心高出髋转轴的标定偏置 [m]
cfg.m_w    = hardware.m_wheel;     % 单个轮与轮毂电机 [kg]
cfg.m_l    = hardware.m_leg;       % 单侧轮上腿机构 [kg]
cfg.m_b    = hardware.m_body;      % 机身与装车载荷，扣除两腿两轮 [kg]
cfg.M_SUP  = hardware.M_SUP;       % 轮上总质量，m_b+2*m_l，用于横向载荷转移 [kg]
cfg.I_w = hardware.I_w;  % 轮 + 轮毂电机绕轮轴转动惯量         [kg*m^2]
cfg.I_b = hardware.I_b;  % 机身俯仰转动惯量                    [kg*m^2]
cfg.I_z = hardware.I_z;   % 名义全车绕两胎中点竖轴的锁定惯量，轮轴自转项另列 [kg*m^2]
cfg.I_z_body = hardware.I_z_body; % 同一竖轴的机身贡献 [kg*m^2]

% 平衡参数表：[腿长L, 轮侧力臂lw, 机体侧力臂lb, 腿惯量Il]，单位 m/m/m/kg*m^2。
[~, cad, cfg.Leg_data, height_roll, cfg.TURN_DATA, cfg.SUPPORT_DATA,cfg.YAW_DATA] = leg_urdf_data();
[comp,~] = leg_config();
cfg.Leg_comp = comp(:,1:3);
cfg.HR_DATA = height_roll;
cfg.WORK_LEN_MIN = cad.Len_min;
cfg.WORK_MASS_MAX = max(height_roll(:,2));
cfg.WORK_LOAD_MIN = min(height_roll(:,5));
cfg.WORK_LOAD_MAX = max(height_roll(:,5));
cfg.LEG_RANGE = [cad.LPV_min,cad.Len_max];
cfg.CAD_MODEL = 1;
cfg.body_x = cad.body_x;
cfg.body_y = cad.body_y;

cfg.M_LEG = cfg.m_l;
cfg.H_BAR = 0.25; % 推导自检工作点 [m]
cfg.H_REF = 0.27; % 状态 x(13) 的高度坐标原点 [m]；目标高度由控制器传入

%% 双轮接地时的升降与横滚
% CAD no-lateral-slip contact tangent, including the H/roll kinetic cross term.
% M_V comes from kinetic energy; M_LOAD is the gravity-potential gradient / g.
cfg.M_V = hardware.M_V;
cfg.M_LOAD = hardware.M_LOAD;
cfg.R_TRK   = cfg.R_l; % 双轮支撑闭链的实测半距 [m]；直立时左右腿力矩臂分别为 ±R_TRK
cfg.I_ROLL = hardware.I_ROLL; % constrained roll inertia at H_REF, not locked CoM inertia [kg*m^2]
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
cfg.COMP.ROLL_LEN_CLAMP = cfg.LEG_RANGE;
cfg.K_CF  = 0.0;
cfg.H_OFF = interp1(cfg.SUPPORT_DATA(:,1),cfg.SUPPORT_DATA(:,2),cfg.H_REF,'linear');

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
