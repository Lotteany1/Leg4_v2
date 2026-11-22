function [u0, info] = tinympc14_admm(mdl, x0, xref, uref, cfg, warm, opts)
% TINYMPC14_ADMM  Leg4 融合 MPC 的 ADMM 参考实现
% 与 tinympc_admm_ref 同序：① 线性代价 ② 反向 ③ 前向 ④ 投影 ⑤ 对偶 ⑥ 残差
% S5a 用 opts.fallback='ref'、opts.return_proj=false 逐行对拍 10/4
% S5b 用 opts.fallback='uss'、opts.return_proj=true 跑 14/6 仿射/箱/速率
%
%   mdl : Ad/Bd/Kinf/Pinf/QuuInv/AmBKt/q_diag/r_diag/u_min/u_max/x_min/x_max
%   opts: fd/APf/BPf/df_rate/u_prev/fallback/return_proj/fb_ffwd
%   坏值：u 非有限即按未收敛跳出；uss 回退 NaN 清零后再钳位
%
% 版本：v0.3  日期：2026-10-20

nx = numel(mdl.q_diag);
nu = numel(mdl.r_diag);
NH = cfg.NH;
NN = NH - 1;
rho = cfg.rho;
input_penalty = tinympc14_input_penalty(mdl,cfg);

if nargin < 6
    warm = [];
end
if nargin < 7
    opts = struct();
end
if ~isfield(opts, 'fd'),          opts.fd = zeros(nx, 1);       end
if ~isfield(opts, 'first_fd'),    opts.first_fd = opts.fd;      end
if ~isfield(opts, 'APf'),         opts.APf = zeros(nx, 1);      end
if ~isfield(opts, 'BPf'),         opts.BPf = zeros(nu, 1);      end
if ~isfield(opts, 'df_rate'),     opts.df_rate = Inf(nu, 1);    end
if ~isfield(opts, 'u_prev'),      opts.u_prev = [];             end
if ~isfield(opts, 'fallback'),    opts.fallback = 'uss';        end
if ~isfield(opts, 'return_proj'), opts.return_proj = true;      end
if ~isfield(opts, 'fb_ffwd'),     opts.fb_ffwd = zeros(nu, 1);  end

fd = opts.fd(:);
step_fd = repmat(fd,1,NN);
step_fd(:,1) = opts.first_fd(:);
fb_ffwd = opts.fb_ffwd(:);
APf = opts.APf(:);
BPf = opts.BPf(:);
rate = opts.df_rate(:);
if isscalar(rate)
    rate = repmat(rate, nu, 1);
end

umax = mdl.u_max(:);  umin = mdl.u_min(:);
xmax = mdl.x_max(:);  xmin = mdl.x_min(:);
st_off = all(abs(xmax) >= 1e5);

if isempty(opts.u_prev)
    u_prev = min(max(uref(:,1), umin), umax);
else
    u_prev = opts.u_prev(:);
end
u_prev(~isfinite(u_prev)) = 0;
u_prev = min(max(u_prev, umin), umax);

if isempty(warm)
    warm = struct();
    warm.v = zeros(nx, NH);  warm.g = zeros(nx, NH);
    warm.z = zeros(nu, NN);  warm.y = zeros(nu, NN);
end
v = warm.v;  g = warm.g;
z = warm.z;  y = warm.y;

q = zeros(nx, NH);  r = zeros(nu, NN);
p = zeros(nx, NH);  d = zeros(nu, NN);
x = zeros(nx, NH);  u = zeros(nu, NN);
x0 = x0(:);

converged = false;
pri = Inf;  dua = Inf;  it = 0;
if isfield(mdl,'rc')
    rc=mdl.rc;
else
    rc=tinympc14_riccati_cache(mdl,cfg);
end
AP=zeros(nx,NN); BP=zeros(nu,NN);
for k=1:NN
    Pf=rc.Pnext(:,:,k)*step_fd(:,k);
    AP(:,k)=rc.At(:,:,k)*Pf;
    BP(:,k)=mdl.Bd'*Pf;
end

for it = 1:cfg.max_iter
    z_prev = z;
    v_prev = v;

    q = -mdl.q_diag(:) .* xref - rho * (v - g);
    r = -mdl.r_diag(:) .* uref - input_penalty .* (z - y);
    % Pinf is the augmented terminal Hessian: base terminal weight is Pinf-rho*I.
    p(:,NH) = -(mdl.Pinf-rho*eye(nx))*xref(:,NH) - rho*(v(:,NH)-g(:,NH));

    for k = NN:-1:1
        d(:,k) = rc.Qu(:,:,k) * (mdl.Bd' * p(:,k+1) + r(:,k) + BP(:,k));
        p(:,k) = q(:,k) + rc.At(:,:,k) * p(:,k+1) - rc.K(:,:,k)' * r(:,k) + AP(:,k);
    end

    x(:,1) = x0;
    for k = 1:NN
        u(:,k) = -rc.K(:,:,k) * x(:,k) - d(:,k);
        x(:,k+1) = mdl.Ad * x(:,k) + mdl.Bd * u(:,k) + step_fd(:,k);
    end

    z = tinympc14_project_input(u+y, umin, umax, rate, u_prev);

    if st_off
        v = x;
    else
        v(:,1) = x(:,1);
        v(:,2:NH) = min(max(x(:,2:NH) + g(:,2:NH), xmin), xmax);
        g(:,2:NH) = g(:,2:NH) + x(:,2:NH) - v(:,2:NH);
    end

    y = y + (u - z);

    if st_off
        pri = max(abs(u - z), [], 'all');
    else
        pri = max(max(abs(u - z), [], 'all'), max(abs(x - v), [], 'all'));
    end
    dua = max(max(input_penalty .* abs(z - z_prev), [], 'all'), rho * max(abs(v - v_prev), [], 'all'));

    if ~isfinite(pri) || ~isfinite(dua) || ~all(isfinite(u(:)))
        break;
    end
    if pri < cfg.pri_tol && dua < cfg.dua_tol
        converged = true;
        break;
    end
end

if converged
    if opts.return_proj
        u0 = z(:,1);
    else
        u0 = u(:,1);
    end
else
    if strcmpi(opts.fallback, 'ref')
        u0 = -mdl.Kinf * (x0 - xref(:,1));
        u0 = min(max(u0, umin), umax);
    else
        if isfield(mdl, 'Bd_pinv')
            Bp = mdl.Bd_pinv;
        else
            Bp = pinv(mdl.Bd);
        end
        rhs = xref(:,1) - mdl.Ad * xref(:,1) - fd;
        uss = Bp * rhs;
        u0 = uss + fb_ffwd - mdl.Kinf * (x0 - xref(:,1));
        u0(~isfinite(u0)) = 0;
        u0 = min(max(u0, max(umin, u_prev - rate)), min(umax, u_prev + rate));
    end
end

info = struct('iters', it, 'converged', converged, 'pri', pri, 'dua', dua, ...
              'u', u, 'x', x, 'z', z, 'u0_raw', u(:,1), ...
              'v', v, 'g', g, 'y', y);
end
