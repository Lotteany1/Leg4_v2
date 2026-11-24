function [A,B,f,residual,terminal_center] = tinympc14_scheduled_ab(p, left, right, psi, roll_height)
% Continuous model at one LPV scheduler input, with no toolbox dependency.
coder.inline('never'); % Keep the continuous model scratch separate from ZOH.
% Invert the firmware height observation on level ground. The deployment
% table uses one common sagittal angle for both measured leg lengths.
vertical = cos(psi);
roll = atan((left-right)*vertical/(2*p.R_TRK));
height = 0.5*(left+right)*vertical*cos(roll);
% Offline terminal grids have no IMU sample and retain their calibrated height.
if nargin < 5 || ~isfinite(roll_height)
    roll_height = height;
end
sched = struct('H',height,'roll_height',roll_height,'alpha',roll, ...
    'lL',left,'lR',right,'psi',psi, ...
    'eq',struct('x_eq',zeros(p.NX,1),'u_eq',zeros(p.NU,1)));
[xe,ue,eq] = tinympc14_fused_eq(p,p.sym7.sw_fused,sched);
terminal_center = xe(5);
sched.eq = struct('x_eq',xe,'u_eq',ue);
[A,B,f] = tinympc14_fused_ab(p,p.sym7.sw_fused,sched);
residual = eq.res;
end
