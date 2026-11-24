function s = tinympc14_sched_pt(cfg, H, alpha, psi)
% 同一水平面双支撑。H 为髋中点比轮轴平均高出的距离 [m]。
% 正 roll 时左侧升高。psi 为左右共用的矢状面腿角 [rad]。
if nargin < 4 || isempty(psi)
    psi = cfg.sym7.psi_nom;
end
offset = cfg.R_TRK*sin(alpha);
vertical = cos(alpha)*cos(psi);
left = (H+offset)/vertical;
right = (H-offset)/vertical;
s = struct('H', H, 'alpha', alpha, 'psi', psi, ...
           'lL', left, 'lR', right, 'roll_height', 0.5*(left+right)*cos(alpha));
end
