function verify_mixmpc_vlwip()
% Check the symmetrical rolling model independently of TinyMPC.
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mixmpc_paths();
p = tinympc14_config();

xe = [0;0;.27;0;0;0];
assert(norm(tinympc14_vlwip_sagittal(p,xe,[0;p.M_SUP*p.g]),Inf)<1e-10, ...
    'The axial force at equilibrium must support the mass above the wheels.');

% Keep both lean and extension rates nonzero to exercise velocity coupling.
x = [.1;.22;.29;.8;-.7;.45];
probe = 1e-6;
for torque = [0, 3, -3]
    u = [torque;0.8*p.M_SUP*p.g];
    [xd,instant] = tinympc14_vlwip_sagittal(p,x,u);
    [~,before] = tinympc14_vlwip_sagittal(p,x-probe*xd,u);
    [~,after] = tinympc14_vlwip_sagittal(p,x+probe*xd,u);
    momentum_rate = (after.rolling_momentum-before.rolling_momentum)/(2*probe);
    energy_rate = (after.energy-before.energy)/(2*probe);
    assert(abs(momentum_rate-torque/p.R_w)<1e-6, 'Rolling momentum balance failed.');
    assert(abs(energy_rate-instant.input_power)<1e-6, 'Mechanical power balance failed.');
end
fprintf('VL-WIP symmetric rolling equilibrium, momentum and power: PASS\n');
end
