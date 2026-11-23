function tinympc14_build_and_verify()
% Rebuild current deployment tables and run model, QP and closed-loop gates.
here=fileparts(mfilename('fullpath')); addpath(here); addpath(fileparts(here)); mixmpc_paths();
root=fileparts(here); if ~isfolder(fullfile(root,'build')), mkdir(fullfile(root,'build')); end
diary(fullfile(root,'build','audit_full_validation.log'));
cleanup=onCleanup(@() diary('off'));
stages={'s03_tinympc14_cache','s04d_tinympc14_lpv_psi','s07_tinympc14_zoh_data', ...
 's07_tinympc14_static_check','s05a_tinympc14_admm_replica','s05b_tinympc14_admm_fused', ...
 's09_tinympc14_fused_verify','s08_tinympc14_runtime_check','s10_tinympc14_nonlinear_check'};
for k=1:numel(stages)
    run_stage(stages{k});
    if k == 3
        clear tinympc14_lpv_eval tinympc14_zoh_apply
        nominal = load(fullfile(root,'build','qr_nominal_model.mat'), 'model');
        lpv = load(fullfile(root,'build','qr_lpv_model.mat'), 'model');
        zoh = load(fullfile(root,'build','qr_zoh_model.mat'), 'model');
        qr_build_cache('write', root, struct('nominal',nominal.model,'lpv',lpv.model,'zoh',zoh.model));
    end
end
tinympc14_scheduler_c_check();
script=fullfile(root,'verify','host','run_host_msvc.cmd');
[status,detail]=system(['"' script '"']);
fprintf('%s',detail);
assert(status==0 && contains(detail,'RESULT PASS'),'Full host C comparison failed');
fprintf('Leg4 current deployment build and MATLAB validation: PASS\n');
end
function run_stage(name)
run(which(name)); % isolate scripts that use clear from the pipeline workspace
end
