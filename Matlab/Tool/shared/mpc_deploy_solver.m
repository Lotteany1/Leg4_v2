function mpc_deploy_solver(D)
% Deploy the solver beside its generated model data.
if strcmp(D.SOLVER_NAME, 'balance_tinympc')
    header = fileread(fullfile(D.SOLVER_DIR, 'balance_tinympc.h'));
    impl = fileread(fullfile(D.SOLVER_DIR, 'balance_tinympc.c'));
    assert(contains(header, 'Tinympc_Async_Ready') && contains(impl, 'tinympc_seq_execution_epoch') && contains(impl, 'Tinympc_Pitch_Observe'), ...
        'Balance solver source is stale: Ready API and epoch guard required');
elseif strcmp(D.SOLVER_NAME, 'leglen_tinympc')
    header = fileread(fullfile(D.SOLVER_DIR, 'leglen_tinympc.h'));
    assert(contains(header, 'Leglen_Tinympc_Get_Force_Anchor') && contains(header, 'Leglen_Tinympc_Set_Force_Anchor'), ...
        'Leg-length solver source is stale: total-force anchor APIs required');
end
if ~isfolder(D.OUT_DIR), mkdir(D.OUT_DIR); end
for ext = {'.c', '.h'}
    name = [D.SOLVER_NAME ext{1}];
    source = fullfile(D.SOLVER_DIR, name);
    target = fullfile(D.OUT_DIR, name);
    assert(isfile(source), 'Missing MPC solver: %s', source);
    if ~strcmpi(char(java.io.File(source).getCanonicalPath()), ...
            char(java.io.File(target).getCanonicalPath()))
        [ok, message] = copyfile(source, target, 'f');
        assert(ok, 'Cannot deploy %s: %s', name, message);
    end
end
fprintf('已部署求解器: %s.c/.h -> %s\n', D.SOLVER_NAME, D.OUT_DIR);
end
