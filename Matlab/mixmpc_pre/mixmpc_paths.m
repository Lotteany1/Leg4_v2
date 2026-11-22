function D = mixmpc_paths()
% Active mixed-MPC directories and shared Matlab/Tool dependency.
D.ROOT = fileparts(mfilename('fullpath'));
D.CONFIG_DIR = D.ROOT;
D.FUNCTIONS_DIR = fullfile(D.ROOT, 'functions');
D.DEPLOY_DIR = fullfile(D.ROOT, 'deploy');
D.VERIFY_DIR = fullfile(D.ROOT, 'verify');
D.BUILD_DIR = fullfile(D.ROOT, 'build');
D.STAGING_DIR = fullfile(D.BUILD_DIR, 'staging');
D.HOST_BUILD_DIR = fullfile(D.BUILD_DIR, 'host');
D.SOLVER_DIR = fullfile(D.DEPLOY_DIR, 'solver');
D.TOOL_ROOT = fullfile(fileparts(D.ROOT), 'Tool');
assert(isfolder(fullfile(D.TOOL_ROOT, 'model')), 'Missing shared Matlab/Tool: %s', D.TOOL_ROOT);
folders = {D.CONFIG_DIR, D.FUNCTIONS_DIR, D.DEPLOY_DIR, D.VERIFY_DIR, ...
    D.TOOL_ROOT, fullfile(D.TOOL_ROOT, 'model'), fullfile(D.TOOL_ROOT, 'shared'), ...
    fullfile(D.TOOL_ROOT, 'solver')};
folders = folders(cellfun(@isfolder,folders));
% Preserve the old loop's precedence, but change the global MATLAB search
% path once and only when needed. Repeated addpath calls invalidate caches.
desired = fliplr(folders);
current = strsplit(path,pathsep);
normalized = strrep(current,'/','\');
target = strrep(desired,'/','\');
if numel(current)>=numel(desired) && all(strcmpi(normalized(1:numel(desired)),target))
    return;
end
owned = false(size(current));
for k=1:numel(target),owned=owned | strcmpi(normalized,target{k});end
path(strjoin([desired,current(~owned)],pathsep));
end
