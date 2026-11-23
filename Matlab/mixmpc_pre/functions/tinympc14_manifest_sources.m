function files = tinympc14_manifest_sources()
% Sources shared by all generated table manifests; excludes build/history.
root = fileparts(fileparts(mfilename('fullpath')));
files = {'mixmpc_paths.m', 'tinympc14_config.m', 'deploy/tinympc14_terminal_decode.h'};
for folder = {'functions', 'deploy', 'verify'}
    entries = dir(fullfile(root, folder{1}, '*.m'));
    for k = 1:numel(entries)
        files{end+1} = [folder{1} '/' entries(k).name]; %#ok<AGROW>
    end
end
for ext = {'*.c', '*.h'}
    entries = dir(fullfile(root, 'deploy', 'solver', ext{1}));
    for k = 1:numel(entries)
        files{end+1} = ['deploy/solver/' entries(k).name]; %#ok<AGROW>
    end
end
for folder = {'deploy/rotation','verify/host'}
    entries=dir(fullfile(root,folder{1},'*'));
    for k=1:numel(entries)
        [~,~,extension]=fileparts(entries(k).name);
        if ~entries(k).isdir && any(strcmp(extension,{'.c','.h','.in','.cmd'}))
            files{end+1}=[folder{1} '/' entries(k).name]; %#ok<AGROW>
        end
    end
end
names = {'Leg_Pos', 'Leg_Spd', 'Leg_Trq', 'Leg_InvTrq', 'Leg_Comp', ...
    'Leg_SpringF', 'Leg_RollComp', 'Leg_CoriolisComp'};
for k = 1:numel(names)
    files{end+1} = ['../Tool/model/' names{k} '.m']; %#ok<AGROW>
end
helpers = dir(fullfile(fileparts(root), 'Tool', 'shared', 'qr_build_*.m'));
for k = 1:numel(helpers)
    files{end+1} = ['../Tool/shared/' helpers(k).name]; %#ok<AGROW>
end
files = sort([files, {'../Tool/leg_config.m', '../Tool/shared/leg_paths.m', '../Tool/shared/mpc_output_path.m'}]);
end
