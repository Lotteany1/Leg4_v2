function synced = tinympc14_deployment_check(output_dir, report_sync)
% Check generated weights and make the firmware/package distinction explicit.
if nargin<2, report_sync=true; end
p = tinympc14_config();
source = fileread(fullfile(output_dir,'tinympc14_data.c'));
for item = {'q','r'}
    name = item{1};
    token = regexp(source,['tinympc14_' name '_diag\[\d+\]\s*=\s*\{([^}]+)\}'],'tokens','once');
    assert(~isempty(token),'Missing generated %s weights',name);
    values = sscanf(strrep(strrep(token{1},'f',''),',',' '),'%f')';
    expected = p.([upper(name) '_DIAG']);
    assert(numel(values)==numel(expected) && all(abs(values-expected(:)')<=1e-7*max(1,abs(expected(:)'))), ...
        'Generated %s weights differ from current config',name);
end
root = fileparts(fileparts(mfilename('fullpath')));
firmware = fullfile(fileparts(fileparts(root)),'Wheel_Leg_Code','Code','Model');
synced = true;
for item = {'tinympc14.c','tinympc14.h','tinympc14_data.c','tinympc14_data.h','tinympc14_schedule.c','tinympc14_cost.c'}
    target = fullfile(firmware,item{1});
    if ~isfile(target) || ~strcmp(tinympc14_sha256(target),tinympc14_sha256(fullfile(output_dir,item{1})))
        synced = false;
    end
end
if ~report_sync, return; end
fprintf('Generated Q = %s\nGenerated R = %s\n',mat2str(p.Q_DIAG),mat2str(p.R_DIAG));
if synced
    fprintf('Firmware source synchronized: %s. Rebuild firmware before flashing.\n',firmware);
else
    warning('tinympc14:FirmwareNotSynchronized', ...
        ['Package generated, but firmware Code/Model still differs. Recompiling firmware uses its existing parameters.\n' ...
         'To explicitly publish the verified package, run build_mixmpc_pre_fast(''%s''), then rebuild firmware.\n' ...
         'Package: %s'],firmware,output_dir);
end
end
