function [report,table] = build_mixmpc_online_model(output_dir,p)
% Build the calibrated online-model candidate as standalone C, row-major.
old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
root = fileparts(fileparts(mfilename('fullpath')));
addpath(root);
mixmpc_paths();
if nargin < 2, p = tinympc14_config(); end
[report,table] = verify_mixmpc_terminal_grid(p);
c = coder.config('lib');
c.TargetLang = 'C';
c.RowMajor = true;
c.InstructionSetExtensions = 'None';
c.EnableDynamicMemoryAllocation = false;
if isfield(p,'URDF_SOURCE'), p = rmfield(p,'URDF_SOURCE'); end
codegen('-config',c,'tinympc14_online_model','-args', ...
    {coder.Constant(p),coder.Constant(table),0,0,0},'-d',char(output_dir),'-c');
copyfile(fullfile(matlabroot,'extern','include','tmwtypes.h'),output_dir);
end
