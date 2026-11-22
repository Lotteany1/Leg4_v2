function folder = tinympc14_export_split(package_dir)
% Export a multi-unit embedded package. The original four-file API is retained.
% No MATLAB-generated arithmetic is rewritten: only constant definitions move.
folder=fullfile(package_dir,'embedded');
if ~isfolder(folder), mkdir(folder); end
for item={'tinympc14.c','tinympc14.h','tinympc14_data.h'}
    content=fileread(fullfile(package_dir,item{1}));
    content=regexprep(content,'(?m)^#define TINYMPC14_LPV_CACHE_SHA256[^\r\n]*','');
    write_changed(fullfile(folder,item{1}),content);
end
source=fileread(fullfile(package_dir,'tinympc14_data.c'));
pattern='/\* >>> lpv >>> \*/(.*?)/\* <<< lpv <<< \*/';
token=regexp(source,pattern,'tokens','once');
assert(~isempty(token),'Split export requires a current polynomial scheduler package');
scheduler=token{1};
if contains(fileread(fullfile(package_dir,'tinympc14_data.h')),'#define TINYMPC14_LPV_RUNTIME_DATA 1')
    cost=regexp(source,'/\* >>> lpv_cost >>> \*/(.*?)/\* <<< lpv_cost <<< \*/','tokens','once');
    assert(~isempty(cost),'Missing runtime terminal coefficients');
    source=regexprep(source,'/\* >>> lpv >>> \*/.*?/\* <<< lpv <<< \*/','');
    source=regexprep(source,'/\* >>> lpv_cost >>> \*/.*?/\* <<< lpv_cost <<< \*/','');
    source=regexprep(source,'(?m)^//[^\r\n]*生成于[^\r\n]*','');
    scheduler=regexprep(scheduler,'(?m)^ \* C/C\+\+ source code generated on[^\r\n]*','');
    write_changed(fullfile(folder,'tinympc14_data.c'),source);
    write_changed(fullfile(folder,'tinympc14_schedule.c'),['#include "tinympc14_data.h"' newline scheduler]);
    write_changed(fullfile(folder,'tinympc14_cost.c'),['#include "tinympc14_data.h"' newline cost{1}]);
    write_manifest(folder);
    return;
end
% Coder places these terminal/Q/R arrays in the scheduler entry function.
pattern='static const (double|float|short) ((?:t0_\w+|b_dv)\[\d+\])\s*=\s*\{[^}]*\};';
[first,last,tokens]=regexp(scheduler,pattern,'start','end','tokens');
assert(numel(first)>=6,'Generated table layout changed; update split exporter');
data=cell(1,numel(first)); parts=cell(1,2*numel(first)+1); cursor=1;
for k=1:numel(first)
    declaration=scheduler(first(k):last(k));
    declaration=regexprep(declaration,'^static const','const');
    data{k}=declaration;
    parts{2*k-1}=scheduler(cursor:first(k)-1);
    parts{2*k}=sprintf('extern const %s %s;',tokens{k}{1},tokens{k}{2});
    cursor=last(k)+1;
end
parts{end}=scheduler(cursor:end); scheduler=[parts{:}];
% C symbols need external linkage and unique names across compilation units.
for k=1:numel(tokens)
    name=regexp(tokens{k}{2},'^\w+','match','once');
    scheduler=regexprep(scheduler,['\<' name '\>'],['tinympc14_cost_' name]);
    for j=1:numel(data)
        data{j}=regexprep(data{j},['\<' name '\>'],['tinympc14_cost_' name]);
    end
end
source=regexprep(source,'/\* >>> lpv >>> \*/.*?/\* <<< lpv <<< \*/','');
% Generation timestamps must not invalidate otherwise unchanged compiler units.
source=regexprep(source,'(?m)^//[^\r\n]*生成于[^\r\n]*','');
scheduler=regexprep(scheduler,'(?m)^ \* C/C\+\+ source code generated on[^\r\n]*','');
write_changed(fullfile(folder,'tinympc14_data.c'),source);
write_changed(fullfile(folder,'tinympc14_schedule.c'),['#include "tinympc14_data.h"' newline scheduler]);
cost_source=strjoin(data,newline);
% Coder emits the int16 maximum symbol in saturated table entries.
% The separate data unit has no dependency on Coder's private type headers.
cost_source=regexprep(cost_source,'\<MAX_int16_T\>','32767');
write_changed(fullfile(folder,'tinympc14_cost.c'),cost_source);
write_manifest(folder);
end

function write_manifest(folder)
manifest='';
for item={'tinympc14.c','tinympc14.h','tinympc14_data.c','tinympc14_data.h','tinympc14_schedule.c','tinympc14_cost.c'}
    manifest=[manifest sprintf('%s *%s\n',tinympc14_sha256(fullfile(folder,item{1})),item{1})]; %#ok<AGROW>
end
write_changed(fullfile(folder,'tinympc14_deployment.sha256'),manifest);
fprintf('Embedded split package: %s (compile all four .c files)\n',folder);
end

function write_changed(file,source)
if isfile(file) && strcmp(fileread(file),source), return; end
temporary=[tempname(fileparts(file)) '.tmp'];
cleanup=onCleanup(@() remove_temporary(temporary)); %#ok<NASGU>
fid=fopen(temporary,'wb'); assert(fid>=0,'Cannot write %s',temporary);
close_file=onCleanup(@() fclose(fid));
bytes=unicode2native(source,'UTF-8');
assert(fwrite(fid,bytes,'uint8')==numel(bytes),'Incomplete split output: %s',file);
clear close_file
tinympc14_replace_generated(temporary,file);
end

function remove_temporary(file)
if isfile(file), delete(file); end
end
