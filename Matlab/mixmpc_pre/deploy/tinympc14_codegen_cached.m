function [body,types,cost,table_type,report] = tinympc14_codegen_cached(p,table)
% Cache real Coder arithmetic using model/toolchain/shape provenance.
% Every build emits fresh Q/R-dependent coefficients. No constant specialization
% on terminal or weight values is allowed in the cached arithmetic.
root=fileparts(fileparts(mfilename('fullpath')));
% Keep Q/R-dependent lengths and values out of the arithmetic cache key.
packed=tinympc14_terminal_pack(table);
[~,anchor_plane]=min(abs(table.angle_axis));
rank=size(table.coords,2);
plane_nodes=numel(table.length_grid)^2;
table=rmfield(table,'coords');
kernel=p; kernel.Q_DIAG=zeros(size(p.Q_DIAG)); kernel.R_DIAG=zeros(size(p.R_DIAG));
kernel.OUT_DIR=''; kernel.STAGING_DIR='';
names=fieldnames(table); shape=struct();
for k=1:numel(names)
    value=table.(names{k});
    shape.(names{k})=struct('class',class(value),'size',size(value));
end
inputs=tinympc14_manifest_sources(); sources=cell(size(inputs)); input_hashes=cell(size(inputs));
for k=1:numel(inputs)
    source=fileread(fullfile(root,inputs{k}));
    input_hashes{k}=tinympc14_sha256(fullfile(root,inputs{k}));
    source=regexprep(source,'(?s)%\s*>>>\s*Q/R-BLOCK-BEGIN\s*>>>.*?%\s*<<<\s*Q/R-BLOCK-END\s*<<<','');
    source=regexprep(source,'(?m)^\s*(?:D|cfg)\.(?:OUT_DIR|FIRMWARE_COMPARE_DIR)\s*=[^\r\n]*','');
    sources{k}=strrep(source,char(13),'');
end
signature=struct('schema',3,'kernel',kernel,'shape',shape,'sources',{sources}, ...
    'matlab',version,'coder',ver('coder'),'computer',computer, ...
    'options','lib C row-major no-ISA no-dynamic-allocation; runtime table and R');
compiler=mex.getCompilerConfigurations('C','Selected');
assert(numel(compiler)==1,'Select one C compiler before generation');
compiler_files=dir(fullfile(compiler.Location,'VC','Tools','MSVC','*','bin','Hostx64','x64','cl.exe'));
compiler_hashes=cell(size(compiler_files));
for k=1:numel(compiler_files)
    compiler_hashes{k}=tinympc14_sha256(fullfile(compiler_files(k).folder,compiler_files(k).name));
end
signature.toolchain=struct('name',compiler.Name,'version',compiler.Version,'location',compiler.Location, ...
    'details',compiler.Details,'binary_sha256',{compiler_hashes});
signature.types_header_sha256=tinympc14_sha256(fullfile(matlabroot,'extern','include','tmwtypes.h'));
key=hash_text(jsonencode(signature));
parent=fullfile(root,'build','lpv_codegen');
if ~isfolder(parent),mkdir(parent);end
metadata=fullfile(parent,[key '.mat']);
hit=false;
if isfile(metadata) && isfile([metadata '.sha256']) ...
        && strcmp(strtrim(fileread([metadata '.sha256'])),tinympc14_sha256(metadata))
    saved=load(metadata,'files','hashes','saved_key','folder');
    folder=char(java.io.File(saved.folder).getCanonicalPath());
    base=char(java.io.File(parent).getCanonicalPath());
    hit=strcmp(saved.saved_key,key) && startsWith(lower(folder),[lower(base) filesep]);
    for k=1:numel(saved.files)
        file=fullfile(folder,saved.files{k});
        hit=hit && isfile(file) && strcmp(tinympc14_sha256(file),saved.hashes{k});
    end
end
started=tic;
if ~hit
    generated=tempname(parent); mkdir(generated);
    c=coder.config('lib');c.TargetLang='C';c.RowMajor=true;
    c.InstructionSetExtensions='None';c.EnableDynamicMemoryAllocation=false;
    codegen('-config',c,'tinympc14_lpv_kernel','-args', ...
        {coder.Constant(kernel),table,zeros(size(p.R_DIAG)),0,0,0,0},'-d',generated,'-c');
    copyfile(fullfile(matlabroot,'extern','include','tmwtypes.h'),generated);
    for k=1:numel(inputs)
        assert(strcmp(input_hashes{k},tinympc14_sha256(fullfile(root,inputs{k}))), ...
            'LPV source changed during generation: %s',inputs{k});
    end
    folder=generated;
    entries=[dir(fullfile(folder,'*.c'));dir(fullfile(folder,'*.h'))];
    files={entries.name};hashes=cellfun(@(x)tinympc14_sha256(fullfile(folder,x)),files,'UniformOutput',false);
    saved_key=key; %#ok<NASGU>
    save(fullfile(folder,'manifest.mat'),'files','hashes','saved_key','signature','folder','-v7');
    if isfile(metadata),copyfile(metadata,[metadata '.rejected_' char(java.util.UUID.randomUUID())]);end
    temporary=[tempname(parent) '.mat'];
    save(temporary,'files','hashes','saved_key','signature','folder','-v7');movefile(temporary,metadata,'f');
    temporary=[tempname(parent) '.sha256'];fid=fopen(temporary,'w');assert(fid>=0);
    fprintf(fid,'%s\n',tinympc14_sha256(metadata));fclose(fid);movefile(temporary,[metadata '.sha256'],'f');
end
report=struct('hit',hit,'seconds',toc(started),'key',key,'folder',folder);
fprintf('ROTATION_CODEGEN hit=%d seconds=%.9f key=%s\n',hit,report.seconds,key);
included={}; types=expand('tinympc14_lpv_kernel_types.h');
included={}; entries=dir(fullfile(folder,'*.c')); pieces=cell(size(entries));
for k=1:numel(entries),pieces{k}=expand(entries(k).name);end
body=strjoin(pieces,newline);
assert(contains(body,'tinympc14_terminal_decode_node('), ...
    'Generated scheduler does not call the terminal coordinate decoder');
decoder=fileread(fullfile(root,'deploy','tinympc14_terminal_decode.h'));
body=[decoder newline body];
declaration=fileread(fullfile(folder,'tinympc14_lpv_kernel.h'));
table_type=regexp(declaration,'tinympc14_lpv_kernel\(const (\w+) \*table','tokens','once');
assert(~isempty(table_type),'Unexpected generated table interface');table_type=table_type{1};
% Emit in declaration order, using row-major storage for matrix fields.
type_source=fileread(fullfile(folder,'tinympc14_lpv_kernel_types.h'));
fields=regexp(type_source,['typedef struct\s*\{([^}]+)\}\s*' table_type '\s*;'],'tokens','once');
assert(~isempty(fields),'Cannot identify generated terminal structure');
fields=regexp(fields{1},'\w+\s+(\w+)(?:\[\d+\])?\s*;','tokens');
assert(numel(fields)==numel(names),'Unexpected generated terminal fields: %d/%d',numel(fields),numel(names));
cost=sprintf('const %s tinympc14_terminal = {\n',table_type);
for k=1:numel(fields)
    name=fields{k}{1};value=table.(name);
    assert(ismatrix(value),'Terminal field must be scalar/vector/matrix');
    if ~isvector(value),value=value.';end
    value=value(:);
    if isinteger(value),fmt='%d,';
    elseif isa(value,'single'),fmt='%.9ef,';
    else,fmt='%.17e,';end
    if numel(value)>1
        literal=['{' newline sprintf([repmat(fmt,1,8) '\n'],value) newline '}'];
    else
        literal=sprintf(fmt,value);literal=literal(1:end-1);
    end
    cost=[cost '    ' literal ',' newline]; %#ok<AGROW>
end
cost=[cost '};' newline sprintf('const double tinympc14_terminal_r[%d] = {',p.NU) ...
    sprintf('%.17e,',p.R_DIAG) '};' newline];

% The kernel structure contains only axes, basis and offset. Independent
% symbols allow the encoded byte length to change without recompiling Coder.
types=[types newline sprintf(['#define TINYMPC14_COORD_RANK %dU\n' ...
    '#define TINYMPC14_COORD_PLANE_NODES %dU\n#define TINYMPC14_COORD_ANCHOR_PLANE %dU\n'], ...
    rank,plane_nodes,anchor_plane-1)];
% Four bytes per C literal keeps the generated initializer small for AC5 and
% MSVC. Logical shifts define the byte order independently of the host.
% A zero guard word permits the decoder to preload at the exact end of a node.
bytes=reshape([packed.payload,zeros(1,mod(-numel(packed.payload),4),'uint8')],4,[]);
words=zeros(1,size(bytes,2)+1,'uint32');
for byte=1:4
    words(1:end-1)=bitor(words(1:end-1),bitshift(uint32(bytes(byte,:)),8*(byte-1)));
end
arrays={'anchor','short',reshape(packed.anchor.',[],1); ...
    'plane_offsets','unsigned int',packed.plane_offsets(:); ...
    'bit_offsets','unsigned short',reshape(packed.bit_offsets.',[],1); ...
    'payload','unsigned int',words(:)};
report.encoded_coord_bytes=2*numel(packed.anchor)+4*numel(packed.plane_offsets)+ ...
    2*numel(packed.bit_offsets)+4*numel(words);
for k=1:size(arrays,1)
    values=arrays{k,3};
    decl=sprintf('const %s tinympc14_coord_%s',arrays{k,2},arrays{k,1});
    bound=sprintf('[%d]',numel(values));
    if strcmp(arrays{k,1},'payload')
        % Changing Q/R at the same rank should not change the public header.
        external_bound='[]';
        fmt='%uU,';
    else
        external_bound=bound;
        fmt='%d,';
    end
    types=[types 'extern ' decl external_bound ';' newline]; %#ok<AGROW>
    cost=[cost decl bound ' = {' newline sprintf([repmat(fmt,1,16) '\n'],values) newline '};' newline]; %#ok<AGROW>
end
types=[types '#ifdef TINYMPC14_TERMINAL_CODEC_TEST' newline ...
    'void Tinympc14_Test_Decode_Node(unsigned int node, short *out);' newline '#endif' newline];

    function source=expand(name)
        if any(strcmp(included,name)),source='';return;end
        included{end+1}=name;source=fileread(fullfile(folder,name));
        source=regexprep(source,'(?m)^#\s*pragma once[^\r\n]*','');
        [first,last,tokens]=regexp(source,'(?m)^#include "([a-zA-Z0-9_]+\.h)"[^\r\n]*','start','end','tokens');
        parts=cell(1,2*numel(first)+1);cursor=1;
        for j=1:numel(first)
            parts{2*j-1}=source(cursor:first(j)-1);name2=tokens{j}{1};
            if strcmp(name2,'mwstdint.h'),parts{2*j}=source(first(j):last(j));
            else,parts{2*j}=expand(name2);end
            cursor=last(j)+1;
        end
        parts{end}=source(cursor:end);source=[parts{:}];
    end
end

function digest=hash_text(source)
md=java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(unicode2native(source,'UTF-8')),'int8'));
digest=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2).',1,[]));
end
