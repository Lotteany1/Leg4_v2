function tinympc14_lpv_generate(fast_mode, cache)
% Exact embedded dynamics plus a positive-definite compressed terminal table.
% Full builds cache dynamics; fast builds reuse them and recompute all Q/R costs.
if nargin<1, fast_mode=false; end
root=fileparts(fileparts(mfilename('fullpath')));
addpath(root); mixmpc_paths(); p=tinympc14_config();
qr_build_check(zeros(p.NX),zeros(p.NX,p.NU),diag(p.Q_DIAG),diag(p.R_DIAG));
if fast_mode
    if nargin<2, cache=qr_build_cache('read',root); end
    [table,model,report]=tinympc14_terminal_build(p,cache.lpv);
else
    [table,model,report]=tinympc14_terminal_build(p);
    temporary=[tempname(fullfile(root,'build')) '.mat'];
    save(temporary,'model','-v7');
    movefile(temporary,fullfile(root,'build','qr_lpv_model.mat'),'f');
end
cache_file=fullfile(root,'build','tinympc14_lpv_psi_cache.mat');
weights=struct('Q',p.Q_DIAG,'R',p.R_DIAG,'rho',p.rho);
save(cache_file,'table','report','weights','-v7');

% Generate the same arithmetic exercised by MATLAB, with no dynamic allocation.
[body,types,cost,table_type,storage]=tinympc14_codegen_cached(p,table);
body=[body sprintf(['\nvoid Tinympc14_Lpv_Data_Update(float left, float right, float psi, tinympc14_model_t *mdl)\n' ...
    '{\n    /* One scheduler task owns this scratch; no large control-task stack frame. */\n' ...
    '    static double ad[196], bd[84], fd[14], k[84], p[196], qu[36], at[196];\n' ...
    '    tinympc14_lpv_kernel(&tinympc14_terminal, tinympc14_terminal_r, (double)left, (double)right, (double)psi, ad, bd, fd, k, p, qu, at);\n' ...
    '    for (int i=0;i<196;i++) { mdl->Ad[i]=(float)ad[i]; mdl->Pinf[i]=(float)p[i]; mdl->AmBKt[i]=(float)at[i]; }\n' ...
    '    for (int i=0;i<84;i++) { mdl->Bd[i]=(float)bd[i]; mdl->Kinf[i]=(float)k[i]; }\n' ...
    '    for (int i=0;i<14;i++) mdl->fd[i]=(float)fd[i];\n' ...
    '    for (int i=0;i<36;i++) mdl->QuuInv[i]=(float)qu[i];\n}\n'])];
header=sprintf(['#define TINYMPC14_LPV_IFACE_VER 4\n' ...
    '#define TINYMPC14_CLOSED_CHAIN_AXIAL_INPUT %d\n' ...
    '#define TINYMPC14_LPV_RUNTIME_DATA 1\n' ...
    '#define TINYMPC14_LPV_L_MIN 0.16f\n#define TINYMPC14_LPV_L_MAX 0.32110177111662558f\n' ...
    '#define TINYMPC14_LPV_PSI_MIN -0.30f\n#define TINYMPC14_LPV_PSI_MAX 0.30f\n' ...
    '#define TINYMPC14_LPV_TERMINAL_RANK %d\n' ...
    '#define TINYMPC14_LPV_CACHE_SHA256 "%s"\n' ...
    'void Tinympc14_Lpv_Data_Update(float left, float right, float psi, tinympc14_model_t *mdl);\n'], ...
    p.sym7.sw_fused.F_H_proj,report.rank,tinympc14_sha256(cache_file));
header=[header types sprintf('\nextern const %s tinympc14_terminal;\nextern const double tinympc14_terminal_r[%d];\n',table_type,p.NU)];
tinympc14_section(fullfile(p.STAGING_DIR,'tinympc14_data.h'),'lpv',header);
tinympc14_section(fullfile(p.STAGING_DIR,'tinympc14_data.c'),'lpv',body);
tinympc14_section(fullfile(p.STAGING_DIR,'tinympc14_data.c'),'lpv_cost',cost);
clear tinympc14_lpv_eval
fprintf(['LPV IFACE 4: exact dynamics, cubic log-Cholesky terminal; host raw coordinates %d B, ' ...
    'encoded coordinates incl. metadata %d B, basis and offset %d B (before linker padding)\n'], ...
    2*numel(table.coords),storage.encoded_coord_bytes,4*(numel(table.basis)+numel(table.offset)));
end

function body=bundle(folder)
% Expand generated local headers so the existing four-file deployment remains
% self-contained. Keep MATLAB's licensed type declarations in the result.
files=dir(fullfile(folder,'*.c')); included={}; pieces=cell(1,numel(files));
assert(any(strcmp({files.name},'tinympc14_lpv_embedded.c')),'Missing generated scheduler');
for k=1:numel(files), pieces{k}=expand(files(k).name); end
body=strjoin(pieces,newline);
    function source=expand(name)
        if any(strcmp(included,name)), source=''; return; end
        included{end+1}=name;
        source=fileread(fullfile(folder,name));
        source=regexprep(source,'(?m)^#\s*pragma once[^\r\n]*','');
        [first,last,tokens]=regexp(source,'(?m)^#include "([a-zA-Z0-9_]+\.h)"[^\r\n]*','start','end','tokens');
        parts=cell(1,2*numel(first)+1); cursor=1;
        for j=1:numel(first)
            parts{2*j-1}=source(cursor:first(j)-1);
            name2=tokens{j}{1};
            if strcmp(name2,'mwstdint.h'), parts{2*j}=source(first(j):last(j));
            else, parts{2*j}=expand(name2); end
            cursor=last(j)+1;
        end
        parts{end}=source(cursor:end); source=[parts{:}];
    end
end
