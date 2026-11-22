function fed = tinympc14_zoh_apply(p, fe)
% f_extra 四行经 4x14 ZOH 系数表折进 14 态
persistent fz rows0

if isempty(fz)
    here = fileparts(mfilename('fullpath'));
    S = load(fullfile(here, '..', 'build', 'tinympc14_zoh_cache.mat'), 'fz', 'rows0');
    fz = S.fz;
    rows0 = S.rows0;
end

assert(numel(fe) == p.NX, 'fe 长度与 NX 不符');
fed = fz.' * fe(rows0 + 1);
end
