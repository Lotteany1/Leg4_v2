function payload = qr_build_cache(action, root, payload)
%QR_BUILD_CACHE Immutable Q/R-independent model cache with source/integrity guards.
file = fullfile(root, 'build', 'qr_model_cache.mat');
digest_file = [file '.sha256'];
switch action
    case 'write'
        signature = source_signature(root);
        check_finite(payload);
        if ~isfolder(fileparts(file)), mkdir(fileparts(file)); end
        tmp = [tempname(fileparts(file)) '.mat'];
        save(tmp, 'signature', 'payload', '-v7');
        digest = hash_bytes(read_bytes(tmp));
        movefile(tmp, file, 'f');
        fid = fopen(digest_file, 'w');
        assert(fid >= 0, 'Cannot write model cache digest');
        cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
        fprintf(fid, '%s\n', digest);
    case 'read'
        if ~isfile(file) || ~isfile(digest_file)
            error('qr_build:CacheMissing', '缺少模型缓存，请先运行原 build 完整构建：%s', file);
        end
        if ~strcmp(strtrim(fileread(digest_file)), hash_bytes(read_bytes(file)))
            error('qr_build:CacheCorrupt', '模型缓存损坏，请先完整构建：%s', file);
        end
        try
            cache = load(file, 'signature', 'payload');
            assert(isfield(cache, 'signature') && isfield(cache, 'payload'));
            check_finite(cache.payload);
        catch
            error('qr_build:CacheCorrupt', '模型缓存结构无效，请先完整构建：%s', file);
        end
        current = source_signature(root);
        if ~isequal(cache.signature, current)
            reason = '源码或非 Q/R 设置发生变化';
            if isfield(cache.signature, 'files') && isfield(cache.signature, 'hashes')
                for k = 1:numel(current.files)
                    j = find(strcmp(cache.signature.files, current.files{k}), 1);
                    if isempty(j) || ~strcmp(cache.signature.hashes{j}, current.hashes{k})
                        reason = current.files{k}; break;
                    end
                end
            end
            error('qr_build:CacheStale', '模型缓存失效（%s），fast 仅支持改 Q/R，请先完整构建。', reason);
        end
        payload = cache.payload;
        fprintf('FAST: 命中模型缓存，跳过符号推导/模型计算/离散化。\n');
    otherwise
        error('qr_build:Action', 'Unknown cache action: %s', action);
end
end

function key = source_signature(root)
[parent, project] = fileparts(root);
% 指纹覆盖模块全部现役源码。老写法把权重文件整份排除，连 Ts 改动都不会让缓存失效；
% 现在改由 Q/R-BLOCK 标记精确圈定"只影响权重"的区块，其余内容照常参与。
files = collect_sources(root, '');
files = [files, collect_sources(fullfile(parent, 'Tool'), '../Tool')];
if strcmp(project, 'mixmpc_pre')
    % These entry points select build actions and do not generate the cached model.
    files(ismember(files, {'build_mixmpc_pre_fast.m', '../Tool/build_leg_tool.m', '../Tool/verify_leg_tool.m'})) = [];
    cfg = tinympc14_config();
    if strcmp(cfg.sym7.k_eff_mode, 'fixed')
        % Fixed stiffness uses cfg.K_EFF. The output compensation is tracked by the deployment manifest.
        files(strcmp(files, '../Tool/model/Leg_SpringF.m')) = [];
    end
end
if strcmp(project, 'lqr')
    files = [files, {'build/dynamics_v2.mat', 'build/dynamics_new_coords.mat', 'build/geometry_cache.mat'}];
end
files = sort(files);
hashes = cell(size(files));
for k = 1:numel(files)
    absolute = fullfile(root, files{k});
    if ~isfile(absolute), error('qr_build:CacheStale', '缺少模型输入 %s，请先完整构建。', files{k}); end
    bytes = read_bytes(absolute);
    if endsWith(files{k}, '.m')
        source = native2unicode(bytes', 'UTF-8');
        % Deployment location is not a physical/model parameter.
        source = regexprep(source, '(?m)^\s*(?:D|cfg)\.(?:OUT_DIR|FIRMWARE_COMPARE_DIR)\s*=[^\r\n]*', '');
        % Q/R-only edits stay a fast-path edit: the marked block carries no model input.
        source = regexprep(source, '(?s)%\s*>>>\s*Q/R-BLOCK-BEGIN\s*>>>.*?%\s*<<<\s*Q/R-BLOCK-END\s*<<<', '');
        if strcmp(project, 'leglen_mpc') && strcmp(files{k}, 'leglen_mpc_config.m')
            % 高度倍率的数值只改变 Q；倍率映射及其余配置仍计入缓存指纹。
            source = regexprep(source, ...
                '(?m)^(height_response|height_damping)[ \t]*=[ \t]*[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?[ \t]*;', ...
                '$1 = <weight scale>;');
        end
        source = strrep(source, char(13), '');
        bytes = unicode2native(source, 'UTF-8');
    end
    hashes{k} = hash_bytes(bytes);
end
key = struct('version', 2, 'files', {files}, 'hashes', {hashes});
% Model/discretization caches are generated with this MATLAB/toolbox/compiler
% environment. A matching source tree alone cannot establish that provenance.
environment = struct('matlab',version,'release',version('-release'),'computer',computer);
toolboxes = ver;
environment.toolboxes = struct('name',{toolboxes.Name},'version',{toolboxes.Version});
compiler = mex.getCompilerConfigurations('C','Selected');
environment.compiler = struct('name',{},'version',{},'location',{});
for k = 1:numel(compiler)
    environment.compiler(k) = struct('name',compiler(k).Name, ...
        'version',compiler(k).Version,'location',compiler(k).Location);
end
key.environment = environment;
if strcmp(project, 'lqr')
    fitting = getenv('LQR_ENABLE_FITTING');
    if isempty(fitting), fitting = '1'; end
    key.fitting = fitting;
end
end

function files = collect_sources(root, prefix)
% 递归收集现役源码。build/ 是生成物，archive/ 与 legacy/ 是历史，
% 三者都不参与缓存失效判断——否则跑一次完整构建就会把 fast 通道自己打失效。
files = {};
folders = {root};
while ~isempty(folders)
    current = folders{1}; folders(1) = [];
    entries = dir(current);
    for k = 1:numel(entries)
        name = entries(k).name;
        if entries(k).isdir
            if ismember(name, {'.', '..', 'build', 'archive'}), continue; end
            folders{end+1} = fullfile(entries(k).folder, name); %#ok<AGROW>
            continue;
        end
        [~, ~, ext] = fileparts(name);
        if ~ismember(ext, {'.m', '.c', '.h', '.cmd'}), continue; end
        relative = strrep(fullfile(entries(k).folder, name), '\', '/');
        relative = relative(numel(root)+2:end);
        if ~isempty(prefix), relative = [prefix '/' relative]; end
        files{end+1} = relative; %#ok<AGROW>
    end
end
end

function bytes = read_bytes(file)
fid = fopen(file, 'rb');
assert(fid >= 0, 'Cannot read %s', file);
cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
bytes = fread(fid, Inf, '*uint8');
end

function digest = hash_bytes(bytes)
md = java.security.MessageDigest.getInstance('SHA-256');
md.update(typecast(uint8(bytes(:)), 'int8'));
digest = lower(reshape(dec2hex(typecast(md.digest(), 'uint8'), 2).', 1, []));
end

function check_finite(value)
if isnumeric(value)
    assert(all(isfinite(value(:))), 'Nonfinite model cache');
elseif isstruct(value)
    fields = fieldnames(value);
    for j = 1:numel(value)
        for k = 1:numel(fields), check_finite(value(j).(fields{k})); end
    end
elseif iscell(value)
    for k = 1:numel(value), check_finite(value{k}); end
end
end
