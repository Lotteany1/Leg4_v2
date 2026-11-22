function tinympc14_install_online(package_dir, model_dir)
% Install the verified solver and data pairs into the firmware model directory.
manifest = fileread(fullfile(package_dir,'tinympc14_online.sha256'));
lines = regexp(strtrim(manifest),'\r?\n','split');
sources = cell(size(lines));
names = cell(size(lines));
hashes = cell(size(lines));
for k = 1:numel(lines)
    token = regexp(lines{k},'^([0-9a-f]{64}) \*(solver/tinympc14(?:_data)?\.[ch])$', ...
        'tokens','once');
    assert(~isempty(token),'Invalid package manifest entry.');
    sources{k} = fullfile(package_dir,token{2});
    [~,name,extension] = fileparts(token{2});
    names{k} = [name extension];
    hashes{k} = token{1};
    assert(strcmp(tinympc14_sha256(sources{k}),hashes{k}),'Package changed: %s',token{2});
end
assert(numel(names)==4 && ...
    all(ismember({'tinympc14.c','tinympc14.h','tinympc14_data.c','tinympc14_data.h'},names)), ...
    'Incomplete online package or duplicate files.');
if ~isfolder(model_dir), mkdir(model_dir); end
for k = 1:numel(names)
    destination = fullfile(model_dir,names{k});
    copyfile(sources{k},destination,'f');
    assert(strcmp(tinympc14_sha256(destination),hashes{k}),'Installed file changed: %s',destination);
end
fprintf('Installed %d TinyMPC C/H files in %s\n',numel(names),model_dir);
end
