function tinympc14_section(file, tag, body)
% 在合并数据文件里替换或插入带标记的段，段内容由某个生成器给出
% 标记形如 /* >>> tag >>> */ ... /* <<< tag <<< */：
%   已有该标记就整段替换（重复运行不会叠加），否则插入到 #endif 之前，源文件插到末尾
% body 的换行按目标文件原有风格落盘，避免混入 LF 与 CRLF 两种行尾
assert(~isempty(tag) && isempty(strfind(tag, '/')), 'bad section tag'); %#ok<STREMP>
body = strrep(strrep(body, char([13 10]), char(10)), char(13), char(10));
body = regexprep(body, '\n+$', '');

nl = char(10);
if isfile(file)
    raw = fileread(file);
    if ~isempty(strfind(raw, char([13 10]))) %#ok<STREMP>
        nl = char([13 10]);
    end
    txt = strrep(strrep(raw, char([13 10]), char(10)), char(13), char(10));
else
    txt = '';
end

mark0 = ['/* >>> ' tag ' >>> */'];
mark1 = ['/* <<< ' tag ' <<< */'];
blk = [mark0 char(10) body char(10) mark1 char(10)];

pat = [regexptranslate('escape', mark0) '.*?' regexptranslate('escape', mark1) '\n'];
hit = regexp(txt, pat, 'match', 'once');
if ~isempty(hit)
    txt = strrep(txt, hit, blk);
else
    [~,~,extension] = fileparts(file);
    pos = [];
    if strcmpi(extension,'.h'), pos = strfind(txt, ['#endif' char(10)]); end
    if isempty(pos)
        txt = [txt blk];
    else
        i = pos(end);
        txt = [txt(1:i-1) blk txt(i:end)];
    end
end

if ~isequal(nl, char(10))
    txt = strrep(txt, char(10), nl);
end
% Replace a completed UTF-8 file; in-place truncation races Windows readers of generated C.
temporary = [tempname(fileparts(file)) '.tmp'];
remove_temporary = onCleanup(@() remove_file(temporary)); %#ok<NASGU>
[fid, message] = fopen(temporary, 'wb');
assert(fid >= 0, 'Cannot write %s: %s', temporary, message);
close_file = onCleanup(@() fclose(fid));
bytes = unicode2native(txt, 'UTF-8');
count = fwrite(fid, bytes, 'uint8');
assert(count == numel(bytes), 'Incomplete generated section: %s', file);
clear close_file
% Observed Permission denied after LPV -> ZOH writes; bound the wait for external readers.
for attempt = 1:20
    [ok, message] = movefile(temporary, file, 'f');
    if ok, break; end
    if ~ispc, break; end
    pause(0.05);
end
assert(ok, 'Cannot replace generated file %s: %s', file, message);
end

function remove_file(file)
if isfile(file), delete(file); end
end
