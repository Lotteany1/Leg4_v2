function tinympc14_section_check()
% Regression for large C sections, independent section markers and UTF-8 round trips.
folder=tempname; mkdir(folder);
file=fullfile(folder,'data.c'); header=fullfile(folder,'data.h');
body=sprintf('#ifndef SAMPLE\n#define SAMPLE\n#endif\n/* 生成数据与中文注释 */\n');
body=[body repmat(sprintf('/* generated payload */\n'),1,90000)];
for k=1:3
    tinympc14_section(file,'lpv',body);
    tinympc14_section(file,'zoh',sprintf('/* 离散外扰 %d */',k));
    text=fileread(file);
    assert(contains(text,'生成数据与中文注释') && contains(text,sprintf('离散外扰 %d',k)),'Section UTF-8 content lost');
    assert(numel(strfind(text,'/* >>> lpv >>> */'))==1 && numel(strfind(text,'/* >>> zoh >>> */'))==1,'Duplicate sections');
    assert(strfind(text,'/* <<< lpv <<< */')<strfind(text,'/* >>> zoh >>> */'),'C sections were nested inside an embedded header guard');
end
fid=fopen(header,'w'); fprintf(fid,'#ifndef DATA_H\n#define DATA_H\n#endif\n'); fclose(fid);
tinympc14_section(header,'declarations','extern int data;');
text=fileread(header); assert(strfind(text,'extern int data;')<strfind(text,'#endif'),'Header declaration escaped guard');
assert(isempty(dir(fullfile(folder,'*.tmp'))),'Temporary file leaked');
fprintf('SECTION IO PASS: large repeated sections, UTF-8, C/header placement; artifacts %s\n',folder);
end
