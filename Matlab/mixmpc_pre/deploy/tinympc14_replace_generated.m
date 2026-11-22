function tinympc14_replace_generated(temporary,destination)
% 文件已写完且关闭。在同一目录换名，让 Windows 上正在映射旧 C 文件的读者继续使用旧内容。
for attempt=1:20
    [ok,msg]=movefile(temporary,destination,'f');
    if ok, return; end
    if ~ispc, break; end
    pause(0.05);
end
error('tinympc14:PublishFailed','Cannot replace %s: %s',destination,msg);
end
