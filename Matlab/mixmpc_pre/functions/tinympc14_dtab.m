function d = tinympc14_dtab(T, l, col)
% 表 T 第 col 列对 l 的中心差分（端点退化为单侧）
h = T(2,1) - T(1,1);
if l <= T(1,1) + h/2
    d = (T(2,col) - T(1,col))/h;
elseif l >= T(end,1) - h/2
    d = (T(end,col) - T(end-1,col))/h;
else
    d = (tinympc14_table_col(T, l + h/2, col) - tinympc14_table_col(T, l - h/2, col))/h;
end
end
