function v = tinympc14_table_col(T, l, col)
% 表 T 第 col 列（1-based，含第一列 l）在 l 处的插值
r = tinympc14_table_row(T, l);
v = r(col - 1);
end
