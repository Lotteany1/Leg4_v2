function row = tinympc14_table_row(T, l)
% 分段线性插值 + 端点钳位（与 Leg_Comp.c 同口径）
l = min(max(l, T(1,1)), T(end,1));
k = 1;
while k < size(T,1)-1 && l >= T(k+1,1)
    k = k+1;
end
s = (l - T(k,1)) / (T(k+1,1) - T(k,1));
row = T(k,2:end) + s*(T(k+1,2:end) - T(k,2:end));
end
