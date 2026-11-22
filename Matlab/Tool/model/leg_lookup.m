function row = leg_lookup(table,length)
%#codegen
% 固定大小物理参数表的分段线性插值，表外保持端点。
coder.inline('always');
T = cast(table,'like',length);
value = min(max(length,T(1,1)),T(end,1));
index = 1;
while index<size(T,1)-1 && value>=T(index+1,1)
    index=index+1;
end
weight=(value-T(index,1))/(T(index+1,1)-T(index,1));
row=T(index,2:end)+weight*(T(index+1,2:end)-T(index,2:end));
end
