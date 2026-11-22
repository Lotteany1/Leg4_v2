function Fs = Leg_SpringF(L0,ang)
%#codegen
% 手感反馈试调的轴向补偿表，左右共用，单位m、N，伸腿为正。
% 短腿减小收腿补偿，中长腿增加补偿；170/340 mm采用已试过的75/185 N。
% ang 保留接口兼容，范围外保持端点值。
nodes = cast([
    0.17, 75.0;
    0.18, 90.0;
    0.19, 115.0;
    0.20, 135.0;
    0.21, 136.0;
    0.22, 137.0;
    0.23, 138.0;
    0.24, 140.0;
    0.25, 150.0;
    0.26, 152.0;
    0.27, 154.0;
    0.28, 156.0;
    0.29, 158.0;
    0.30, 164.0;
    0.31, 168.0;
    0.32, 175.0;
    0.33, 180.0;
    0.34, 185.0;
], 'like', L0);
Fs = nodes(end,2);
if L0 <= nodes(1,1)
    Fs = nodes(1,2);
    return;
end
for k = 2:size(nodes,1)
    if L0 <= nodes(k,1)
        ratio = (L0-nodes(k-1,1))/(nodes(k,1)-nodes(k-1,1));
        Fs = nodes(k-1,2)+ratio*(nodes(k,2)-nodes(k-1,2));
        return;
    end
end
end
