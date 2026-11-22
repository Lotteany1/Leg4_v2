function packed = tinympc14_terminal_pack(table)
% Lossless random access to int16 coordinates relative to the zero-lean plane.
% C export writes anchor.' and bit_offsets.' in MATLAB linear order so that
% the channels of one node and the bit prefixes of one plane are contiguous.
nLeg = numel(table.length_grid);
nNode = nLeg*nLeg;
angles = abs(table.angle_axis(:));
nPlane = numel(angles);
rank = size(table.coords,2);
assert(nNode>0 && nPlane>=2 && rank>0 && isa(table.coords,'int16') && ...
    ismatrix(table.coords) && size(table.coords,1)==nNode*nPlane, ...
    'Invalid scheduled int16 coordinate table');
[nearest,anchorPlane] = min(angles);
assert(all(isfinite(angles)) && nnz(angles==nearest)==1, ...
    'Terminal grid requires a unique plane closest to zero lean');

node = 1:nNode;
anchor = table.coords((anchorPlane-1)*nNode+node,:);
planes = [1:anchorPlane-1,anchorPlane+1:nPlane];
planeOffsets = zeros(1,numel(planes),'uint32');
bitOffsets = zeros(numel(planes),rank+1,'uint16');
chunks = cell(1,numel(planes));
totalBytes = 0;
for group = 1:numel(planes)
    plane = planes(group);
    delta = int32(table.coords((plane-1)*nNode+node,:))-int32(anchor);
    codes = uint32(bitshift(abs(delta),1)-int32(delta<0));
    widths = ceil(log2(double(max(codes,[],1))+1));
    prefix = [0,cumsum(widths)];
    assert(prefix(end)<=double(intmax('uint16')),'Terminal bit offsets exceed uint16');
    bitOffsets(group,:) = uint16(prefix);
    stride = ceil(prefix(end)/8);
    planeOffsets(group) = uint32(totalBytes);
    totalBytes = totalBytes+nNode*stride;
    assert(totalBytes<=double(intmax('uint32')),'Terminal payload exceeds uint32');

    % Each column of bytes holds one complete, byte-aligned node. The low
    % bit of a channel's ZigZag code is at prefix(channel) in that record.
    bytes = zeros(stride,nNode,'uint8');
    for channel = 1:rank
        if widths(channel)==0
            continue;
        end
        shift = mod(prefix(channel),8);
        value = bitshift(codes(:,channel).',shift);
        first = floor(prefix(channel)/8)+1;
        last = ceil(prefix(channel+1)/8);
        for byte = first:last
            bytes(byte,:) = bitor(bytes(byte,:),uint8(bitand(value,255)));
            value = bitshift(value,-8);
        end
    end
    chunks{group} = reshape(bytes,1,[]);
end
packed = struct('anchor',anchor,'plane_offsets',planeOffsets, ...
    'bit_offsets',bitOffsets,'payload',[chunks{:}]);
end
