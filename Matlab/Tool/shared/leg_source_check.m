function leg_source_check()
% 构建入口核对 CAD 来源，补偿查表由模型与部署产物对拍验证。
root=fileparts(fileparts(mfilename('fullpath'))); repo=fileparts(fileparts(root));
header=fileread(fullfile(repo,'Wheel_Leg_Code','Code','Config','physical_config.h'));
source=fullfile(repo,'001','001','urdf','001.urdf');
fid=fopen(source,'rb'); assert(fid>=0,'Missing 001 URDF');
cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
digest=java.security.MessageDigest.getInstance('SHA-256');
digest.update(fread(fid,Inf,'*uint8'));
actual=lower(reshape(dec2hex(typecast(digest.digest(),'uint8'),2)',1,[]));
assert(contains(header,actual),'001 CAD data stale: run calibrate_001_urdf.');
[~,p]=leg_urdf_data();
keys={'m_total','m_body','m_leg','m_wheel','M_SUP','half_track','wheel_radius','l_c','body_x','Len_min','Len_max','I_w','I_b','I_z','I_z_body'};
for k=1:numel(keys)
    token=regexp(header,['#define\s+CAD_' upper(keys{k}) '\s+([0-9.eE+-]+)f'],'tokens','once');
    assert(~isempty(token) && abs(str2double(token{1})-p.(keys{k}))<1e-10, ...
        'physical_config.h differs from MATLAB: %s.',keys{k});
end
end
