function h=tinympc14_sha256(path)
fid=fopen(path,'rb'); assert(fid>=0,'Cannot read %s',path);
c=onCleanup(@() fclose(fid)); b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256'); md.update(b);
d=typecast(md.digest(),'uint8'); h=lower(reshape(dec2hex(d,2)',1,[]));
end
