function tinympc14_scheduler_c_check()
% Check every exported matrix, not only the first actuator output of the QP.
root=fileparts(fileparts(mfilename('fullpath')));
p=tinympc14_config(); stream=RandStream('mt19937ar','Seed',29);
samples=[p.LEG_RANGE(1)+diff(p.LEG_RANGE)*rand(stream,200,2),-.3+.6*rand(stream,200,1)];
[a,b,c]=ndgrid([p.LEG_RANGE(1),p.LEG_RANGE(1)+.005,.25,p.LEG_RANGE(2)-.005,p.LEG_RANGE(2)],[p.LEG_RANGE(1),p.LEG_RANGE(1)+.005,.25,p.LEG_RANGE(2)-.005,p.LEG_RANGE(2)],[-.3,0,.3]);
samples=[samples;a(:),b(:),c(:)];
% Explicitly cover the narrow transition that defeated the global fit.
for left=[p.LEG_RANGE(1),.25,p.LEG_RANGE(2)]
    for right=[p.LEG_RANGE(1),.25,p.LEG_RANGE(2)]
        [~,~,~,~,center]=tinympc14_scheduled_ab(p,left,right,0);
        angle=center+[-.001,-.0001,0,.0001,.001];
        samples=[samples;repmat([left,right],5,1),angle']; %#ok<AGROW>
    end
end
% Legacy samples retain their calibrated projection. Low-leg cases supply the
% independent roll-inertia height alongside the usual bounded LPV coordinates.
samples(:,4)=NaN;
raw=[.14 .14 0; .13 .13 0; .12 .12 0; p.roll_height(1) p.roll_height(1) 0; ...
    .12 .15 .2; .12 .15 -.2; .27 .27 .3];
h=mean(raw(:,1:2),2).*cos(raw(:,3));
samples=[samples; max(raw(:,1:2),p.LEG_RANGE(1)), .1*ones(size(h)), h];
samples=double(single(samples));
% Each invocation owns its fixtures, executable and MSVC objects.
folder=tempname;
[ok,msg]=mkdir(folder);
assert(ok,'Cannot create scheduler test directory %s: %s',folder,msg);
cleanup=onCleanup(@() rmdir(folder,'s')); %#ok<NASGU>
% Give the compiled decoder the original integer values in C node order.
cached=load(fullfile(root,'build','tinympc14_lpv_psi_cache.mat'),'table');
coords=cached.table.coords;
write_input(fullfile(folder,'scheduler_coords.bin'),size(coords),coords.','int16');
write_input(fullfile(folder,'scheduler_points.bin'),[],single(samples.'),'single');
script=fullfile(root,'verify','host','run_scheduler_msvc.cmd');
[status,detail]=system(sprintf('call "%s" "%s"',script,folder));
fprintf('%s',detail); assert(status==0,'Scheduler host compilation/execution failed (exit %d)',status);
got=read_matrices(fullfile(folder,'scheduler_matrices.bin'),size(samples,1));
assert(isequal(size(got),[806,size(samples,1)]) && all(isfinite(got),'all'));
counts=[196,84,14,84,196,36,196]; worst=0;
for k=1:size(samples,1)
    x=samples(k,:); m=tinympc14_lpv_eval(p,x(1),x(2),x(3),x(4));
    ref=[rm(m.Ad);rm(m.Bd);m.fd;rm(m.Kinf);rm(m.Pinf);rm(m.QuuInv);rm(m.AmBKt)];
    offset=0;
    for count=counts
        ii=offset+(1:count); offset=offset+count;
        tolerance=max(1e-5*abs(ref(ii)),1e-9*max(1,max(abs(ref(ii)))));
        ratio=max(abs(got(ii,k)-ref(ii))./tolerance); worst=max(worst,ratio);
        assert(ratio<=1,'Scheduler MATLAB/C mismatch at sample %d, block ending %d',k,offset);
    end
end
fprintf('PASS scheduler MATLAB/C: %d samples, all seven matrices, max tolerance fraction %.4g\n',size(samples,1),worst);
end

function write_input(file,dimensions,data,precision)
[fid,msg]=fopen(file,'wb','ieee-le');
assert(fid>=0,'Cannot open scheduler input %s for writing: %s',file,msg);
try
    header_count=fwrite(fid,dimensions,'uint32');
    count=fwrite(fid,data,precision);
    assert(header_count==numel(dimensions) && count==numel(data), ...
        'Incomplete scheduler input %s: %s',file,ferror(fid));
catch err
    fclose(fid);
    rethrow(err);
end
assert(fclose(fid)==0,'Cannot finish writing scheduler input %s',file);
end

function got=read_matrices(file,nSamples)
[fid,msg]=fopen(file,'rb','ieee-le');
assert(fid>=0,'Cannot open scheduler output %s for reading: %s',file,msg);
cleanup=onCleanup(@() fclose(fid)); %#ok<NASGU>
[got,count]=fread(fid,[806,nSamples],'single=>double');
assert(count==806*nSamples,'Incomplete scheduler output %s: %s',file,ferror(fid));
extra=fread(fid,1,'*uint8');
assert(isempty(extra) && feof(fid),'Unexpected trailing data or read error in %s',file);
end

function v=rm(M)
v=reshape(M.',[],1);
end
