function verify_leg_tool()
% 001 实际闭环、解析雅可比、虚功、逆映射，以及原始/部署 C 与 MATLAB 对拍。
old_path=path; cleanup=onCleanup(@()path(old_path)); %#ok<NASGU>
root=fileparts(mfilename('fullpath')); addpath(fullfile(root,'shared'));
D=leg_paths(); addpath(D.ROOT,D.MODEL_DIR,D.SOLVER_DIR);
addpath(fullfile(fileparts(root),'mixmpc_pre')); mixmpc_paths();
if ~isfolder(D.VERIFY_BUILD_DIR), mkdir(D.VERIFY_BUILD_DIR); end
addpath(D.VERIFY_BUILD_DIR);
leg_source_check();
clear leg_vmc_raw leg_vmc_cmsis leg_comp_raw leg_comp_cmsis
clear leg_springf_raw leg_springf_cmsis leg_rollcomp_raw leg_rollcomp_cmsis
[table,p]=leg_config();
assert(abs(p.m_body+2*p.m_leg+2*p.m_wheel-p.m_total)<1e-10,'Mass partition mismatch');
assert(all(diff(table(:,1))>0) && all(table(:,4)>=p.m_leg*table(:,3).^2),'Invalid leg inertia table');
repo=fileparts(fileparts(root));
r=mixmpc_robot_config(fullfile(repo,'001','001','urdf','001.urdf'));
m=mixmpc_reference_config(r,p);
g=leg_mechanism_data(); zero=[atan2(-g(2,2),g(1,2));atan2(-g(2,2),-g(1,2))];
inputs=[]; geometry_error=0; speed_error=0; work_error=0; inverse_ratio=0;
for L=linspace(.16,p.Len_max,25)
    q=mixmpc_vertical_pose(m,L);
    for lean=linspace(-.3,.3,17)
        active=reshape(q(7:10),2,2)-lean;
        [joint,valid]=mixmpc_closure(r,active);
        assert(valid,'CAD branch disconnected');
        pose=mixmpc_robot_fk(r,joint);
        phi=zero-active(:,1);
        pos=Leg_Pos(phi(1),phi(2));
        endpoint=pose.p([1 3],r.side(1).frames(11))-pose.p([1 3],r.side(1).frames(1));
        geometry_error=max(geometry_error,norm([pos(1)*cos(pos(2));-pos(1)*sin(pos(2))]-endpoint));
        h=1e-6; J=zeros(2);
        for j=1:2
            pp=phi; pm=phi; pp(j)=pp(j)+h; pm(j)=pm(j)-h;
            a=Leg_Pos(pp(1),pp(2)); b=Leg_Pos(pm(1),pm(2));
            J(:,j)=[a(1)-b(1);atan2(sin(a(2)-b(2)),cos(a(2)-b(2)))]/(2*h);
        end
        vel=[.7;-.4]; load=[80;2];
        spd=Leg_Spd(phi(1),phi(2),vel(1),vel(2));
        torque=Leg_Trq(load(1),load(2),phi(1),phi(2));
        speed_error=max(speed_error,norm(spd-J*vel,Inf));
        work_error=max(work_error,abs(torque'*vel-load'*spd));
        [F,Tp]=Leg_InvTrq(torque(1),torque(2),phi(1),phi(2));
        sigma=min(svd(J));
        inverse_ratio=max(inverse_ratio,norm([F;Tp]-load)/norm(load)/(1e-6/sigma^2));
        inputs(end+1,:)=[phi',vel',load']; %#ok<AGROW>
    end
end
assert(geometry_error<1e-9 && speed_error<1e-7 && work_error<1e-9 && inverse_ratio<=1.02,'001 geometry/virtual-work verification failed');
fprintf('001 independent CAD FK/Jacobian/work PASS: %d poses, %.3e m, %.3e velocity, %.3e W\n',size(inputs,1),geometry_error,speed_error,work_error);
% Flatten only the public entry files to avoid incompatible transitive variants.
stage=fullfile(D.VERIFY_BUILD_DIR,'raw'); if ~isfolder(stage), mkdir(stage); end
names={'Leg_Pos','Leg_Spd','Leg_Trq','Leg_InvTrq'};
for k=1:numel(names)
    for ext={'.c','.h'}
        copyfile(fullfile(D.CODEGEN_DIR,names{k},[names{k} ext{1}]),stage);
    end
end
runtime={};
for name={'rt_nonfinite.c','rt_nonfinite.h','rt_defines.h','rtwtypes.h'}
    source=fullfile(D.CODEGEN_DIR,'Leg_Pos',name{1});
    if isfile(source)
        copyfile(source,stage);
        if endsWith(name{1},'.c'), runtime{end+1}=fullfile(stage,name{1}); end %#ok<AGROW>
    end
end
raw=cellfun(@(n)fullfile(stage,[n '.c']),names,'UniformOutput',false);
deployed=cellfun(@(n)fullfile(D.OUT_DIR,[n '.c']),names,'UniformOutput',false);
mex('-outdir',D.VERIFY_BUILD_DIR,'-output','leg_vmc_raw',['-I' stage],fullfile(D.SOLVER_DIR,'mex_leg_vmc.c'),raw{:},runtime{:});
mex('-outdir',D.VERIFY_BUILD_DIR,'-output','leg_vmc_cmsis',['-I' D.SOLVER_DIR],['-I' D.OUT_DIR],fullfile(D.SOLVER_DIR,'mex_leg_vmc.c'),deployed{:});
input=single(inputs);
[a,b,c,d]=leg_vmc_raw(input); [e,f,g,h]=leg_vmc_cmsis(input);
assert(isequaln(a,e) && isequaln(b,f) && isequaln(c,g) && isequaln(d,h),'Raw/deployed VMC C mismatch');
mat=zeros(size(input,1),8);
for k=1:size(input,1)
    v=input(k,:); pos=Leg_Pos(v(1),v(2)); spd=Leg_Spd(v(1),v(2),v(3),v(4));
    trq=Leg_Trq(v(5),v(6),v(1),v(2)); [F,Tp]=Leg_InvTrq(v(5),v(6),v(1),v(2));
    mat(k,:)=[pos',spd',trq',F,Tp];
end
err=max(abs(double([e f g h])-mat)./max(1,abs(mat)),[],'all');
assert(err<2e-5,'VMC C/MATLAB mismatch');
check_compensation(D,p,table);
fprintf('001 Tool raw/CMSIS/MATLAB comparison PASS, relative error %.3e\n',err);
end

function check_compensation(D,p,table)
% 原始生成 C、固件 C、MATLAB 三方比较，同时检查标定点与物理单位。
config=fullfile(fileparts(fileparts(D.ROOT)),'Wheel_Leg_Code','Code','Config');
for name={'Leg_Comp','Leg_SpringF','Leg_RollComp'}
    n=name{1}; tag=lower(n);
    wrapper=fullfile(D.SOLVER_DIR,['mex_' tag '.c']);
    lib=fullfile(D.CODEGEN_DIR,n);
    runtime={};
    if isfile(fullfile(lib,'rt_nonfinite.c')), runtime={fullfile(lib,'rt_nonfinite.c')}; end
    mex('-outdir',D.VERIFY_BUILD_DIR,'-output',[tag '_raw'],['-I' lib],wrapper,fullfile(lib,[n '.c']),runtime{:});
    mex('-outdir',D.VERIFY_BUILD_DIR,'-output',[tag '_cmsis'],['-I' D.SOLVER_DIR],['-I' D.OUT_DIR],['-I' config],wrapper,fullfile(D.OUT_DIR,[n '.c']));
end
node=single([table(:,1);(table(1:end-1,1)+table(2:end,1))/2;.05;.4;NaN;Inf;-Inf]);
[a,b]=leg_comp_raw(node); [c,d]=leg_comp_cmsis(node);
assert(isequaln(a,c) && isequaln(b,d),'Gravity compensation C mismatch');
for k=1:numel(node)
    [x,y]=Leg_Comp(node(k)); assert(max(abs(double([a(k)-x,b(k)-y])))<5e-7,'Gravity compensation MATLAB mismatch');
end
lengths=single([.05;linspace(.16,p.Len_max,200)';.34;.45;Inf;-Inf]);
a=leg_springf_raw(lengths); b=leg_springf_cmsis(lengths);
assert(max(abs(double(a-b)))<2e-5,'Spring lookup raw/deployed C differs');
for k=1:numel(lengths)
    expected=Leg_SpringF(lengths(k),single(0));
    assert(abs(double(b(k)-expected))<2e-5,'Spring C/MATLAB mismatch');
end
assert(abs(double(leg_springf_cmsis(single(.17)))-75.0)<2e-4 && ...
       abs(double(leg_springf_cmsis(single(.34)))-185.0)<2e-4,'Spring lookup endpoints changed');
[H,v,w]=ndgrid([.16 .17 .27 p.Len_max .4],[-3 -1 0 1 3],[-6 -1 0 1 6]);
input=single([H(:),w(:),v(:)]);
[a,b,c]=leg_rollcomp_raw(input); [d,e,f]=leg_rollcomp_cmsis(input);
assert(isequaln(a,d) && isequaln(b,e) && isequaln(c,f) && isequaln(d,-e),'Turning compensation C mismatch');
for k=1:size(input,1)
    [x,y,z]=Leg_RollComp(input(k,1),input(k,2),input(k,3));
    assert(max(abs(double([d(k)-x,e(k)-y,f(k)-z])))<1e-4,'Turning C/MATLAB mismatch');
end
fprintf('Mass/inertia table, empirical spring calibration and turning compensation PASS\n');
end
