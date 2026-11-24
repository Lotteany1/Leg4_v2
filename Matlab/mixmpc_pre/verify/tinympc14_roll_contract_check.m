function tinympc14_roll_contract_check(p)
% Catch a predictor/plant mismatch before relative tracking gates obscure its cause.
if nargin<1, p=tinympc14_config(); end
max_drift=0; max_fed=0;
for roll=[-0.09,0,0.09]
    sp=tinympc14_sched_pt(p,0.27,roll,0);
    plant=tinympc14_plant_init(p,sp); x=plant.xb;
    force=tinympc14_roll_force(p,p.H_REF+x(13),x(11),plant.force_roll,plant.force_psi);
    moment=p.ROLL_TIP_GAIN*p.M_SUP*p.g*(p.H_REF+x(13)+p.H_OFF)*x(11);
    [h_arm,r_arm]=tinympc14_leg_jacobian(p.R_TRK,sp.H,plant.force_roll,plant.force_psi');
    assert(norm([h_arm*force;r_arm*force-moment],Inf)<1e-12,'Roll disturbance moment/sign mismatch');
    hold=plant.uss; hold(5:6)=hold(5:6)-force;
    % Frozen scenario linearization need not be exactly at the nonlinear equilibrium.
    nominal=plant.Adp*x+plant.Bdp*plant.uss+plant.fdp;
    drift=norm(tinympc14_plant_step(p,plant,x,hold)-nominal,Inf);
    max_drift=max(max_drift,drift);
    assert(drift<1e-9,'Plant omitted or duplicated the tipping disturbance at a nonzero roll equilibrium');
    rt=tinympc14_runtime_init(p);
    [rt,y]=tinympc14_runtime_step(rt,0,x,x,sp.lL,sp.lR,x(5),x(7),x(2),x(4));
    model_force=tinympc14_roll_force(p,p.H_REF+x(13),x(11),rt.model.force_roll,rt.model.force_psi);
    expected=rt.model.Bd(:,5:6)*model_force;
    max_fed=max(max_fed,norm(y.fed-expected,Inf));
    assert(norm(y.fed-expected,Inf)<1e-12,'Runtime tipping prediction differs from plant disturbance');
end
if isfield(p,'roll_height') && ~isfield(p,'URDF_SOURCE')
    raw = [.14 .14 0; .13 .13 0; .12 .12 0; p.roll_height(1) p.roll_height(1) 0; ...
           .14 .14 0; .12 .15 .2];
    rt = tinympc14_runtime_init(p);
    for k=1:size(raw,1)
        left=raw(k,1); right=raw(k,2); roll=raw(k,3);
        x=zeros(p.NX,1); x([5,7])=.1; x(11)=roll;
        [rt,y]=tinympc14_runtime_step(rt,10*k,x,x,left,right,.1,.1,0,0);
        h=mean([left right])*cos(roll);
        expected=min(max(h,p.roll_height(1)),p.roll_height(end));
        assert(y.lpv_updated && abs(rt.last_roll_height-expected)<1e-12);
        assert(isequal(rt.model.lL,double(single(max(left,p.LEG_RANGE(1))))) && ...
               isequal(rt.model.lR,double(single(max(right,p.LEG_RANGE(1))))));
        ref=tinympc14_lpv_eval(p,left,right,.1,h);
        assert(isequal(rt.model.Bd,ref.Bd),'Measured roll inertia missed the live dynamics');
    end
end
fprintf('ROLL CONTRACT PASS: nonzero-roll cancellation residual %.3e, prediction mismatch %.3e\n',max_drift,max_fed);
end
