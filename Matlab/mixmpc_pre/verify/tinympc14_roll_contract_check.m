function tinympc14_roll_contract_check(p)
% Catch a predictor/plant mismatch before relative tracking gates obscure its cause.
if nargin<1, p=tinympc14_config(); end
max_drift=0; max_fed=0;
for roll=[-0.09,0,0.09]
    sp=tinympc14_sched_pt(p,0.27,roll,0);
    plant=tinympc14_plant_init(p,sp); x=plant.xb;
    force=tinympc14_roll_force(p,p.H_REF+x(13),x(11),plant.force_roll,plant.force_psi);
    [~,~,slope]=tinympc14_hr_params(p,p.H_REF+x(13));
    moment=p.ROLL_TIP_GAIN*slope*x(11);
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
fprintf('ROLL CONTRACT PASS: nonzero-roll cancellation residual %.3e, prediction mismatch %.3e\n',max_drift,max_fed);
end
