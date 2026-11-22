function hit = mixmpc_reference_impact(model, q, v, terrain)
% Plastic, sticking impact against the measured near-zero-gap planes.
% A candidate impulse must be compressive, inside each cone, and leave the
% omitted touching wheels with nonclosing normal velocity.
dyn = mixmpc_full_dynamics(model,q,v);
c = mixmpc_contact_model(model,dyn.kin,terrain);
v = dyn.kin.v;
tol = model.contact_tolerance;
near = abs(c.gap)<=tol.gap;
hit = struct('status','no_admissible_sticking_impact','v',v,'impulse',zeros(3,2), ...
    'support',false(2,1),'gap',c.gap,'kinetic_loss',0,'balance_residual',NaN);
if any(c.gap < -tol.gap) || ~any(near & c.velocity(3,:)' < -tol.impact_speed)
    hit.status = 'no_valid_touchdown';
    return
end
for code = [3 1 2]
    support = logical(bitget(uint8(code),[1;2]));
    if any(support & ~near), continue, end
    rows = repelem(support,3);
    [v_plus,lambda,info] = mixmpc_contact_kkt(dyn.M,v,c.J(rows,:),zeros(nnz(rows),1),c.mu(support));
    post = reshape(c.J*v_plus,3,2);
    if ~info.consistent || ~info.admissible || any(post(3,near & ~support)<-tol.impact_speed)
        continue
    end
    hit.status = 'ok';
    hit.support = support;
    hit.v = v_plus;
    hit.impulse(rows) = lambda;
    hit.kinetic_loss = 0.5*(v'*dyn.M*v-v_plus'*dyn.M*v_plus);
    hit.balance_residual = norm(dyn.M*(v_plus-v)-c.J'*hit.impulse(:),Inf);
    return
end
end
