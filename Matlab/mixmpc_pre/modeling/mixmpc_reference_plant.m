function plant = mixmpc_reference_plant(model, q, v, tau, terrain, fixed_support)
% Constrained CAD reference for sticking/rolling or flight. Logical support
% order is [left right]. With no fixed support, test unilateral feasibility.
% Fixed-mode calls retain the trial wrench so an integrator can locate a guard.
tau = tau(:);
validateattributes(tau,{'double'},{'numel',6,'real','finite'});
dyn = mixmpc_full_dynamics(model,q,v);
c = mixmpc_contact_model(model,dyn.kin,terrain);
rhs = [zeros(6,1); tau]+dyn.Qspring-dyn.h;
free = dyn.M\rhs;
tol = model.contact_tolerance;
normal_v = c.velocity(3,:)';
near = c.gap<=tol.gap;
closed = near;
if nargin>=6
    assert(numel(fixed_support)==2,'Expected left and right support flags.');
    % During event localization, a positive inactive gap is still open.
    closed = c.gap<=0;
    plant = evaluate(logical(fixed_support(:)));
    return
end

if any(c.gap < -tol.gap)
    plant = evaluate(false(2,1));
    plant.status = 'penetration';
    return
end
if any(dyn.joint_margin < -1e-6)
    plant = evaluate(false(2,1));
    return
end
if any(near & normal_v < -tol.impact_speed)
    plant = evaluate(false(2,1));
    plant.status = 'impact_required';
    return
end

eligible = near & (vecnorm(c.velocity)'<=tol.velocity);
for code = [3 1 2 0]
    support = logical(bitget(uint8(code),[1;2]));
    if any(support & ~eligible), continue, end
    plant = evaluate(support);
    if strcmp(plant.status,'ok'), return, end
end
plant.status = 'no_admissible_rolling_mode';

    function p = evaluate(support)
        rows = repelem(support,3);
        [a, f, info] = mixmpc_contact_kkt(dyn.M,free,c.J(rows,:),-c.bias(rows),c.mu(support));
        wrench = zeros(6,1);
        wrench(rows) = f;
        wrench = reshape(wrench,3,2);
        accel = reshape(c.J*a+c.bias,3,2);
        residual = dyn.M*a-rhs-c.J'*wrench(:);
        labels = {'FLIGHT','LS','RS','DS'};
        status = 'ok';
        if ~info.consistent
            status = 'contact_rank_incompatible';
        elseif any(dyn.joint_margin < -1e-6)
            status = 'cad_joint_limit';
        elseif ~info.admissible
            status = 'contact_force_infeasible';
        elseif any(c.gap < -tol.gap) || any(abs(c.gap(support))>tol.gap) || ...
                any(vecnorm(c.velocity(:,support))>tol.velocity)
            status = 'inconsistent_contact_state';
        elseif any(~support & closed & (normal_v < -tol.impact_speed | ...
                (abs(normal_v)<=tol.impact_speed & accel(3,:)' < -tol.normal_acceleration)))
            status = 'inactive_contact_closing';
        end
        p = struct('qdd',a,'support',support,'mode',labels{1+[1 2]*support}, ...
            'status',status,'contact_rank',info.rank,'constraint_consistent',info.consistent, ...
            'wrench_admissible',info.admissible,'contact',c,'contact_acceleration',accel, ...
            'reaction',wrench,'reaction_world',reshape(pagemtimes(c.basis,reshape(wrench,3,1,2)),3,2), ...
            'friction_margin',c.mu'.*wrench(3,:)-vecnorm(wrench(1:2,:)), ...
            'equation_residual',residual,'dyn',dyn);
    end
end
