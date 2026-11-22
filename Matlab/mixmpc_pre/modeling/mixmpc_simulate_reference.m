function sim = mixmpc_simulate_reference(model, q, v, input, terrain, duration, dt)
% RK4 with event localization between output times. input is a six-vector
% or a function tau = input(t,q,v). The state is continuous at release, and
% plastic touchdown changes v by a mechanically solved impulse.
validateattributes(duration,{'double'},{'scalar','positive','finite'});
validateattributes(dt,{'double'},{'scalar','positive','finite'});
count = ceil(duration/dt)+1;
grid = linspace(0,duration,count);
if ~isa(input,'function_handle')
    tau = input;
    input = @(~,~,~) tau;
end
time_tol = 1e-10;
tol = model.contact_tolerance;
events = struct('time',{},'kind',{},'before',{},'after',{},'q',{}, ...
    'v_before',{},'v_after',{},'impulse',{},'kinetic_loss',{},'balance_residual',{});
sim = struct('t',grid,'q',nan(12,count),'v',nan(12,count),'tau',nan(6,count), ...
    'reaction',nan(3,2,count),'gap',nan(2,count),'contact_speed',nan(2,count), ...
    'energy',nan(3,count),'linear_momentum',nan(3,count),'angular_momentum_com',nan(3,count), ...
    'equation_error',nan(1,count),'constraint_error',nan(1,count), ...
    'loop_error',nan(1,count),'friction_margin',nan(1,count),'joint_margin',nan(1,count), ...
    'contact_rank',nan(1,count),'wheel_center',nan(3,2,count), ...
    'mode',{cell(1,count)},'events',events,'status','','stop_time',0);
y = [q(:);v(:)];
t = 0;
p = state(t,y);
if strcmp(p.status,'impact_required')
    [y,p,ok] = touchdown(t,y,p);
    if ~ok, sim.status = p.status; end
end
last = 1;
record(last,p);
if isempty(sim.status) && ~strcmp(p.status,'ok')
    sim.status = p.status;
end
for sample = 2:count
    if ~isempty(sim.status), break, end
    switches = 0;
    while grid(sample)-t>time_tol
        h = grid(sample)-t;
        active = p.support;
        trial = advance(t,y,h,active);
        next = state(t+h,trial,active);
        [kind,release] = guard(next);
        if isempty(kind)
            t = grid(sample);
            y = trial;
            p = next;
            if ~strcmp(p.status,'ok'), sim.status = p.status; end
            break
        end

        % Locate the first contact guard within this RK step. Store the
        % crossing state with its solved impulse instead of clipping gaps.
        lo = 0;
        hi = h;
        while hi-lo>time_tol
            mid = (lo+hi)/2;
            probe_y = advance(t,y,mid,active);
            probe = state(t+mid,probe_y,active);
            [probe_kind,probe_release] = guard(probe);
            if isempty(probe_kind)
                lo = mid;
            else
                hi = mid;
                trial = probe_y;
                next = probe;
                kind = probe_kind;
                release = probe_release;
            end
        end
        switches = switches+1;
        t = t+hi;
        y = trial;
        if switches>6
            sim.status = 'unresolved_contact_chatter';
            p = next;
        elseif strcmp(kind,'touchdown')
            [y,p,ok] = touchdown(t,y,next);
            if ~ok, sim.status = p.status; end
        elseif strcmp(kind,'liftoff')
            p = state(t,y,active & ~release);
            events(end+1) = event(t,kind,next,p,y(13:24),zeros(3,2),0,0); %#ok<AGROW>
            if ~strcmp(p.status,'ok'), sim.status = p.status; end
        else
            sim.status = kind;
            p = next;
        end
        if ~isempty(sim.status), break, end
    end
    if ~isempty(sim.status), break, end
    last = sample;
    record(last,p);
end
if isempty(sim.status), sim.status = 'ok'; end
sim.stop_time = t;
fields = fieldnames(sim);
for i = 1:numel(fields)
    name = fields{i};
    if any(strcmp(name,{'events','stop_time','status'})), continue, end
    if ndims(sim.(name))==3
        sim.(name) = sim.(name)(:,:,1:last);
    else
        sim.(name) = sim.(name)(:,1:last);
    end
end
sim.events = events;
sim.terrain = terrain;
sim.failure = [];
if ~strcmp(sim.status,'ok')
    sim.failure = struct('time',t,'plant_status',p.status,'mode',p.mode,'q',p.dyn.kin.q, ...
        'v',p.dyn.kin.v,'gap',p.contact.gap,'contact_rank',p.contact_rank, ...
        'reaction',p.reaction,'friction_margin',p.friction_margin);
end

    function out = state(at,z,varargin)
        a = z(1:12);
        b = z(13:24);
        out = mixmpc_reference_plant(model,a,b,input(at,a,b),terrain,varargin{:});
    end

    function z = advance(at,z,h,active)
        k1 = slope(at,z,active);
        k2 = slope(at+h/2,z+h*k1/2,active);
        k3 = slope(at+h/2,z+h*k2/2,active);
        k4 = slope(at+h,z+h*k3,active);
        z = z+h*(k1+2*k2+2*k3+k4)/6;
    end

    function dy = slope(at,z,active)
        s = state(at,z,active);
        dy = [z(13:24);s.qdd];
    end

    function [kind,release] = guard(s)
        impact = ~s.support & s.contact.gap<=0 & s.contact.velocity(3,:)' < -tol.impact_speed;
        release = s.support & s.reaction(3,:)' < -tol.release_force;
        kind = '';
        if any(impact)
            kind = 'touchdown';
        elseif any(release)
            kind = 'liftoff';
        elseif ~s.wrench_admissible
            kind = 'sticking_feasibility_lost';
        elseif ~s.constraint_consistent
            kind = 'contact_rank_incompatible';
        end
    end

    function [z,after,ok] = touchdown(at,z,before)
        hit = mixmpc_reference_impact(model,z(1:12),z(13:24),terrain);
        after = before;
        if ~strcmp(hit.status,'ok')
            after.status = hit.status;
            ok = false;
            return
        end
        incoming = z(13:24);
        z(13:24) = hit.v;
        after = state(at,z,hit.support);
        if ~strcmp(after.status,'ok')
            after = state(at,z);
        end
        events(end+1) = event(at,'touchdown',before,after,incoming,hit.impulse, ...
            hit.kinetic_loss,hit.balance_residual);
        ok = strcmp(after.status,'ok');
    end

    function record(index,s)
        sim.q(:,index) = s.dyn.kin.q;
        sim.v(:,index) = s.dyn.kin.v;
        sim.tau(:,index) = input(t,sim.q(:,index),sim.v(:,index));
        sim.reaction(:,:,index) = s.reaction;
        sim.gap(:,index) = s.contact.gap;
        sim.contact_speed(:,index) = vecnorm(s.contact.velocity)';
        sim.energy(:,index) = s.dyn.energy;
        sim.linear_momentum(:,index) = s.dyn.linear_momentum;
        sim.angular_momentum_com(:,index) = s.dyn.angular_momentum_com;
        sim.equation_error(index) = norm(s.equation_residual,Inf);
        sim.constraint_error(index) = norm(s.contact_acceleration(:,s.support),'inf');
        sim.loop_error(index) = norm(s.dyn.kin.loop_position(:),Inf);
        sim.friction_margin(index) = min([Inf s.friction_margin(s.support)]);
        sim.joint_margin(index) = min(s.dyn.joint_margin);
        sim.contact_rank(index) = s.contact_rank;
        sim.wheel_center(:,:,index) = s.dyn.kin.pose.p(:,model.wheel_link);
        sim.mode{index} = s.mode;
    end
end

function e = event(t,kind,before,after,incoming,impulse,loss,residual)
e = struct('time',t,'kind',kind,'before',before.mode,'after',after.mode, ...
    'q',before.dyn.kin.q,'v_before',incoming,'v_after',after.dyn.kin.v, ...
    'impulse',impulse,'kinetic_loss',loss,'balance_residual',residual);
end
