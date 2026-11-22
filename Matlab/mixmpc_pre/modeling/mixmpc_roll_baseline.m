function [cases, traces] = mixmpc_roll_baseline(p, terminal)
% Legacy 14-state plant, observed using the two-wheel geometry. Stop at the
% first crossing of the declared training bounds, before extrapolating the plant.
% The comparison changes only the reference and uses the same plant and solver.
sw = p.sym7.sw_fused;
p.ONLINE_TERMINAL = terminal;
length_limit = p.LEG_RANGE;
opts = [ 1  0.0 0.002 0.00  0.00 0.000
         3  0.2 0.005 0.00  0.00 0.000
         5  0.5 0.010 0.15 -0.10 0.000
         3  0.2 0.010 0.12 -0.08 0.012 ];
dt = 0.002;
steps = round(1.6/dt);
cases = cell(0);
traces = cell(0);
for row = 1:size(opts,1)
    for mirror = [1 -1]
        angle = mirror*deg2rad(opts(row,1));
        rate = mirror*opts(row,2);
        psi = opts(row,4:5);
        floor_z = [0 opts(row,6)];
        if mirror<0
            psi = fliplr(psi);
            floor_z = fliplr(floor_z);
        end
        [~,hbox] = mixmpc_ground_geometry(0,angle,psi,floor_z,p.R_TRK,length_limit);
        H = hbox(2)-opts(row,3);
        lens = mixmpc_ground_geometry(H,angle,psi,floor_z,p.R_TRK,length_limit);
        assert(all(lens>=length_limit(1) & lens<=length_limit(2)), 'Invalid initial leg lengths.');
        initial = tinympc14_fused_eq(p,sw,tinympc14_sched_pt(p,p.H_REF,0,0));
        initial([5 7 11 12 13]) = [psi'; angle; rate; H-p.H_REF];
        xr = zeros(p.NX,1);
        xr(9) = p.sym7.pitch_ref;
        requested_h = length_limit(2);
        for margin = [0 0.005]
            rt = tinympc14_runtime_init(p,2);
            x = initial;
            xlog = nan(p.NX,steps+1);
            ulog = nan(p.NU,steps+1);
            lengths = nan(2,steps+1);
            href = nan(1,steps+1);
            min_room = Inf;
            status = 'no_recovery_within_window';
            for k = 1:steps+1
                time = (k-1)*dt;
                h = p.H_REF + x(13);
                len = mixmpc_ground_geometry(h,x(11),x([5,7])',floor_z,p.R_TRK,length_limit);
                xlog(:,k) = x;
                lengths(:,k) = len';
                min_room = min([min_room len-length_limit(1) length_limit(2)-len]);
                if any(len<length_limit(1) | len>length_limit(2))
                    status = 'training_length_crossing';
                    break
                end
                if k>steps, break, end

                xr(13) = requested_h-p.H_REF;
                if margin>0
                    [~,window] = mixmpc_ground_geometry(h,x(11),x([5,7])',floor_z,p.R_TRK, ...
                        length_limit+[margin -margin]);
                    if window(1)>window(2)
                        status = 'empty_height_interval';
                        break
                    end
                    xr(13) = min(max(requested_h,window(1)),window(2))-p.H_REF;
                end
                href(k) = xr(13)+p.H_REF;
                [rt,y] = tinympc14_runtime_step(rt,time*1000,x,xr,len(1),len(2),x(5),x(7),x(2),x(4));
                u = y.u_plant;
                ulog(:,k) = u;
                op = tinympc14_fused_op(p,struct('lL',len(1),'lR',len(2),'H',h,'psi',mean(x([5 7]))));
                a = tinympc14_fused_dyn(p,sw,op,x,u);
                b = tinympc14_fused_dyn(p,sw,op,x+dt*a/2,u);
                c = tinympc14_fused_dyn(p,sw,op,x+dt*b/2,u);
                d = tinympc14_fused_dyn(p,sw,op,x+dt*c,u);
                x = x + dt*(a+2*b+2*c+d)/6;
            end
            last_outside = find(abs(xlog(11,1:k))>deg2rad(1),1,'last');
            if isempty(last_outside), last_outside = 0; end
            recovery_s = last_outside*dt;
            if k==steps+1 && recovery_s<=1 && time-recovery_s>=0.5
                status = 'roll_recovered_inside_training_bounds';
            end
            cases{end+1} = struct('initial_roll_deg',rad2deg(angle), 'initial_roll_rate_rad_s',rate, ... %#ok<AGROW>
                'ground_z_m',floor_z,'initial_length_m',lens,'reference_margin_m',margin, ...
                'raw_height_ref_m',requested_h,'status',status,'end_time_s',time, ...
                'min_length_room_m',min_room,'end_roll_deg',rad2deg(xlog(11,k)), ...
                'peak_input_abs',max(abs(ulog(:,1:k)),[],2,'omitnan'), ...
                'min_projected_height_ref_m',min(href(1:k),[],'omitnan'), ...
                'solve_success_ratio',mean(rt.conv));
            traces{end+1} = struct('t',(0:k-1)*dt,'x',xlog(:,1:k),'u',ulog(:,1:k), ... %#ok<AGROW>
                'length_m',lengths(:,1:k),'height_ref_m',href(1:k));
        end
    end
end
cases = [cases{:}];
end
