function target = tinympc14_height_reference(p,x,applied,target)
% 留出按当前速度、已有轴力、变化率和控制延迟计算的制动距离。
% target 为绝对髋高 [m]；不改用户目标，只限制本拍交给控制器的参考。
if ~isfield(p,'CAD_MODEL'), return; end
projection=cos(x(11))*cos(x([5,7]));
if any(projection<=0), return; end
offset=[1;-1]*p.R_TRK*sin(x(11));
lower=max(p.WORK_LEN_MIN*projection-offset);
upper=min(p.LEG_RANGE(2)*projection-offset);
if lower>=upper, return; end
arms=1./projection;
force=applied(5:6);
load=p.WORK_LOAD_MIN;
mass=p.WORK_MASS_MAX;
down=(load-p.F_MIN*sum(arms))/mass;
up=(p.F_MAX*sum(arms)-p.WORK_LOAD_MAX)/mass;
if down<=0 || up<=0, return; end
slew=2*p.DF_RATE/p.Ts;
delay=3*p.Ts;
up_delay=delay+max(sum(force)-2*p.F_MIN,0)/slew;
down_delay=delay+max(2*p.F_MAX-sum(force),0)/slew;
up_acc=max((arms'*force-load)/mass,0);
down_acc=max((p.WORK_LOAD_MAX-arms'*force)/mass,0);
rise=max(x(14),0); fall=max(-x(14),0);
rise_end=rise+up_acc*up_delay;
fall_end=fall+down_acc*down_delay;
upper_margin=rise*up_delay+.5*up_acc*up_delay^2+rise_end^2/(2*down);
lower_margin=fall*down_delay+.5*down_acc*down_delay^2+fall_end^2/(2*up);
safe_upper=max(lower,upper-upper_margin);
safe_lower=min(safe_upper,lower+lower_margin);
target=min(max(target,safe_lower),safe_upper);
end
