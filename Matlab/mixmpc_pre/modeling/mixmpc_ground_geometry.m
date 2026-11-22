function [length_m, height_interval] = mixmpc_ground_geometry(height, roll, leg_angle, ground_z, half_track, limits)
% Sagittal roll-then-pitch geometry for two wheels on horizontal planes.
% H is the hip-midpoint height above the mean wheel-center height.
% Positive roll raises the left hip. Equal thin-wheel radii cancel from the
% wheel-center height difference, even though their common height varies with roll.
vertical = cos(roll).*cos(leg_angle);
assert(all(vertical>0), 'Both legs must point downward in the contact chart.');
offset = [half_track -half_track]*sin(roll) - (ground_z-mean(ground_z));
length_m = (height+offset)./vertical;
height_interval = [max(limits(1)*vertical-offset), min(limits(2)*vertical-offset)];
end
