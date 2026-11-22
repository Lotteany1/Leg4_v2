function next = tinympc14_plant_step(p, plant, x, input)
% Frozen ZOH plant plus the model-external tipping disturbance, held for one node.
force = tinympc14_roll_force(p,p.H_REF+x(13),x(11),plant.force_roll,plant.force_psi);
input(5:6) = input(5:6) + force;
next = plant.Adp*x + plant.Bdp*input + plant.fdp;
end
