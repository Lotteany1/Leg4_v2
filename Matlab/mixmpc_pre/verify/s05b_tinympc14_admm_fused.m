% Exact constrained QP checks for the current 14-state, 6-input formulation.
clear; clc
here=fileparts(mfilename('fullpath')); addpath(here); addpath(fileparts(here)); mixmpc_paths();
tinympc14_qp_check();
fprintf('S5b constrained QP verification: PASS\n');
