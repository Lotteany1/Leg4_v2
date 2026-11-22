% Current solver correctness is checked against a condensed QP, rather than
% reproducing the inherited Leg3 reference-cost defect.
clear; clc
here=fileparts(mfilename('fullpath')); addpath(here); addpath(fileparts(here)); mixmpc_paths();
tinympc14_audit_regression();
fprintf('S5a independent scalar/reference regressions: PASS\n');
