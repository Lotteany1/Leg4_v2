% Full LPV grid/table generation; Q/R-only channel: build_mixmpc_pre_fast.
addpath(fileparts(fileparts(mfilename('fullpath'))));
mixmpc_paths();
tinympc14_lpv_generate(false);
