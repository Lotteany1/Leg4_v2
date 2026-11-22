% Full nominal table generation; fast reuses the exact model stage.
addpath(fileparts(fileparts(mfilename('fullpath'))));
mixmpc_paths();
tinympc14_cache_generate(false);
