function target = mpc_output_path(target)
%MPC_OUTPUT_PATH Resolve against MATLAB pwd, not the JVM startup directory.
target = char(target);
if ~java.io.File(target).isAbsolute()
    target = fullfile(pwd, target);
end
target = char(java.io.File(target).getCanonicalPath());
end
