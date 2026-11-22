function tinympc14_bundle_online(generated_dir, solver_dir)
% Keep the generated scheduler with the solver and its shared tables with the data.
files = dir(fullfile(generated_dir, '*.c'));
assert(any(strcmp({files.name}, 'tinympc14_online_model.c')), 'Missing Coder entry point.');
included = {};
for k = 1:numel(files)
    name = files(k).name;
    if strcmp(name, 'tinympc14_online_model_data.c')
        continue
    end
    append('tinympc14.c', expand(name));
end
if any(strcmp({files.name}, 'tinympc14_online_model_data.c'))
    included = {};
    append('tinympc14_data.c', expand('tinympc14_online_model_data.c'));
end

    function code = expand(name)
        if any(strcmp(name, included))
            code = '';
            return
        end
        included{end+1} = name;
        code = fileread(fullfile(generated_dir, name));
        if strcmp(name,'tmwtypes.h')
            % Preserve the licensed type definitions in the translation unit.
            code = regexprep(code,'(?m)^#\s*pragma once[^\r\n]*','');
        end
        [starts, stops, tokens] = regexp(code, '(?m)^#include "([a-zA-Z0-9_]+\.h)"[^\r\n]*', ...
            'start', 'end', 'tokens');
        pieces = cell(1,2*numel(starts)+1);
        cursor = 1;
        for index = 1:numel(starts)
            pieces{2*index-1} = code(cursor:starts(index)-1);
            header = tokens{index}{1};
            if strcmp(header,'mwstdint.h')
                % MATLAB's internal tooling header is behind MW_LIBTOOLING.
                pieces{2*index} = code(starts(index):stops(index));
            else
                pieces{2*index} = expand(header);
            end
            cursor = stops(index)+1;
        end
        pieces{end} = code(cursor:end);
        code = [pieces{:}];
    end

    function append(name, code)
        fid = fopen(fullfile(solver_dir, name), 'a');
        assert(fid >= 0, 'Cannot append to %s.', name);
        cleanup = onCleanup(@() fclose(fid)); %#ok<NASGU>
        fprintf(fid, '\n%s', code);
    end
end
