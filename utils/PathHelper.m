classdef PathHelper
    methods (Static)
        function full_path = ensurePath(base_dir, subfolder, filename, create_if_missing)
            if nargin < 4
                create_if_missing = false;
            end
            if isempty(subfolder)
                target_dir = base_dir;
            else
                target_dir = fullfile(base_dir, subfolder);
            end
            if create_if_missing && ~exist(target_dir, 'dir')
                mkdir(target_dir);
            end
            if nargin < 3 || isempty(filename)
                full_path = target_dir;
            else
                full_path = fullfile(target_dir, filename);
            end
        end
    end
end
