function setup_paths()
%SETUP_PATHS Configure path for workspace

% Add Path for folders containing codes
addpath("classes\");
addpath("scripts\");
addpath("utils\");

% Create output directories for results and figures
if ~exist('results', 'dir') 
    mkdir('results'); 
end

if ~exist('plots', 'dir') 
    mkdir('plots'); 
end

fprintf('Paths configured. Now ready to run experiments. \n');

end

