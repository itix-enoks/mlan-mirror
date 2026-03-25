%% Main Execution Script for 802.11ac vs 802.11ax Analysis
clc; close all; % Removed 'clear' here to be safe

% 1. Create the figures directory if it doesn't exist
if ~exist('figures', 'dir')
    mkdir('figures');
end

% 2. Define the scripts
% We use a string array to help prevent workspace conflicts
all_scripts = ["system_level_tot_ind_thr.m", ...
               "system_level_pak_del_drop.m", "system_level_jain_se.m"];

fprintf('Starting Full Simulation Suite...\n');

for i = 1:length(all_scripts)
    % Re-assign the target script inside the loop so it survives a 'clear' 
    % inside the called script
    current_script = all_scripts(i); 
    
    fprintf('Running: %s ...\n', current_script);
    
    % Execute the script
    run(current_script);
    
    % IMPORTANT: After 'run' finishes, the 'clear' inside the next 
    % script will run. We need to make sure the loop can continue.
end

fprintf('All simulations complete. Check the "figures" folder.\n');