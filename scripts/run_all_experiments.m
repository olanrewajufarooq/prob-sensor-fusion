% Master script to run everything
clear; close all;
setup_paths();

fprintf('======================================\n');
fprintf('  RUNNING ALL EXPERIMENTS\n');
fprintf('======================================\n\n');

% Run each experiment
fprintf('Step 1/5: Running baseline...\n');
run_baseline;

fprintf('\nStep 2/5: Running correlated noise...\n');
run_correlated;

fprintf('\nStep 3/5: Running heavy-tailed noise...\n');
run_heavytail;

fprintf('\nStep 4/5: Running robust filters...\n');
run_robust;

fprintf('\nStep 5/5: Generating comparison plots...\n');
compare_all;

fprintf('\n======================================\n');
fprintf('  ALL EXPERIMENTS COMPLETE!\n');
fprintf('  Check results/ and figures/ folders\n');
fprintf('======================================\n');