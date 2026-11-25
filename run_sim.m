% run_sim.m  -  run the flight-log-calibrated simulation on the newest log in logs/.
% In MATLAB, from this folder:  run_sim      (or set logFile below to pick a specific log)
% Set quick = false for the full-resolution tune search and 300 Monte-Carlo runs.
quick = true;
logFile = lab_default_log();
tic
S = fc_simulation(logFile, Quick=quick);
fprintf('Total time %.0f s\n', toc);
