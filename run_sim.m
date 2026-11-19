% run_sim.m  -  run the flight-log-calibrated simulation on the 9 Nov 2026 flight.
% In MATLAB:  cd(fullfile(userpath, "fc_test")); run_sim
% Set quick = false for the full-resolution tune search and 300 Monte-Carlo runs.
quick = true;
logFile = "logs\2026-11-09 16-30-17.bin";
tic
S = fc_simulation(logFile, Quick=quick);
fprintf('Total time %.0f s\n', toc);
