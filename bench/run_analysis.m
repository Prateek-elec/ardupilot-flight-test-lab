% run_analysis.m - analyse the latest SITL flight and compare with the real flight.
% In MATLAB, from the bench folder:  run_analysis
here = fileparts(mfilename('fullpath'));
runs = dir(fullfile(here, "..", "sitl", "run_*"));
[~, i] = max([runs.datenum]); runDir = fullfile(runs(i).folder, runs(i).name);
logs = dir(fullfile(runDir, "logs", "*.BIN"));
S = load(fullfile(runDir, "physics_log.mat"));
A = sitl_analysis(fullfile(logs(end).folder, logs(end).name), S.L);

% ---- same metrics on the REAL flight (10 Hz log) ----
realLog = lab_default_log();   % the real flight to compare with (newest log in logs/)
R = compare_real_vs_sitl(realLog, fullfile(logs(end).folder, logs(end).name));
save(fullfile(runDir, "analysis.mat"), 'A', 'R');
