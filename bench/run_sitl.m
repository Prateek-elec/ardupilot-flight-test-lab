% run_sitl.m - real ArduCopter SITL firmware flying the validated MATLAB physics.
% In MATLAB:  cd(fullfile(userpath, "fc_test", "bench")); run_sitl
% Watch live in Mission Planner: connection type TCP, 127.0.0.1, port 5762.
out = run_sitl_bench();
