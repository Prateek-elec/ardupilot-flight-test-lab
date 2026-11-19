% run_tune_tests.m - fly candidate tunes on the real ArduCopter firmware (SITL)
% with realistic vibration, gyro noise and turbulence, then compare.
% In MATLAB:  cd(fullfile(userpath, "fc_test", "bench")); run_tune_tests

% Environment matched to the 9 Nov flight: VIBE on IMU1 ~ 3.6/2.4/13 m/s^2,
% stronger gusts. Same random seed for every run -> identical conditions.
env = {'VibAccel', [3.6 2.4 13], 'VibGyro', deg2rad([4 4 2]), 'GyroNoise', deg2rad(0.8), ...
       'Gusts', 1.8, 'Wind', [3 1.5 0], 'Seed', 11};

tunes = { ...
  "A_current", {}; ...
  "B_yawP",    {"ATC_RAT_YAW_P", 0.34; "ATC_RAT_YAW_I", 0.034}; ...
  "C_yawP_pitchD", {"ATC_RAT_YAW_P", 0.34; "ATC_RAT_YAW_I", 0.034; "ATC_RAT_PIT_D", 0.0034}};

res = struct('label', {}, 'A', {}, 'runDir', {});
for k = 1:size(tunes, 1)
    fprintf('\n\n################ TUNE %d/%d: %s ################\n', k, size(tunes,1), tunes{k,1});
    o = run_sitl_bench('Label', tunes{k,1}, 'ParamOverrides', tunes{k,2}, env{:});
    res(k).label = tunes{k,1}; res(k).A = o.analysis; res(k).runDir = o.runDir;
    pause(3);
end
save(fullfile(fileparts(mfilename('fullpath')), "results", "tune_tests.mat"), 'res', 'tunes', 'env');
compare_tunes(res);
