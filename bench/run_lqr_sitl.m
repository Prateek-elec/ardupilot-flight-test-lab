% run_lqr_sitl.m - fly the LQR tune from lqr_pid_tuner.m on the real ArduCopter firmware
% (SITL + MATLAB 6-DOF physics) against the flown 12 Nov tune and Tune C.
%
% Three plants, same vibration/turbulence/seed for every run:
%   "p12nov"          - physics set to the plant identified from the 12 Nov flights (the LQR
%                       design plant): authority 538/370/55 rad/s^2 per unit, motor lag 25 ms
%   "p12nov_delay7ms" - same, plus 7 ms extra motor-command delay (worst-case delay test)
%   "p9nov"           - the default quad_params calibration (297/224/36, 30 ms): ~45 % less
%                       authority, an off-design robustness check
% In MATLAB:  cd(fullfile(userpath, "fc_test", "bench")); run_lqr_sitl

here = fileparts(mfilename('fullpath'));
G = readtable(fullfile(fileparts(here), "fleet_results", "lqr_tuner", "gains.csv"), 'TextType', 'string');
pick = @(ax, c) G(G.Axis == ax & G.Case == c, :);
tuneOverrides = @(r, p, y) { ...
    "ATC_RAT_RLL_P", r.P; "ATC_RAT_RLL_I", r.I; "ATC_RAT_RLL_D", r.D; ...
    "ATC_RAT_PIT_P", p.P; "ATC_RAT_PIT_I", p.I; "ATC_RAT_PIT_D", p.D; ...
    "ATC_RAT_YAW_P", y.P; "ATC_RAT_YAW_I", y.I; "ATC_RAT_YAW_D", y.D; ...
    "INS_GYRO_FILTER", 91};                       % the filter the 12 Nov plant was identified with
cases = ["as flown 12 Nov", "Tune C (post_crash_fix)", "LQR step 1"];
labels = ["A_flown12Nov", "B_tuneC", "C_LQR"];

env = {'VibAccel', [3.6 2.4 13], 'VibGyro', deg2rad([4 4 2]), 'GyroNoise', deg2rad(0.8), ...
       'Gusts', 1.8, 'Wind', [3 1.5 0], 'Seed', 11};
p12 = quad_params('auth_roll', 538, 'auth_pitch', 370, 'auth_yaw', 55, 'tau_m', 0.025);
plants = struct('tag', {"p12nov", "p12nov_delay7ms", "p9nov"}, ...
    'P', {p12, p12, quad_params()}, ...
    'delay', {0, 0.007, 0});            % +7 ms: total ~13 ms, the phase-based (worst-case) roll delay

SR = struct();
for p = 1:numel(plants)
    res = struct('label', {}, 'A', {}, 'runDir', {});
    for k = 1:numel(cases)
        ov = tuneOverrides(pick("roll", cases(k)), pick("pitch", cases(k)), pick("yaw", cases(k)));
        lab = labels(k) + "_" + plants(p).tag;
        fprintf('\n\n################ %s plant: %s ################\n', plants(p).tag, cases(k));
        try
            o = run_sitl_bench('Label', lab, 'ParamOverrides', ov, 'Params', plants(p).P, 'MotorDelay', plants(p).delay, env{:});
            res(end+1) = struct('label', labels(k), 'A', o.analysis, 'runDir', o.runDir); %#ok<SAGROW>
        catch e
            fprintf(2, 'RUN FAILED (%s): %s\n', lab, e.message);
        end
        pause(3);
    end
    SR.(plants(p).tag) = res;
    save(fullfile(here, "results", "lqr_sitl.mat"), 'SR', 'cases', 'env');
    if numel(res) > 1, compare_tunes(res, "lqr_" + plants(p).tag); end
end
