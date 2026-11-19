function T = run_tuner_sitl(resultDir)
%RUN_TUNER_SITL Fly the tuner's step-1 gains against the current tune on ArduCopter SITL.
%   T = run_tuner_sitl("...\2026-11-12 17-04-34_lqr_tune")
%   Uses gains.csv and quality.csv written by fc_tune_from_log. Three plants, same
%   vibration/turbulence/seed for every run:
%     pfit           - physics set to the identified plant (authority, motor lag)
%     pfit_delay7ms  - same, +7 ms extra motor-command delay (worst-case delay)
%     pweak          - 45 % less control authority (as identified on 9 Nov)
%   Then lqr_sitl_check analyses rate-loop ringing and tracking from the logs.
G = readtable(fullfile(resultDir, "gains.csv"), 'TextType', 'string');
Q = readtable(fullfile(resultDir, "quality.csv"), 'TextType', 'string');
K = @(ax) Q.ModelK(Q.Axis == ax); tm = mean([Q.ModelMotorLag_ms(Q.Axis == "roll"), Q.ModelMotorLag_ms(Q.Axis == "pitch")]) / 1000;
pick = @(ax, c) G(G.Axis == ax & G.Case == c, :);
ovr = @(c) { ...
    "ATC_RAT_RLL_P", pick("roll", c).P; "ATC_RAT_RLL_I", pick("roll", c).I; "ATC_RAT_RLL_D", pick("roll", c).D; ...
    "ATC_RAT_PIT_P", pick("pitch", c).P; "ATC_RAT_PIT_I", pick("pitch", c).I; "ATC_RAT_PIT_D", pick("pitch", c).D; ...
    "ATC_RAT_YAW_P", pick("yaw", c).P; "ATC_RAT_YAW_I", pick("yaw", c).I; "ATC_RAT_YAW_D", pick("yaw", c).D};
tunes = struct('case', {"current (from log)", "LQR step 1"}, 'label', {"A_current", "C_new"});
env = {'VibAccel', [3.6 2.4 13], 'VibGyro', deg2rad([4 4 2]), 'GyroNoise', deg2rad(0.8), 'Gusts', 1.8, 'Wind', [3 1.5 0], 'Seed', 11};
Pfit = quad_params('auth_roll', K("roll"), 'auth_pitch', K("pitch"), 'auth_yaw', K("yaw"), 'tau_m', tm);
Pweak = quad_params('auth_roll', 0.55 * K("roll"), 'auth_pitch', 0.55 * K("pitch"), 'auth_yaw', 0.65 * K("yaw"), 'tau_m', tm);
plants = struct('tag', {"pfit", "pfit_delay7ms", "pweak"}, 'P', {Pfit, Pfit, Pweak}, 'delay', {0, 0.007, 0});
stamp = string(datetime('now', 'Format', 'yyyyMMdd'));
for p = 1:numel(plants)
    for t = 1:numel(tunes)
        lab = tunes(t).label + "_" + plants(p).tag;
        fprintf('\n\n################ %s plant: %s ################\n', plants(p).tag, tunes(t).case);
        try
            run_sitl_bench('Label', lab, 'ParamOverrides', ovr(tunes(t).case), 'Params', plants(p).P, ...
                'MotorDelay', plants(p).delay, env{:});
        catch e
            fprintf(2, 'RUN FAILED (%s): %s\n', lab, e.message);
        end
        pause(3);
    end
end
T = lqr_sitl_check("run_" + stamp + "_*_p*");
T = T(startsWith(T.Tune, ["A_current", "C_new"]), :);
disp(T);
writetable(T, fullfile(resultDir, "sitl_tuner_check.csv"));
end
