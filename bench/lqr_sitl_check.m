function T = lqr_sitl_check(pattern)
%LQR_SITL_CHECK Rate-loop health of the run_lqr_sitl.m flights, from the SITL DataFlash
%logs (RATE, 400 Hz) and the MATLAB physics logs:
%   - rate tracking error RMS (roll/pitch) while airborne
%   - rate-loop ringing: share of body-rate power in 6-40 Hz, and its peak frequency
%   - effective actuator lag: delay between roll motor command and roll acceleration in the
%     physics (checks that the extra MotorDelay was really applied)
%   T = lqr_sitl_check();
if nargin < 1, pattern = "run_*_p*"; end
here = fileparts(mfilename('fullpath'));
runs = dir(fullfile(fileparts(here), "sitl", pattern));
runs = runs([runs.isdir] & ~cellfun(@isempty, regexp({runs.name}, '^run_\d{8}_\d{6}_[ABC]_\w+_p\w+$', 'once')));
if isempty(runs), error('lqr_sitl_check:none', 'No run_lqr_sitl run folders match %s', pattern); end
rows = {};
for k = 1:numel(runs)
    d = fullfile(runs(k).folder, runs(k).name);
    b = dir(fullfile(d, "logs", "*.BIN")); if isempty(b), continue; end
    lo = ardupilotreader(fullfile(b(end).folder, b(end).name));
    M = readMessages(lo, MessageNames={'RATE'}); R = M.MsgData{1};
    t = seconds(R.Properties.RowTimes); a = t > 32 & t < 130;           % airborne part of the test
    fs = 1 / median(diff(t));
    v = struct();
    ax = ["R", "P"];
    for i = 1:2
        w = double(R.(ax(i))(a)); wd = double(R.(ax(i) + "Des")(a));
        v.err(i) = rms(wd - w);
        [pxx, f] = pwelch(detrend(w - wd), hann(1024), 512, 1024, fs);
        hi = f >= 6 & f <= 40; v.ring(i) = 100 * sum(pxx(hi)) / sum(pxx(f >= 0.5));
        [~, j] = max(pxx .* hi); v.fpk(i) = f(j);
    end
    % actuator lag in the physics: roll command (QuadX mix) -> roll acceleration
    S = load(fullfile(d, "physics_log.mat")); L = S.L;
    m = L.t > 32 & L.t < 130; dt = median(diff(L.t));
    u = L.pwm1(m) - L.pwm2(m) - L.pwm3(m) + L.pwm4(m);
    acc = gradient(L.p(m)) / dt;
    [c, lags] = xcorr(detrend(acc), detrend(u), round(0.1 / dt));
    [~, j] = max(abs(c)); lagMs = 1000 * lags(j) * dt;
    parts = split(string(runs(k).name), "_");
    rows(end+1, :) = {strjoin(parts(6:end), "_"), parts(4) + "_" + parts(5),v.err(1), v.err(2), v.ring(1), v.ring(2), v.fpk(1), v.fpk(2), lagMs}; %#ok<AGROW>
end
T = cell2table(rows, 'VariableNames', ["Plant", "Tune", "RollRateErr_degps", "PitchRateErr_degps", ...
    "RollRing6to40Hz_pct", "PitchRing6to40Hz_pct", "RollRingPeak_Hz", "PitchRingPeak_Hz", "ActuatorLag_ms"]);
T = sortrows(T, ["Plant", "Tune"]);
disp(T);
writetable(T, fullfile(here, "results", "lqr_sitl_check.csv"));
end
