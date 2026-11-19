function out = fc_tune_from_log(logFiles, opts)
%FC_TUNE_FROM_LOG  ArduPilot flight log(s) -> identified plant -> LQR-designed PID -> .param
%
%   out = fc_tune_from_log("C:\logs\2026-11-12 17-04-34.bin")
%   out = fc_tune_from_log(["a.bin", "b.bin"], SITL=true)       % several logs, then SITL check
%
%   1. Reads each log (UAV Toolbox ardupilotreader): RATE at 400 Hz, PARM, EV, CTUN.
%      The current gains and filters are taken from the log's own parameters.
%   2. Finds the flights (EV take-off -> landed, else throttle above idle) and identifies
%      roll/pitch/yaw:  G(s) = K e^{-tau s} / (s (tm s + 1))   (fc_identify_rate)
%   3. Grades each axis (good / weak / unusable). Roll or pitch unusable -> stops with the
%      reason. Yaw unusable -> yaw is left at its current values.
%   4. LQR -> PID design with delay-aware margins and Monte Carlo (lqr_pid_tuner).
%   5. Optional SITL check: flies the current tune and the LQR step-1 tune on the real
%      ArduCopter firmware with physics set to the identified plant (run_sitl_bench).
%
%   Output folder (default: next to the log, "<log name>_lqr_tune"): fc_lqr_tune.param,
%   summary.txt, gains.csv, quality.csv, figures.
%
%   Logging needed: 400 Hz rate logs (LOG_BITMASK with ATTITUDE_FAST + PID, e.g. 180223).
%   Best data: an ArduPilot SYSID flight (rate chirp per axis); normal flying with plenty
%   of stick input also works.

arguments
    logFiles string
    opts.OutDir (1,1) string = ""
    opts.Delay (1,1) string {mustBeMember(opts.Delay, ["replay", "identified", "component"])} = "replay"
    opts.PMmin (1,1) double = 50
    opts.GMmin (1,1) double = 6
    opts.MCmaxFail (1,1) double = 10
    opts.MaxIncrease (1,1) double = 1.5
    opts.MaxDecrease (1,1) double = 0.7
    opts.Plots (1,1) logical = true
    opts.SITL (1,1) logical = false
    opts.Report (1,1) logical = true        % PDF report (MATLAB Report Generator)
    opts.Progress = []                      % @(fraction, message)
end
prog = @(f, msg) []; if ~isempty(opts.Progress), prog = opts.Progress; end
logFiles = logFiles(:)';
for f = logFiles, if ~isfile(f), error('fc_tune:nofile', 'Log file not found: %s', f); end, end
warnings = strings(0);

% ---------------------------------------------------------------- 1. read logs
flights = struct('id', {}, 'rate_hz', {}, 'dur_s', {}, 'ratedata', {});
par = []; LI = {};
for i = 1:numel(logFiles)
    [~, n, e] = fileparts(logFiles(i));
    prog(0.02 + 0.06 * (i - 1) / numel(logFiles), "Reading " + n + e);
    try
        [fl, p, note] = readLog(logFiles(i));
    catch err
        fl = flights([]); p = []; note = "Could not read this file (" + err.message + ")";
    end
    flights = [flights, fl]; %#ok<AGROW>
    if ~isempty(fl), par = p; end                       % current tune = last USABLE log
    LI(end+1, :) = {string(n + e), ~isempty(fl), numel(fl), sum([fl.dur_s]), note}; %#ok<AGROW>
end
LT = cell2table(LI, 'VariableNames', ["File", "Used", "Flights", "Airborne_s", "Note"]);
if isempty(flights)
    error('fc_tune:noflight', 'None of the selected logs can be used:\n%s', ...
        strjoin("  " + LT.File + ": " + LT.Note, newline));
end
% results go next to the newest log that was actually USED
usedPaths = logFiles(LT.Used);
[lp, ln] = fileparts(usedPaths(end));
if opts.OutDir == "", opts.OutDir = fullfile(lp, ln + "_lqr_tune"); end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
cur = currentTune(par);
fprintf('\n%d flight segment(s), %.0f s of 400 Hz data. Current tune from the log:\n', numel(flights), sum([flights.dur_s]));
fprintf('  roll  P %.3f I %.3f D %.4f | pitch P %.3f I %.3f D %.4f | yaw P %.3f I %.3f D %.4f\n', ...
    cur.P(1), cur.I(1), cur.D(1), cur.P(2), cur.I(2), cur.D(2), cur.P(3), cur.I(3), cur.D(3));

% ---------------------------------------------------------------- 2. identify
prog(0.08, "Identifying the plant from the flight data");
axn = ["roll", "pitch", "yaw"]; ID = struct(); Q = {};
for a = 1:3
    w = []; u = []; z = []; dt = []; seg = [];
    for k = 1:numel(flights)
        r = flights(k).ratedata;
        w = [w; r.w(:, a)]; u = [u; r.u(:, a)]; z = [z; r.wd(:, a)]; dt = [dt; r.dt]; seg = [seg; k * ones(size(r.dt))]; %#ok<AGROW>
    end
    id = fc_identify_rate(w, u, z, dt, seg);
    [grade, why] = gradeAxis(id);
    ID.(axn(a)) = id;
    Q(end+1, :) = {axn(a), grade, id.Krad, 1000 * id.tm, 1000 * id.tau, id.fit, id.fitVal, id.fband(1), id.fband(2), id.nseg, why}; %#ok<AGROW>
    fprintf('  %-5s %-8s K %6.0f  motor lag %5.1f ms  delay %5.1f ms  gain fit %5.1f %%  coherence %4.2f  band %4.1f-%4.1f Hz  %s\n', ...
        axn(a), upper(grade), id.Krad, 1000 * id.tm, 1000 * id.tau, id.fit, id.fitVal, id.fband, why);
end
QT = cell2table(Q, 'VariableNames', ["Axis", "Grade", "K_radps2", "MotorLag_ms", "PhaseDelay_ms", "GainFit_pct", "Coherence", "BandLo_Hz", "BandHi_Hz", "Segments", "Note"]);
for a = 1:2
    if QT.Grade(a) == "unusable"
        error('fc_tune:noid', ['There is not enough %s movement in these logs to learn how the drone responds.\n' ...
            'Fly again with more %s stick input (or an ArduPilot SYSID flight on %s) and add that log.'], axn(a), axn(a), axn(a));
    end
end
keep = strings(0);
if QT.Grade(3) == "unusable"
    keep = "yaw";
    ID.yaw = fallbackYaw(ID);
end

% ---------------------------------------------------------------- 3. design
F = struct('ID', ID, 'flights', flights);
R = lqr_pid_tuner(F=F, Current=cur, OutDir=opts.OutDir, KeepAxes=keep, Delay=opts.Delay, PMmin=opts.PMmin, ...
    GMmin=opts.GMmin, MCmaxFail=opts.MCmaxFail, MaxIncrease=opts.MaxIncrease, MaxDecrease=opts.MaxDecrease, DFilterHz=cur.FLTD, ...
    TargetFilterHz=cur.FLTT, Plots=opts.Plots, Progress=@(f, m) prog(0.1 + 0.8 * f, m), FigureVisible="off", ...
    CurrentLabel="current (from log)", CompareTuneC=false);
% the tuner names the log's own tune "as flown 12 Nov": rename for this log
R.gains.Case(R.gains.Case == "as flown 12 Nov") = "current (from log)";
R.replay.Case(R.replay.Case == "as flown 12 Nov") = "current (from log)";
QT.ReplayFit_pct = nan(3, 1); QT.Confidence = strings(3, 1); QT.BandFit_pct = nan(3, 1); QT.HoldoutFit_pct = nan(3, 1);
QT.ModelK = nan(3, 1); QT.ModelMotorLag_ms = nan(3, 1); QT.ModelDelay_ms = nan(3, 1);
for a = 1:3
    QT.ReplayFit_pct(a) = R.replay.ModelFitToMeasured_pct(R.replay.Axis == axn(a) & R.replay.Case == "current (from log)");
    fi = R.(axn(a)).fitInfo; mm = R.(axn(a)).model;
    QT.BandFit_pct(a) = fi.fitAll; QT.HoldoutFit_pct(a) = fi.fitHold;
    QT.ModelK(a) = mm.K; QT.ModelMotorLag_ms(a) = 1000 * mm.tm; QT.ModelDelay_ms(a) = 1000 * mm.tau;
end
% plain-language result per axis
AX = struct([]);
for a = 1:3
    g = @(c) R.gains(R.gains.Axis == axn(a) & R.gains.Case == c, :);
    c0 = g("current (from log)"); c1 = g("LQR step 1"); ct = g("LQR target");
    kept = any(keep == axn(a));
    conf = confidence(QT.Grade(a), QT.ReplayFit_pct(a), kept);
    QT.Confidence(a) = conf;
    r = R.(axn(a));
    if kept, nw = struct('P', cur.P(a), 'I', cur.I(a), 'D', cur.D(a)); angP = cur.AngP(a); else, nw = r.step1; angP = r.angP; end
    A = struct('Axis', axn(a), 'Confidence', conf, 'Kept', kept, ...
        'Cur', struct('P', cur.P(a), 'I', cur.I(a), 'D', cur.D(a), 'AngP', cur.AngP(a)), ...
        'New', struct('P', nw.P, 'I', nw.I, 'D', nw.D, 'AngP', angP), ...
        'Final', struct('P', ct.P, 'I', ct.I, 'D', ct.D), 'IsFinal', isequal([c1.P c1.I c1.D], [ct.P ct.I ct.D]), ...
        'PM', [c0.PM_deg c1.PM_deg], 'GM', [c0.GM_dB c1.GM_dB], 'Overshoot', [c0.Overshoot_pct c1.Overshoot_pct], ...
        'Rise_ms', 1000 * [c0.Rise_s c1.Rise_s], 'MCfail', [c0.MonteCarloFailPct c1.MonteCarloFailPct], ...
        'Text', axisText(axn(a), c0, c1, kept, conf, QT(a, :)));
    AX = [AX, A]; %#ok<AGROW>
    if kept
        warnings(end+1) = sprintf("%s: not enough %s movement in the logs to tune it, so %s is left as it is.", ...
            cap(axn(a)), axn(a), axn(a)); %#ok<AGROW>
    elseif conf == "Low"
        warnings(end+1) = sprintf("%s: low confidence. The model matches only %.0f %% of your flight, so the new %s gains may not feel as predicted. Test gently.", ...
            cap(axn(a)), QT.ReplayFit_pct(a), axn(a)); %#ok<AGROW>
    elseif conf == "Medium"
        warnings(end+1) = sprintf("%s: medium confidence. The model matches %.0f %% of your flight; the new gains are safe on paper but check how %s feels.", ...
            cap(axn(a)), QT.ReplayFit_pct(a), axn(a)); %#ok<AGROW>
    end
end
for i = find(~LT.Used)'
    warnings(end+1) = LT.File(i) + " was not used: " + LT.Note(i); %#ok<AGROW>
end
writetable(R.gains, fullfile(opts.OutDir, "gains.csv"));
writetable(R.replay, fullfile(opts.OutDir, "replay.csv"));
writetable(QT, fullfile(opts.OutDir, "quality.csv"));

% ---------------------------------------------------------------- 4. optional SITL
ST = table();
if opts.SITL
    prog(0.92, "SITL check: flying current and LQR tunes on ArduCopter firmware (a few minutes)");
    try
        ST = sitlCheck(R, ID, cur, logFiles(end));
        writetable(ST, fullfile(opts.OutDir, "sitl_check.csv"));
    catch e
        warnings(end+1) = "The simulator test could not run: " + e.message;
    end
end

% ---------------------------------------------------------------- 5. summary
if any([AX(1:2).Confidence] == "Low"), verdict = "caution";
elseif any([AX.Confidence] == "Low" | [AX.Confidence] == "Medium"), verdict = "ok-check";
else, verdict = "ok"; end
out = struct('outDir', opts.OutDir, 'paramFile', fullfile(opts.OutDir, "fc_lqr_tune.param"), ...
    'gains', R.gains, 'quality', QT, 'replay', R.replay, 'sitl', ST, 'warnings', warnings, 'current', cur, ...
    'logs', logFiles, 'logInfo', LT, 'axes', AX, 'verdict', verdict, ...
    'flights', numel(flights), 'airborne_s', sum([flights.dur_s]), 'R', R);
out.figures = string(fullfile(opts.OutDir, ["1_design_space.png", "2_loops.png", "3_margins.png", "4_replay.png", "5_delay_fit.png"]));
out.figures = out.figures(isfile(out.figures));
out.summary = summaryText(out, keep);
fid = fopen(fullfile(opts.OutDir, "summary.txt"), 'w'); fprintf(fid, '%s', out.summary); fclose(fid);
fprintf('\n%s', out.summary);
out.reportFile = "";
if opts.Report && opts.Plots
    prog(0.97, "Writing the PDF report");
    try
        out.reportFile = fc_tune_report(out);
        fprintf('PDF report: %s\n', out.reportFile);
    catch e
        out.warnings(end+1) = "The PDF report could not be written: " + e.message;
        fprintf(2, 'PDF report failed: %s\n', e.message);
    end
end
prog(1, "Done");
end

%% =====================================================================
function [fl, par, note] = readLog(f)
NOLOG = "This log has no rate data, so it cannot be used for tuning. Set LOG_BITMASK = 180223 (adds 400 Hz rate logging) and fly again.";
fl = struct('id', {}, 'rate_hz', {}, 'dur_s', {}, 'ratedata', {}); par = []; viaThrottle = false; note = "";
[~, nm] = fileparts(f);
lo = ardupilotreader(f);
av = string(lo.AvailableMessages.MsgName);
if ~any(av == "RATE"), note = NOLOG; return; end
want = intersect(["RATE", "PARM", "EV", "CTUN"], av);
T = readMessages(lo, MessageNames=cellstr(want(:)'));
G = @(n) getm(T, n);
tt = @(x) seconds(x.Properties.RowTimes);
par = G("PARM");
rt = G("RATE"); trt = tt(rt);
fs = 1 / median(diff(trt));
if fs < 300
    note = sprintf("Rate data was logged at only %.0f Hz; the tuner needs 400 Hz. Set LOG_BITMASK = 180223 and fly again.", fs);
    return
end
% flights: EV 28 = took off, 18 = landed; else throttle out above idle
spans = zeros(0, 2);
ev = G("EV");
if ~isempty(ev)
    te = tt(ev); id = double(ev.Id); to = te(id == 28); ld = te(id == 18);
    for k = 1:numel(to)
        e = ld(find(ld > to(k), 1)); if isempty(e), e = trt(end); end
        spans(end+1, :) = [to(k) e]; %#ok<AGROW>
    end
end
if isempty(spans)
    ct = G("CTUN");
    if ~isempty(ct)
        tc = tt(ct); air = double(ct.ThO) > 0.2;
        d = diff([0; air; 0]); s0 = find(d == 1); s1 = find(d == -1) - 1;
        for k = 1:numel(s0), spans(end+1, :) = [tc(s0(k)) tc(s1(k))]; end %#ok<AGROW>
        viaThrottle = ~isempty(spans);
    end
end
for k = 1:size(spans, 1)
    s = spans(k, :) + [3 -1];                    % skip the take-off transient and touchdown
    if diff(s) < 15, continue; end
    a = trt >= s(1) & trt <= s(2);
    r.id = string(nm) + sprintf(" F%d", k);
    r.rate_hz = fs; r.dur_s = diff(s);
    r.ratedata = struct('w', deg2rad(double([rt.R(a) rt.P(a) rt.Y(a)])), 'u', double([rt.ROut(a) rt.POut(a) rt.YOut(a)]), ...
        'wd', deg2rad(double([rt.RDes(a) rt.PDes(a) rt.YDes(a)])), 'dt', [diff(trt(a)); 1 / fs]);
    fl(end+1) = r; %#ok<AGROW>
end
if isempty(fl)
    note = "No flight of 15 seconds or longer found in this log.";
else
    note = sprintf("OK: %d flight(s), %.0f s in the air", numel(fl), sum([fl.dur_s]));
    if viaThrottle, note = note + " (found from throttle; no take-off events)"; end
end
fprintf('%s: %s\n', nm, note);
end

function x = getm(T, n)
x = []; k = find(string(T.MsgName) == n, 1);
if ~isempty(k), x = T.MsgData{k}; end
end

function cur = currentTune(par)
pv = @(n, d) paramLast(par, n, d);
ax = ["RLL", "PIT", "YAW"];
for a = 1:3
    cur.P(a) = pv("ATC_RAT_" + ax(a) + "_P", NaN); cur.I(a) = pv("ATC_RAT_" + ax(a) + "_I", NaN);
    cur.D(a) = pv("ATC_RAT_" + ax(a) + "_D", 0);   cur.AngP(a) = pv("ATC_ANG_" + ax(a) + "_P", 4.5);
end
if any(isnan([cur.P cur.I])), error('fc_tune:noparam', 'The log has no ATC_RAT_* parameters (PARM messages missing).'); end
cur.FLTD = pv("ATC_RAT_RLL_FLTD", 20); if cur.FLTD <= 0, cur.FLTD = 20; end
cur.FLTT = pv("ATC_RAT_RLL_FLTT", 20); if cur.FLTT <= 0, cur.FLTT = 20; end
cur.GyroFilter = pv("INS_GYRO_FILTER", NaN);
end

function v = paramLast(P, n, d)
v = d; if isempty(P), return; end
k = find(string(P.Name) == n, 1, 'last'); if ~isempty(k), v = double(P.Value(k)); end
end

function [g, why] = gradeAxis(id)
if isnan(id.Krad), g = "unusable"; why = "too little excitation: no coherent band"; return; end
bw = diff(id.fband);
if id.fit >= 90 && id.fitVal >= 0.6 && bw >= 3
    g = "good"; why = "";
else
    g = "weak"; why = sprintf("gain fit %.0f %%, coherence %.2f, band %.1f Hz wide", id.fit, id.fitVal, bw);
end
end

function c = confidence(grade, replayFit, kept)
% High: clean identification and the model reproduces the flight well.
if kept, c = "n/a";
elseif grade == "good" && replayFit >= 50, c = "High";
elseif (grade == "good" && replayFit >= 25) || (grade == "weak" && replayFit >= 50), c = "Medium";
else, c = "Low";
end
end

function s = cap(x)
s = upper(extractBefore(x, 2)) + extractAfter(x, 1);
end

function t = axisText(ax, c0, c1, kept, conf, q) %#ok<INUSD>
% one or two plain sentences on what the change does
if kept
    t = "Not enough " + ax + " movement in the logs to tune it. Kept as it is.";
    return
end
t = "";
if c1.PM_deg > c0.PM_deg + 5
    t = "Further from oscillating.";
elseif c1.PM_deg < c0.PM_deg - 5
    t = "A little closer to oscillating, still within the safe limit.";
end
if c1.Overshoot_pct < c0.Overshoot_pct - 3
    t = t + sprintf(" Less overshoot/wobble after a stick input (%.0f %% -> %.0f %%).", c0.Overshoot_pct, c1.Overshoot_pct);
end
if c1.Rise_s > 1.15 * c0.Rise_s
    t = t + sprintf(" Response a little softer (%.0f -> %.0f ms).", 1000 * c0.Rise_s, 1000 * c1.Rise_s);
elseif c1.Rise_s < 0.87 * c0.Rise_s
    t = t + sprintf(" Response quicker (%.0f -> %.0f ms).", 1000 * c0.Rise_s, 1000 * c1.Rise_s);
end
t = strtrim(t);
if t == "", t = "Little change from the current tune."; end
end

function v = ifelse(c, a, b)
if c, v = a; else, v = b; end
end

function y = fallbackYaw(ID)
% yaw stand-in so the tuner can run; yaw is kept at its current values anyway
y = ID.roll; y.Krad = 0.1 * ID.roll.Krad; y.tm = 0.01; y.tau = 0; y.fband = [1 2];
end

function ST = sitlCheck(R, ID, cur, logFile)
P = quad_params('auth_roll', ID.roll.Krad, 'auth_pitch', ID.pitch.Krad, 'auth_yaw', max(ID.yaw.Krad, 10), ...
    'tau_m', mean([ID.roll.tm ID.pitch.tm]));
env = {'VibAccel', [3.6 2.4 13], 'VibGyro', deg2rad([4 4 2]), 'GyroNoise', deg2rad(0.8), 'Gusts', 1.8, 'Wind', [3 1.5 0], 'Seed', 11};
ax = ["RLL", "PIT", "YAW"]; axn = ["roll", "pitch", "yaw"];
cases = {"current", cur; "LQR step 1", []};
rows = {};
for c = 1:2
    ov = {};
    for a = 1:3
        if c == 1, g = struct('P', cur.P(a), 'I', cur.I(a), 'D', cur.D(a)); else, g = R.(axn(a)).step1; end
        ov = [ov; {"ATC_RAT_" + ax(a) + "_P", g.P; "ATC_RAT_" + ax(a) + "_I", g.I; "ATC_RAT_" + ax(a) + "_D", g.D}]; %#ok<AGROW>
    end
    lab = ["A_current_plog", "C_LQR_plog"];
    o = run_sitl_bench('Label', lab(c), 'LogFile', logFile, 'ParamOverrides', ov, 'Params', P, env{:});
    A = o.analysis;
    rows(end+1, :) = {cases{c, 1}, A.stepTrackRms, A.stepOvershoot, A.yaw.trackRms, A.altHoldRms, A.loiter.rms, ...
        A.outNoise(1), A.outNoise(2), A.outNoise(3), string(o.runDir)}; %#ok<AGROW>
    pause(2);
end
ST = cell2table(rows, 'VariableNames', ["Tune", "AttTrackRMS_deg", "StepOvershoot_pct", "YawTrackRMS_degps", ...
    "AltHoldRMS_m", "LoiterRMS_m", "MotorNoiseRoll_pct", "MotorNoisePitch_pct", "MotorNoiseYaw_pct", "RunDir"]);
disp(ST);
end

function s = summaryText(out, keep)
G = out.gains; Q = out.quality; axn = ["roll", "pitch", "yaw"]; nm = ["RLL", "PIT", "YAW"];
L = strings(0);
L(end+1) = "Your FC LQR PID tuner - " + string(datetime('now', 'Format', 'yyyy-MM-dd HH:mm'));
L(end+1) = "Logs: " + strjoin(out.logs, ", ");
L(end+1) = sprintf("%d flight segment(s), %.0f s of 400 Hz data", out.flights, out.airborne_s);
L(end+1) = "";
L(end+1) = "LOGS";
for i = 1:height(out.logInfo)
    L(end+1) = sprintf("  %-28s %s", out.logInfo.File(i), out.logInfo.Note(i)); %#ok<AGROW>
end
L(end+1) = "";
L(end+1) = "DATA QUALITY";
for a = 1:3
    L(end+1) = sprintf("  %-5s %-8s confidence %-6s gain fit %5.1f %%, coherence %.2f, replay fit %5.1f %%  %s", axn(a), upper(Q.Grade(a)), ...
        Q.Confidence(a), Q.GainFit_pct(a), Q.Coherence(a), Q.ReplayFit_pct(a), Q.Note(a)); %#ok<AGROW>
end
L(end+1) = "";
L(end+1) = "RATE LOOP (design delay, margins, Monte-Carlo fail %)";
for a = 1:3
    for c = ["current (from log)", "LQR step 1", "LQR target"]
        r = G(G.Axis == axn(a) & G.Case == c, :);
        L(end+1) = sprintf("  %-5s %-19s P %.3f I %.3f D %.4f | %5.1f Hz  PM %5.1f deg  GM %5.1f dB | MC fail %3.0f %% | overshoot %3.0f %%", ...
            axn(a), c, r.P, r.I, r.D, r.Crossover_Hz, r.PM_deg, r.GM_dB, r.MonteCarloFailPct, r.Overshoot_pct); %#ok<AGROW>
    end
end
L(end+1) = "";
L(end+1) = "PARAMETERS TO LOAD (step 1): " + out.paramFile;
for a = 1:3
    r = out.R.(axn(a)); g = r.step1; ap = r.angP;
    if any(keep == axn(a)), g = r.current; ap = out.current.AngP(a); end
    L(end+1) = sprintf("  ATC_RAT_%s_P %.3f   ATC_RAT_%s_I %.3f   ATC_RAT_%s_D %.4f   ATC_ANG_%s_P %.2f", nm(a), g.P, nm(a), g.I, nm(a), g.D, nm(a), ap); %#ok<AGROW>
end
if ~isempty(out.sitl) && height(out.sitl) > 0
    L(end+1) = "";
    L(end+1) = "SITL CHECK (ArduCopter firmware, identified physics)";
    for k = 1:height(out.sitl)
        S = out.sitl(k, :);
        L(end+1) = sprintf("  %-11s attitude RMS %.2f deg, overshoot %.1f %%, yaw RMS %.1f deg/s, motor noise %.2f/%.2f/%.2f %%", ...
            S.Tune, S.AttTrackRMS_deg, S.StepOvershoot_pct, S.YawTrackRMS_degps, S.MotorNoiseRoll_pct, S.MotorNoisePitch_pct, S.MotorNoiseYaw_pct); %#ok<AGROW>
    end
end
if ~isempty(out.warnings)
    L(end+1) = "";
    L(end+1) = "WARNINGS";
    for w = out.warnings, L(end+1) = "  - " + w; end %#ok<AGROW>
end
L(end+1) = "";
L(end+1) = "Before flying: back up your parameters, load the .param, hover low (1-2 m) in STABILIZE/ALT_HOLD";
L(end+1) = "for 1-2 minutes, land, and run this tool again on the new log.";
s = strjoin(L, newline) + newline;
end
