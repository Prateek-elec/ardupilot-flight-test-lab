function A = sitl_analysis(binFile, physics, opts)
%SITL_ANALYSIS Analyse a SITL flight of ArduCopter on the MATLAB physics.
%
%   1. Controller replica check: feeds the rate target and gyro that the REAL
%      ArduPilot firmware logged (400 Hz) into the MATLAB AC_PID replica and
%      compares the controller output sample by sample.
%   2. Flight performance from the scripted manoeuvres: attitude step
%      response, yaw, climb, position hold in wind, RTL.
%   3. Physics consistency: what ArduPilot's EKF saw vs the MATLAB truth.
%
%   A = sitl_analysis("...\logs\00000001.BIN", out.physics);

arguments
    binFile (1,1) string
    physics = []
    opts.OutDir (1,1) string = ""
end
if opts.OutDir == "", opts.OutDir = fullfile(fileparts(binFile), "..", "analysis"); end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end

lo = ardupilotreader(binFile);
want = {'RATE','ATT','RCIN','CTUN','MODE','PARM','PIDR','PIDP','PIDY','POS','PSCN','PSCE','XKF1','VIBE'};
avail = intersect(want, cellstr(lo.AvailableMessages.MsgName)); avail = avail(:)';
T = readMessages(lo, MessageNames=avail);
G = @(n) getm(T, n);
rate = G("RATE"); att = G("ATT"); rcin = G("RCIN"); ctun = G("CTUN"); parm = G("PARM"); md = G("MODE");
ts = @(x) seconds(x.Properties.RowTimes);
fprintf('\n=== SITL log %s\n', binFile);
fprintf('RATE logged at %.0f Hz, %.0f s\n', 1/median(diff(ts(rate))), ts(rate(end,:)) - ts(rate(1,:)));

pv = @(n, d) paramLast(parm, n, d);
key = ["RLL","PIT","YAW"]; axn = ["Roll","Pitch","Yaw"];
pidMsg = ["PIDR","PIDP","PIDY"];
desF = ["RDes","PDes","YDes"]; actF = ["R","P","Y"]; outF = ["ROut","POut","YOut"];

%% 1. controller replica vs real ArduPilot
fprintf('\n--- Controller replica vs real ArduPilot firmware (same inputs, 400 Hz)\n');
air = airborne(ctun, rate);
if ~any(air)
    error("sitl_analysis:notAirborne", "The vehicle never got airborne in %s - nothing to analyse. Check the mission output (arming, thrust).", binFile);
end
for a = 1:3
    c = struct('P', pv("ATC_RAT_"+key(a)+"_P",0.135), 'I', pv("ATC_RAT_"+key(a)+"_I",0.135), ...
        'D', pv("ATC_RAT_"+key(a)+"_D",0.0036), 'FF', pv("ATC_RAT_"+key(a)+"_FF",0), ...
        'IMAX', pv("ATC_RAT_"+key(a)+"_IMAX",0.5), 'FLTT', pv("ATC_RAT_"+key(a)+"_FLTT",20), ...
        'FLTE', pv("ATC_RAT_"+key(a)+"_FLTE",0), 'FLTD', pv("ATC_RAT_"+key(a)+"_FLTD",20));
    t = ts(rate); sel = air;
    tgt = deg2rad(double(rate.(desF(a))(sel))); meas = deg2rad(double(rate.(actF(a))(sel)));
    outLog = double(rate.(outF(a))(sel)); tt = t(sel);
    % replica runs on the same samples; I-term initialised from the log
    pid = G(pidMsg(a));
    I0 = 0;
    if ~isempty(pid), I0 = interp1(ts(pid), double(pid.I), tt(1), 'nearest', 'extrap'); end
    dtv = diff([tt(1) - 1/400; tt]);
    [o, parts] = pidReplica(c, tgt, meas, dtv, I0);
    seg = [0; find(diff(tt) > 0.05); numel(tt)];      % restart at gaps (separate flights)
    if numel(seg) > 2
        for s = 1:numel(seg)-1
            k = seg(s)+1:seg(s+1);
            I0s = I0; if ~isempty(pid), I0s = interp1(ts(pid), double(pid.I), tt(k(1)), 'nearest', 'extrap'); end
            [o(k), pk] = pidReplica(c, tgt(k), meas(k), dtv(k), I0s);
            parts.P(k) = pk.P; parts.I(k) = pk.I; parts.D(k) = pk.D;
        end
    end
    A.ctrl(a).fit = fitPct(outLog, o);
    A.ctrl(a).rmse = rms(o - outLog);
    A.ctrl(a).t = tt; A.ctrl(a).log = outLog; A.ctrl(a).rep = o;
    msgPID = "";
    if ~isempty(pid)
        tp = ts(pid); kk = tp >= tt(1) & tp <= tt(end);
        pI = interp1(tt, parts.P, tp(kk), 'nearest'); pD = interp1(tt, parts.D, tp(kk), 'nearest');
        A.ctrl(a).fitP = fitPct(double(pid.P(kk)), pI);
        A.ctrl(a).fitD = fitPct(double(pid.D(kk)), pD);
        msgPID = sprintf(' | P-term fit %.1f %%, D-term fit %.1f %%', A.ctrl(a).fitP, A.ctrl(a).fitD);
    end
    fprintf('  %-5s output fit %6.2f %%  (RMS diff %.4f of full scale)%s\n', axn(a), A.ctrl(a).fit, A.ctrl(a).rmse, msgPID);
end

%% 2. manoeuvre performance
fprintf('\n--- Flight performance (ArduPilot firmware on the validated physics)\n');
tA = ts(att); tR = ts(rcin);
steps = detectSteps(tR, double(rcin.C1), double(rcin.C2), double(rcin.C4), double(rcin.C3));
A.steps = table();
for s = 1:numel(steps)
    st = steps(s);
    w = tA >= st.t0 - 0.5 & tA <= st.t0 + st.dur + 3;
    if st.axis <= 2
        desNames = ["DesRoll","DesPitch"]; actNames = ["Roll","Pitch"];
        des = double(att.(desNames(st.axis))(w)); act = double(att.(actNames(st.axis))(w));
        tt = tA(w) - st.t0;
        pk = max(abs(des)); sgn = sign(des(find(abs(des) == pk, 1)));
        m = stepMetricsTrack(tt, sgn*des, sgn*act, st.dur);
        A.steps = [A.steps; table(string(st.name), m.trackRms, m.lagMs, m.overshoot, m.settleMs, pk, ...
            'VariableNames', ["Step","TrackRMS_deg","Lag_ms","Overshoot_pct","Settle_ms","PeakAngle_deg"])]; %#ok<AGROW>
        fprintf('  %-14s peak %5.1f deg | tracking RMS %5.2f deg | lag %4.0f ms | overshoot %5.1f %% | settle %5.0f ms\n', ...
            st.name, pk, m.trackRms, m.lagMs, m.overshoot, m.settleMs);
        S(s).t = tt; S(s).des = des; S(s).act = act; S(s).name = st.name; %#ok<AGROW>
    end
end
% yaw: rate tracking during the yaw step
ys = steps([steps.axis] == 4);
if ~isempty(ys)
    tRt = ts(rate); w = tRt >= ys(1).t0 & tRt <= ys(1).t0 + ys(1).dur;
    yd = double(rate.YDes(w)); ya = double(rate.Y(w));
    A.yaw.rateCmd = max(abs(yd)); A.yaw.rateAch = max(abs(ya)); A.yaw.trackRms = rms(yd - ya);
    tw = tRt(w) - ys(1).t0; i90 = find(abs(ya) >= 0.9*A.yaw.rateCmd, 1);
    if isempty(i90), A.yaw.rise90 = NaN; else, A.yaw.rise90 = tw(i90); end
    A.yaw.t = tw; A.yaw.des = yd; A.yaw.act = ya;
    fprintf('  YAW step       commanded %5.1f deg/s, achieved %5.1f deg/s, 90%% rise %.2f s, rate tracking RMS %.1f deg/s\n', ...
        A.yaw.rateCmd, A.yaw.rateAch, A.yaw.rise90, A.yaw.trackRms);
end
% climb
cs = steps([steps.axis] == 3);
tC = ts(ctun);
if ~isempty(cs)
    w = tC >= cs(1).t0 & tC <= cs(1).t0 + cs(1).dur + 5;
    crt = double(ctun.CRt(w)); if max(abs(crt)) > 20, crt = crt/100; end
    dcr = double(ctun.DCRt(w)); if max(abs(dcr)) > 20, dcr = dcr/100; end
    A.climb.desired = max(dcr); A.climb.achieved = max(crt);
    altw = double(ctun.Alt(w)); A.climb.gain = altw(end) - altw(1);
    after = tC >= cs(1).t0 + cs(1).dur + 1 & tC <= cs(1).t0 + cs(1).dur + 5;
    A.climb.holdErr = rms(double(ctun.DAlt(after)) - double(ctun.Alt(after)));
    fprintf('  CLIMB          desired %.2f m/s, achieved %.2f m/s, gained %.2f m, altitude hold error after %.3f m RMS\n', ...
        A.climb.desired, A.climb.achieved, A.climb.gain, A.climb.holdErr);
end
% altitude hold during ALT_HOLD/LOITER segments
inHold = modeMask(md, tC, [2 5]) & double(ctun.ThO) > 0.2;
A.altHoldRms = rms(double(ctun.DAlt(inHold)) - double(ctun.Alt(inHold)));
fprintf('  ALT HOLD       altitude error %.3f m RMS (in ALT_HOLD/LOITER)\n', A.altHoldRms);
% loiter position hold in wind
psn = G("PSCN"); pse = G("PSCE");
if ~isempty(psn) && ~isempty(pse)
    tp = ts(psn); lw = modeMask(md, tp, 5);
    en = double(psn.TPN(lw)) - double(psn.PN(lw)); ee = interp1(ts(pse), double(pse.TPE) - double(pse.PE), tp(lw));
    if max(abs([en; ee])) > 50, en = en/100; ee = ee/100; end   % cm -> m on older formats
    A.loiter.rms = rms(hypot(en, ee)); A.loiter.max = max(hypot(en, ee));
    fprintf('  LOITER (wind)  position error %.2f m RMS, %.2f m max\n', A.loiter.rms, A.loiter.max);
end
% motor-command noise: high-frequency part (> ~20 Hz) of each rate-controller output
tRr = ts(rate); hov = interp1(tC, double(inHold), tRr, 'nearest', 0) > 0.5;
for a = 1:3
    x = double(rate.(outF(a))); hp = x - movmean(x, 20);   % 20 samples at 400 Hz = 50 ms
    A.outNoise(a) = 100 * rms(hp(hov));
end
fprintf('  MOTOR NOISE    controller output >20 Hz: roll %.2f %%, pitch %.2f %%, yaw %.2f %% RMS\n', A.outNoise);
% attitude steps summary for comparisons
if ~isempty(A.steps)
    A.stepSettleMs = mean(A.steps.Settle_ms); A.stepOvershoot = mean(A.steps.Overshoot_pct);
    A.stepTrackRms = mean(A.steps.TrackRMS_deg);
end
A.S = S;
A.hoverThr = median(double(ctun.ThO(inHold)));
fprintf('  Hover throttle %.3f (real flight learned MOT_THST_HOVER %.3f)\n', A.hoverThr, pv("MOT_THST_HOVER", NaN));

%% 3. EKF vs physics truth
if ~isempty(physics)
    x1 = G("XKF1");
    if ~isempty(x1)
        tk = ts(x1);
        tp = physics.t;
        off = estimateOffset(tk, double(x1.Roll), tp, rad2deg(physics.roll));
        r = interp1(tp + off, rad2deg(physics.roll), tk); p = interp1(tp + off, rad2deg(physics.pitch), tk);
        k = ~isnan(r);
        A.ekf.rollErr = rms(double(x1.Roll(k)) - r(k)); A.ekf.pitchErr = rms(double(x1.Pitch(k)) - p(k));
        fprintf('\n--- EKF attitude vs MATLAB truth: roll %.2f deg RMS, pitch %.2f deg RMS\n', A.ekf.rollErr, A.ekf.pitchErr);
    end
end

%% plots
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1400 900]);
tl = tiledlayout(3, 2, 'TileSpacing', 'compact');
title(tl, 'ArduCopter SITL on MATLAB physics: controller replica check and step responses');
for a = 1:3
    nexttile; c = A.ctrl(a); w = c.t >= c.t(1) + 30 & c.t <= c.t(1) + 34;
    plot(c.t(w), c.log(w), 'k-', 'LineWidth', 1.8); hold on; plot(c.t(w), c.rep(w), '--', 'LineWidth', 1.4);
    title(sprintf('%s rate-controller output: ArduPilot vs MATLAB replica (fit %.1f %%)', axn(a), c.fit)); legend('ArduPilot (SITL log)', 'MATLAB replica'); grid on
end
valid = find(arrayfun(@(q) ~isempty(q.t), S));     % attitude steps only (not the takeoff climb)
for s = valid(1:min(3, numel(valid)))
    nexttile; plot(S(s).t, S(s).des, 'k--', 'LineWidth', 1.4); hold on; plot(S(s).t, S(s).act, 'LineWidth', 1.6);
    title(sprintf('%s step: target vs actual', S(s).name)); xlabel('s from stick step'); ylabel('deg'); legend('target', 'actual'); grid on
end
exportgraphics(f, fullfile(opts.OutDir, "sitl_analysis.png"), 'Resolution', 110);
if strcmp(vis, 'off'), delete(f); end
A.figure = fullfile(opts.OutDir, "sitl_analysis.png");
end

%% =====================================================================
function x = getm(T, n)
k = find(T.MsgName == n, 1); if isempty(k), x = []; else, x = T.MsgData{k}; end
end

function v = paramLast(parm, name, def)
i = find(string(parm.Name) == name, 1, 'last');
if isempty(i), v = def; else, v = double(parm.Value(i)); end
end

function sel = airborne(ctun, rate)
tc = seconds(ctun.Properties.RowTimes); tr = seconds(rate.Properties.RowTimes);
a = interp1(tc, double(double(ctun.ThO) > 0.2 & double(ctun.Alt) > 0.5), tr, 'nearest', 0);
sel = a > 0.5;
end

function [o, parts] = pidReplica(c, tgt, meas, dt, I0)
% ArduPilot AC_PID::update_all (target/error/derivative filters, I limit)
n = numel(tgt); o = zeros(n,1); parts.P = o; parts.I = o; parts.D = o;
tf = tgt(1); ef = tgt(1) - meas(1); efPrev = ef; dF = 0; I = I0;
for k = 1:n
    h = dt(k); aT = lpa(c.FLTT, h); aE = lpa(c.FLTE, h); aD = lpa(c.FLTD, h);
    tf = tf + aT*(tgt(k) - tf);
    ef = ef + aE*((tf - meas(k)) - ef);
    if h > 0, dF = dF + aD*((ef - efPrev)/h - dF); end
    efPrev = ef;
    I = max(min(I + c.I*ef*h, c.IMAX), -c.IMAX);
    parts.P(k) = c.P*ef; parts.I(k) = I; parts.D(k) = c.D*dF;
    o(k) = parts.P(k) + I + parts.D(k) + c.FF*tf;
end
end

function a = lpa(fc, dt)
if fc <= 0, a = 1; else, a = dt/(dt + 1/(2*pi*fc)); end
end

function f = fitPct(y, yh)
k = ~isnan(y) & ~isnan(yh);
f = 100*(1 - norm(y(k) - yh(k)) / max(norm(y(k) - mean(y(k))), eps));
end

function steps = detectSteps(t, c1, c2, c4, c3)
steps = struct('t0', {}, 'dur', {}, 'axis', {}, 'name', {});
defs = {c1, 1, "ROLL"; c2, 2, "PITCH"; c3, 3, "CLIMB"; c4, 4, "YAW"};
for d = 1:size(defs, 1)
    x = defs{d,1}; on = abs(x - 1500) > 150 & x > 900;
    e = diff([0; on; 0]); s = find(e == 1); f = find(e == -1) - 1;
    for i = 1:numel(s)
        dir = sign(x(s(i)) - 1500);
        nm = defs{d,3};
        if d == 1, nm = nm + ternaryStr(dir > 0, " right", " left"); end
        if d == 2, nm = nm + ternaryStr(dir < 0, " forward", " back"); end
        steps(end+1) = struct('t0', t(s(i)), 'dur', t(f(i)) - t(s(i)), 'axis', defs{d,2}, 'name', nm); %#ok<AGROW>
    end
end
[~, o] = sort([steps.t0]); steps = steps(o);
end

function s = ternaryStr(c, a, b)
if c, s = a; else, s = b; end
end

function m = stepMetricsTrack(t, des, act, dur)
on = t >= 0 & t <= dur + 2;
m.trackRms = rms(des(on) - act(on));
% lag: cross-correlation shift between target and response
dtm = median(diff(t)); [xc, lags] = xcorr(act(on) - mean(act(on)), des(on) - mean(des(on)), round(0.3/dtm));
[~, i] = max(xc); m.lagMs = 1e3 * lags(i) * dtm;
pk = max(des(on)); m.overshoot = max(0, (max(act(t >= 0 & t <= dur)) - pk) / max(pk, eps) * 100);
release = t >= dur; ar = act(release); tr = t(release);
outBand = find(abs(ar) > 0.1*pk, 1, 'last');
if isempty(outBand), m.settleMs = 0; else, m.settleMs = 1e3*(tr(min(outBand+1, numel(tr))) - dur); end
end

function mask = modeMask(md, t, modes)
tm = seconds(md.Properties.RowTimes); mn = double(md.ModeNum);
cur = interp1(tm, mn, t, 'previous', NaN);
cur(t < tm(1)) = NaN;
mask = ismember(cur, modes);
end

function off = estimateOffset(tk, yk, tp, yp)
% time offset between SITL log clock and physics clock via roll correlation
offs = -5:0.01:5; best = -inf; off = 0;
for o = offs
    y = interp1(tp + o, yp, tk);
    k = ~isnan(y);
    if nnz(k) > 100
        c = -mean((y(k) - yk(k)).^2);
        if c > best, best = c; off = o; end
    end
end
end
