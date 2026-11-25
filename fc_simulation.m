function S = fc_simulation(binFile, opts)
%FC_SIMULATION Flight-log-calibrated simulation of an ArduCopter quad.
%
%   1. Reads the tuning parameters that were active during the flight.
%   2. Rebuilds ArduPilot's attitude/rate controllers at 400 Hz
%      (AC_PID with FLTT/FLTE/FLTD filters, sqrt controller, input shaping,
%      rate feed-forward, gyro low-pass filter, output limits).
%   3. Grey-box identifies the airframe per axis (control authority K,
%      motor time constant tau, trim torque b) so the simulated drone
%      reproduces the logged flight, and reports the fit.
%   4. Linear analysis: gain/phase margins, crossover, bandwidth,
%      sensitivity peak, noise gain.
%   5. Time-domain tests: rate step, 10 deg attitude step, gust rejection,
%      realistic replay of your own stick inputs with turbulence + gyro noise.
%   6. Searches for an improved tune with safety constraints and compares
%      it with the current tune, including a Monte-Carlo robustness check.
%   7. Identifies and simulates the altitude loop.
%
%   S = fc_simulation("2026-11-09 16-30-17.bin");
%   S = fc_simulation(file, OutDir="sim_results", Quick=true);

arguments
    binFile (1,1) string
    opts.OutDir (1,1) string = ""
    opts.Quick (1,1) logical = false     % coarser search, fewer Monte-Carlo runs
    opts.Seed (1,1) double = 1
end

rng(opts.Seed);
if opts.OutDir == ""
    opts.OutDir = fullfile(fileparts(mfilename('fullpath')), "sim_results");
end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
dt = 1/400;                 % ArduCopter main loop
Td = 0.0075;                % output latency: ~2 loops + DShot/ESC (assumed)

%% ---------------- Load log ----------------
fprintf('\n=== Loading %s\n', binFile);
lo = ardupilotreader(binFile);
T = readMessages(lo, MessageNames={'RATE','ATT','CTUN','PARM','ARM'});
G = @(n) T.MsgData{find(T.MsgName == n, 1)};
rate = G("RATE"); att = G("ATT"); ctun = G("CTUN"); parm = G("PARM");
tsec = @(x) seconds(x.Properties.RowTimes);

% airborne window = longest stretch with throttle out > 0.25
tc = tsec(ctun); air = double(ctun.ThO) > 0.25;
seg = maskSegments(tc, air);
[~, iBest] = max(seg(:,2) - seg(:,1));
win = seg(iBest, :) + [3 -3];
fprintf('Flight window used: %.0f s to %.0f s (%.0f s airborne)\n', win(1), win(2), diff(win));

pAt = @(name, def) paramAt(parm, name, win(1), def);
axesNames = ["Roll","Pitch","Yaw"]; key = ["RLL","PIT","YAW"];
for a = 1:3
    k = key(a);
    C(a) = struct('P', pAt("ATC_RAT_"+k+"_P",0.135), 'I', pAt("ATC_RAT_"+k+"_I",0.135), ...
        'D', pAt("ATC_RAT_"+k+"_D",0.0036), 'FF', pAt("ATC_RAT_"+k+"_FF",0), ...
        'IMAX', pAt("ATC_RAT_"+k+"_IMAX",0.5), 'FLTT', pAt("ATC_RAT_"+k+"_FLTT",20), ...
        'FLTE', pAt("ATC_RAT_"+k+"_FLTE",0), 'FLTD', pAt("ATC_RAT_"+k+"_FLTD",20), ...
        'GYRO', pAt("INS_GYRO_FILTER",20)); %#ok<AGROW>
end
accKey = ["ATC_ACCEL_R_MAX","ATC_ACCEL_P_MAX","ATC_ACCEL_Y_MAX"];
for a = 1:3
    CA(a) = struct('angP', pAt("ATC_ANG_"+key(a)+"_P",4.5), ...
        'acc', deg2rad(pAt(accKey(a),110000)/100), ...
        'tc', pAt("ATC_INPUT_TC",0.15), 'ff', pAt("ATC_RATE_FF_ENAB",1)); %#ok<AGROW>
end
fprintf('\nController in the log (at flight start):\n');
for a = 1:3
    fprintf('  %-5s rate P=%.4f I=%.4f D=%.5f FLTT=%g FLTE=%g FLTD=%g | ANG_P=%.2f ACCEL=%.0f deg/s^2\n', ...
        axesNames(a), C(a).P, C(a).I, C(a).D, C(a).FLTT, C(a).FLTE, C(a).FLTD, CA(a).angP, rad2deg(CA(a).acc));
end
fprintf('  INS_GYRO_FILTER=%g Hz, loop 400 Hz, assumed output latency %.1f ms\n', C(1).GYRO, Td*1e3);

%% ---------------- Identify each axis ----------------
tr = tsec(rate); inW = tr >= win(1) & tr <= win(2);
fsRate = 1 / median(diff(tr));
if fsRate < 200 || nnz(inW) < 400
    error('fc_sim:lowRate', ['Rate data in this log is logged at %.0f Hz (%d samples in the flight window); ' ...
        'identification needs 400 Hz. Set LOG_BITMASK = 180223 (adds fast rate/PID logging) and fly again.'], ...
        fsRate, nnz(inW));
end
ta = tsec(att);  inA = ta >= win(1) & ta <= win(2);
des = {rate.RDes, rate.PDes, rate.YDes}; act = {rate.R, rate.P, rate.Y}; out = {rate.ROut, rate.POut, rate.YOut};
angDes = {att.DesRoll, att.DesPitch, att.DesYaw}; ang = {att.Roll, att.Pitch, att.Yaw};
fprintf('\n=== Grey-box identification (400 Hz controller replica vs %.0f Hz log)\n', 1/median(diff(tr)));
for a = 1:3
    tL = tr(inW);
    [PL(a), ID(a)] = identifyAxis(tL, double(des{a}(inW)), double(act{a}(inW)), double(out{a}(inW)), C(a), dt, Td); %#ok<AGROW>
    fprintf('  %-5s K=%7.1f rad/s^2 per unit | motor tau=%5.1f ms | trim=%+6.2f rad/s^2 | fit: rate %4.1f %%, output %4.1f %%\n', ...
        axesNames(a), PL(a).K, PL(a).tau*1e3, PL(a).b, ID(a).fitRate, ID(a).fitOut);
end

% Attitude-level validation: replay logged angle targets through the full cascade
fprintf('\n=== Attitude-level validation (replay logged angle targets through the full cascade)\n');
for a = 1:3
    tL = ta(inA); y = deg2rad(double(ang{a}(inA))); u = deg2rad(double(angDes{a}(inA)));
    if a == 3, y = unwrap(y); u = y(1) + unwrap(u - y(1)); end
    t4 = (tL(1):dt:tL(end))';
    cmd = interp1(tL, u, t4, 'previous');
    ca = CA(a); ca.ff = 0;            % logged DesRoll is already the shaped target
    sim = simAxis(C(a), ca, PL(a), "angle", cmd, dt, 0, 0, y(1));
    idx = min(round((tL - tL(1))/dt) + 1, numel(t4));
    VAL(a).fit = fitPct(y, sim.ang(idx)); %#ok<AGROW>
    VAL(a).rms = rad2deg(rms(sim.ang(idx) - y)); %#ok<AGROW>
    VAL(a).t = tL; VAL(a).log = y; VAL(a).sim = sim.ang(idx); %#ok<AGROW>
    fprintf('  %-5s angle fit %5.1f %%  (RMS model error %.2f deg)\n', axesNames(a), VAL(a).fit, VAL(a).rms);
end

%% ---------------- Linear analysis + tune search ----------------
fprintf('\n=== Stability & performance: current tune vs optimized tune\n');
for a = 1:3
    fprintf('  %s:\n', axesNames(a));
    LN(a).cur = loopMetrics(C(a), PL(a), Td); %#ok<AGROW>
    [Copt(a), LN(a).opt] = searchTune(C(a), PL(a), Td, a == 3, opts.Quick); %#ok<AGROW>
end
for a = 1:3
    CAopt(a) = CA(a); %#ok<AGROW>
    CAopt(a).angP = searchAngP(Copt(a), CA(a), PL(a), dt);
end

%% ---------------- Time-domain scenarios ----------------
fprintf('\n=== Time-domain scenarios (nonlinear 400 Hz simulation)\n');
stepDeg = [10 10 30];
gust = [8 8 2];                       % rad/s^2 torque-disturbance step (assumed)
for a = 1:3
    n = round(2.5/dt); cmd = [zeros(round(0.2/dt),1); deg2rad(stepDeg(a))*ones(n,1)];
    for v = ["cur","opt"]
        [c, ca] = pick(v, C(a), Copt(a), CA(a), CAopt(a));
        s = simAxis(c, ca, PL(a), "angle", cmd, dt, 0, 0, 0);
        TD(a).(v).step = s; %#ok<AGROW>
        TD(a).(v).stepInfo = stepMetrics((0:numel(cmd)-1)'*dt, s.ang, deg2rad(stepDeg(a)), 0.2); %#ok<AGROW>
        d = [zeros(round(0.5/dt),1); gust(a)*ones(round(2/dt),1)];
        g = simAxis(c, ca, PL(a), "angle", zeros(size(d)), dt, d, 0, 0);
        TD(a).(v).gust = g; %#ok<AGROW>
        TD(a).(v).gustPeak = rad2deg(max(abs(g.ang))); %#ok<AGROW>
        TD(a).(v).gustRecover = recoveryTime((0:numel(d)-1)'*dt, g.ang, 0.5, deg2rad(0.5)); %#ok<AGROW>
    end
end

% Realistic replay: your own stick inputs + turbulence + gyro noise, both tunes
for a = 1:3
    tL = ta(inA); u = deg2rad(double(angDes{a}(inA)));
    if a == 3, u = unwrap(u); u = u - u(1); end
    t4 = (tL(1):dt:tL(end))';
    cmd = interp1(tL, u, t4, 'previous');
    turb = coloredNoise(numel(t4), dt, 3, gust(a)*0.5);   % ~3 Hz turbulence torque
    noise = deg2rad(0.5) * randn(numel(t4), 1);            % gyro noise 0.5 deg/s RMS
    for v = ["cur","opt"]
        [c, ca] = pick(v, C(a), Copt(a), CA(a), CAopt(a));
        ca.ff = 0;
        s = simAxis(c, ca, PL(a), "angle", cmd, dt, turb, noise, 0);
        RP(a).(v).rmsErr = rad2deg(rms(s.angT - s.ang)); %#ok<AGROW>
        RP(a).(v).maxErr = rad2deg(max(abs(s.angT - s.ang))); %#ok<AGROW>
        hp = s.out - movmean(s.out, round(0.05/dt));
        RP(a).(v).outNoise = 100 * rms(hp); %#ok<AGROW>
        RP(a).(v).sat = 100 * mean(abs(s.out) >= 0.999); %#ok<AGROW>
        RP(a).(v).sim = s; %#ok<AGROW>
    end
    RP(a).t = t4; %#ok<AGROW>
end

%% ---------------- Monte-Carlo robustness ----------------
nMC = 300; if opts.Quick, nMC = 80; end
fprintf('\n=== Monte-Carlo robustness (%d airframe variations: K +/-30 %%, tau x0.6-1.6, latency +0-5 ms)\n', nMC);
for a = 1:3
    for v = ["cur","opt"]
        c = C(a); if v == "opt", c = Copt(a); end
        pm = nan(nMC,1); gm = nan(nMC,1);
        for i = 1:nMC
            pl = PL(a); pl.K = pl.K * (0.7 + 0.6*rand); pl.tau = pl.tau * (0.6 + 1.0*rand);
            m = loopMargins(c, pl, Td + 0.005*rand);
            pm(i) = m.PM; gm(i) = m.GMdB;
        end
        MC(a).(v).pm = pm; MC(a).(v).gm = gm; %#ok<AGROW>
        MC(a).(v).poor = 100 * mean(pm < 35 | gm < 4); %#ok<AGROW>
        MC(a).(v).unstable = 100 * mean(pm <= 0 | gm <= 0); %#ok<AGROW>
        MC(a).(v).pm5 = prctile(pm, 5); MC(a).(v).gm5 = prctile(gm, 5); %#ok<AGROW>
    end
end

%% ---------------- Altitude loop ----------------
fprintf('\n=== Altitude loop\n');
ALT = altitudeModel(ctun, win, pAt, dt);

%% ---------------- Report ----------------
printReport(axesNames, C, Copt, CA, CAopt, PL, ID, VAL, LN, TD, RP, MC, ALT, stepDeg, gust);
makePlots(opts.OutDir, axesNames, ID, VAL, C, Copt, PL, Td, TD, RP, MC, ALT, dt, stepDeg);
writeParams(opts.OutDir, key, C, Copt, CA, CAopt);

S = struct('controller', C, 'controllerOpt', Copt, 'angle', CA, 'angleOpt', CAopt, 'plant', PL, ...
    'ident', ID, 'validation', VAL, 'linear', LN, 'timeDomain', TD, 'replay', RP, ...
    'monteCarlo', MC, 'altitude', ALT, 'window', win, 'outDir', opts.OutDir);
fprintf('\nFigures and recommended parameters saved in:\n  %s\n', opts.OutDir);
end

%% =====================================================================
%% Simulation core
%% =====================================================================
function s = simAxis(c, ca, pl, mode, u, dt, dist, noise, x0)
% mode "rate": u = rate target (rad/s). mode "angle": u = pilot angle command (rad).
n = numel(u);
if isscalar(dist), dist = dist * ones(n,1); end
if isscalar(noise), noise = noise * ones(n,1); end
aT = lpAlpha(c.FLTT, dt); aE = lpAlpha(c.FLTE, dt); aD = lpAlpha(c.FLTD, dt);
[b0,b1,b2,a1,a2] = biquadLP(c.GYRO, 1/dt);
nd = max(0, round(pl.Td / dt)); buf = zeros(nd + 1, 1);
am = dt / (pl.tau + dt);
% start in trimmed hover: integrator already holds the trim torque
uTrim = max(min(-pl.b / pl.K, c.IMAX), -c.IMAX);
buf(:) = uTrim; bi = 1; nb = nd + 1;          % ring buffer for output latency
r = 0; th = x0; um = uTrim; tf = 0; ef = 0; efPrev = 0; dF = 0; I = uTrim;
g1 = 0; g2 = 0; thT = x0; wT = 0; first = true;
% cache struct fields as locals (much faster inside the loop)
P = c.P; Ki = c.I; D = c.D; FF = c.FF; IMAX = c.IMAX; K = pl.K; b = pl.b;
isRate = mode == "rate"; useFF = false; acc = 0; tcInv = 0; angP = 0;
if ~isRate, useFF = logical(ca.ff); acc = ca.acc; tcInv = 1/max(ca.tc,1e-3); angP = ca.angP; end
sr = zeros(n,1); so = sr; sa = sr; st = sr; srt = sr;
for k = 1:n
    % gyro + 2nd-order low-pass (LowPassFilter2p)
    x = r + noise(k);
    g0 = x - g1*a1 - g2*a2; meas = g0*b0 + g1*b1 + g2*b2; g2 = g1; g1 = g0;
    % target
    if isRate
        tgt = u(k);
    else
        if useFF
            wDes = sqrtCtrl(u(k) - thT, tcInv, acc, dt);
            wT = wT + max(min(wDes - wT, acc*dt), -acc*dt);
            thT = thT + wT*dt;
        else
            thT = u(k); wT = 0;
        end
        tgt = wT + sqrtCtrl(thT - th, angP, acc, dt);
    end
    % AC_PID::update_all
    if first, tf = tgt; ef = tgt - meas; efPrev = ef; first = false; end
    tf = tf + aT*(tgt - tf);
    ef = ef + aE*((tf - meas) - ef);
    dF = dF + aD*((ef - efPrev)/dt - dF); efPrev = ef;
    o = P*ef + I + D*dF + FF*tf;
    if abs(o) < 1 || sign(ef) ~= sign(o)                 % anti-windup
        I = max(min(I + Ki*ef*dt, IMAX), -IMAX);
    end
    o = max(min(P*ef + I + D*dF + FF*tf, 1), -1);
    % latency (ring buffer) + motor lag + rigid body
    delayed = buf(bi); buf(bi) = o; bi = bi + 1; if bi > nb, bi = 1; end
    if nd == 0, delayed = o; end
    um = um + am*(delayed - um);
    r = r + (K*um + b + dist(k))*dt;
    th = th + r*dt;
    sr(k) = r; so(k) = o; sa(k) = th; st(k) = thT; srt(k) = tgt;
end
s.rate = sr; s.out = so; s.ang = sa; s.angT = st; s.rateT = srt;
end

function out = sqrtCtrl(err, p, acc, dt)
if acc <= 0 || p <= 0
    out = p*err;
else
    lin = acc / p^2;
    if abs(err) > lin
        out = sign(err) * sqrt(2*acc*(abs(err) - lin/2));
    else
        out = p*err;
    end
end
out = max(min(out, abs(err)/dt), -abs(err)/dt);
end

function a = lpAlpha(fc, dt)
if fc <= 0, a = 1; else, a = dt / (dt + 1/(2*pi*fc)); end
end

function [b0,b1,b2,a1,a2] = biquadLP(fc, fs)
if fc <= 0, b0 = 1; b1 = 0; b2 = 0; a1 = 0; a2 = 0; return; end
fr = fs / fc; ohm = tan(pi/fr); c = 1 + 2*cos(pi/4)*ohm + ohm^2;
b0 = ohm^2/c; b1 = 2*b0; b2 = b0;
a1 = 2*(ohm^2 - 1)/c; a2 = (1 - 2*cos(pi/4)*ohm + ohm^2)/c;
end

%% =====================================================================
%% Identification
%% =====================================================================
function [pl, info] = identifyAxis(tL, desDeg, actDeg, out, c, dt, Td)
t4 = (tL(1):dt:tL(end))';
tgt = interp1(tL, deg2rad(desDeg), t4, 'pchip');
idx = min(round((tL - tL(1))/dt) + 1, numel(t4));
y = deg2rad(actDeg);
sy = max(std(y), 1e-3); so = max(std(out), 1e-3);
best = inf; xBest = [];
for K0 = [150 400 1000]                    % multi-start
    x0 = [log(K0), log(0.03), 0];
    x = fminsearch(@cost, x0, optimset('MaxFunEvals', 300, 'TolX', 1e-3, 'TolFun', 1e-5, 'Display', 'off'));
    J = cost(x);
    if J < best, best = J; xBest = x; end
end
pl = unpack(xBest, Td);
s = simAxis(c, [], pl, "rate", tgt, dt, 0, 0, 0);
info.fitRate = fitPct(y, s.rate(idx));
info.fitOut = fitPct(out, s.out(idx));
info.t = tL; info.logRate = y; info.simRate = s.rate(idx); info.logOut = out; info.simOut = s.out(idx);

    function J = cost(x)
        p = unpack(x, Td);
        sm = simAxis(c, [], p, "rate", tgt, dt, 0, 0, 0);
        e1 = (sm.rate(idx) - y)/sy; e2 = (sm.out(idx) - out)/so;
        J = mean(e1.^2) + mean(e2.^2);
        if ~isfinite(J), J = 1e6; end
    end
end

function p = unpack(x, Td)
p.K = exp(x(1)); p.tau = min(max(exp(x(2)), 0.004), 0.25); p.b = x(3); p.Td = Td;
end

%% =====================================================================
%% Linear analysis
%% =====================================================================
function m = loopMargins(c, pl, Td)
% Fast metrics for searches / Monte-Carlo: margins, crossover, Ms, noise gain.
[L, Tn] = buildLoop(c, pl, Td);
am = allmargin(L);
gm = am.GainMargin(am.GMFrequency > 0); if isempty(gm), gm = inf; end
pm = am.PhaseMargin(am.PMFrequency > 0); wc = am.PMFrequency(am.PMFrequency > 0);
if isempty(pm), pm = 90; wc = 0; end
[m.PM, i] = min(pm); m.wcHz = wc(i)/(2*pi);
m.GMdB = 20*log10(min(gm));
if ~am.Stable, m.PM = min(m.PM, 0); m.GMdB = min(m.GMdB, 0); end
w = 2*pi*logspace(-1, 2.5, 300);
Lw = squeeze(freqresp(L, w));
m.Ms = max(abs(1 ./ (1 + Lw)));
m.noise100 = abs(squeeze(freqresp(Tn, 2*pi*100)));
end

function [L, Tn, Tcl] = buildLoop(c, pl, Td)
s = tf('s');
Gp = pl.K / (s * (pl.tau*s + 1)); Gp.InputDelay = Td;
wg = 2*pi*c.GYRO; Hg = wg^2 / (s^2 + sqrt(2)*wg*s + wg^2);
Cs = lp1(c.FLTE) * (c.P + c.I/s + c.D*s*lp1(c.FLTD));
L = Cs * Gp * Hg;
Tn = feedback(Cs*Hg, Gp);
Tcl = lp1(c.FLTT) * feedback(Cs*Gp, Hg);
end

function m = loopMetrics(c, pl, Td)
m = loopMargins(c, pl, Td);
[m.L, ~, m.T] = buildLoop(c, pl, Td);
try, m.bwHz = bandwidth(m.T)/(2*pi); catch, m.bwHz = NaN; end
tt = (0:0.0005:1.5)';
ws = warning('off', 'all');
try
    y = step(m.T, tt);
    si = stepinfo(y, tt, 1, 'SettlingTimeThreshold', 0.05);
    m.rise = si.RiseTime; m.overshoot = si.Overshoot; m.settle = si.SettlingTime;
catch
    m.rise = NaN; m.overshoot = NaN; m.settle = NaN;
end
warning(ws);
end

function F = lp1(fc)
if fc <= 0, F = tf(1); else, F = tf(1, [1/(2*pi*fc) 1]); end
end

function [cOpt, mOpt] = searchTune(c, pl, Td, isYaw, quick)
% Maximise crossover subject to PM >= 50 deg, GM >= 8 dB, Ms <= 1.6,
% noise gain at 100 Hz <= 1.25x current (vibration-limited airframe).
base = loopMargins(c, pl, Td);
if quick, pS = 0.5:0.25:3; dS = 0.25:0.25:3; else, pS = 0.5:0.1:3; dS = 0.25:0.125:3; end
if isYaw, dS = 1; end
best = -inf; cOpt = c;
fprintf('    searching %d gain combinations ...', numel(pS)*numel(dS));
for ps = pS
    for ds = dS
        cc = c; cc.P = c.P*ps; cc.D = c.D*ds;
        if isYaw, cc.I = cc.P/10; else, cc.I = cc.P; end
        m = loopMargins(cc, pl, Td);
        ok = m.PM >= 50 && m.GMdB >= 8 && m.Ms <= 1.6 && m.noise100 <= 1.25*base.noise100;
        if ok && m.wcHz > best
            best = m.wcHz; cOpt = cc;
        end
    end
end
if isinf(best), fprintf(' no safer/faster tune found - keeping current\n'); else, fprintf(' done\n'); end
mOpt = loopMetrics(cOpt, pl, Td);
end

function angP = searchAngP(c, ca, pl, dt)
% Smallest ANG_P (gentlest, least motor noise) whose 10 deg step settles
% within 10 % of the fastest achievable, with <= 5 % overshoot. Range 3-12.
cmd = [zeros(round(0.2/dt),1); deg2rad(10)*ones(round(2/dt),1)];
ps = 3:0.5:12; st = inf(size(ps));
for i = 1:numel(ps)
    ca2 = ca; ca2.angP = ps(i);
    s = simAxis(c, ca2, pl, "angle", cmd, dt, 0, 0, 0);
    m = stepMetrics((0:numel(cmd)-1)'*dt, s.ang, deg2rad(10), 0.2);
    if m.overshoot <= 5 && isfinite(m.settle), st(i) = m.settle; end
end
if all(isinf(st)), angP = ca.angP; return; end
angP = ps(find(st <= 1.1*min(st), 1));
end

%% =====================================================================
%% Altitude
%% =====================================================================
function A = altitudeModel(ctun, win, pAt, dt)
t = seconds(ctun.Properties.RowTimes); m = t >= win(1) & t <= win(2);
t = t(m); thr = double(ctun.ThO(m)); thH = double(ctun.ThH(m)); alt = double(ctun.Alt(m)); dAlt = double(ctun.DAlt(m));
cr = double(ctun.CRt(m));
vzNum = gradient(alt, t);
sc = (vzNum' * cr) / max(vzNum' * vzNum, eps);        % detect cm/s vs m/s
if abs(sc) > 20, cr = cr / 100; end
g = 9.80665;
t4 = (t(1):dt:t(end))';
u = interp1(t, thr ./ max(thH, 0.05), t4, 'previous');
idx = min(round((t - t(1))/dt) + 1, numel(t4));
cost = @(x) altCost(x, u, dt, idx, cr, g, cr(1));
x = fminsearch(cost, [log(1) log(0.3) 0], optimset('MaxFunEvals', 400, 'Display', 'off'));
A.k = exp(x(1)); A.drag = exp(x(2)); A.bias = x(3);
vz = vzSim([A.k A.drag A.bias], u, dt, g, cr(1));
A.fitVz = fitPct(cr, vz(idx));
% closed loop with logged PSC gains
pz = pAt("PSC_POSZ_P",1); vp = pAt("PSC_VELZ_P",5); ap = pAt("PSC_ACCZ_P",0.5); ai = pAt("PSC_ACCZ_I",1);
imax = pAt("PSC_ACCZ_IMAX",800); fe = pAt("PSC_ACCZ_FLTE",20); vup = pAt("PILOT_SPEED_UP",250)/100;
vdn = pAt("PILOT_SPEED_DN",0)/100; if vdn <= 0, vdn = vup; end
A.gains = [pz vp ap ai];
cmd = interp1(t, dAlt, t4, 'previous');
s = altLoop(A, cmd, dt, g, pz, vp, ap, ai, imax, fe, vup, vdn, 0, alt(1));
A.fitAlt = fitPct(alt, s.z(idx)); A.t = t; A.logAlt = alt; A.simAlt = s.z(idx);
A.rmsAlt = rms(s.z(idx) - alt);
st = [zeros(round(1/dt),1); ones(round(8/dt),1)];
A.step = altLoop(A, st, dt, g, pz, vp, ap, ai, imax, fe, vup, vdn, 0, 0);
A.stepInfo = stepMetrics((0:numel(st)-1)'*dt, A.step.z, 1, 1);
dd = [zeros(round(1/dt),1); -2*ones(round(6/dt),1)];
A.gust = altLoop(A, zeros(size(dd)), dt, g, pz, vp, ap, ai, imax, fe, vup, vdn, dd, 0);
A.gustDrop = -min(A.gust.z);
fprintf('  vertical model: thrust gain %.2f, drag %.2f 1/s | climb-rate fit %.1f %% | altitude replay fit %.1f %% (RMS %.2f m)\n', ...
    A.k, A.drag, A.fitVz, A.fitAlt, A.rmsAlt);
end

function J = altCost(x, u, dt, idx, cr, g, v0)
vz = vzSim(x, u, dt, g, v0);
J = mean((vz(idx) - cr).^2);
if ~isfinite(J), J = 1e6; end
end

function vz = vzSim(x, u, dt, g, v0)
k = exp(x(1)); c = exp(x(2)); b = x(3);
vz = zeros(numel(u),1); v = v0;
for i = 1:numel(u)
    v = v + (g*k*(u(i) - 1) - c*v + b)*dt; vz(i) = v;
end
end

function s = altLoop(A, cmd, dt, g, pz, vp, ap, ai, imax, fe, vup, vdn, dist, z0)
n = numel(cmd); if isscalar(dist), dist = dist*ones(n,1); end
z = z0; v = 0; I = 0; ef = 0; aE = lpAlpha(fe, dt); accMeas = 0;
s.z = zeros(n,1); s.thr = s.z;
for k = 1:n
    vt = sqrtCtrl(cmd(k) - z, pz, 2.5, dt); vt = max(min(vt, vup), -vdn);
    at = vp*(vt - v) * 100;                         % cm/s^2
    ef = ef + aE*((at - accMeas*100) - ef);
    I = max(min(I + ai*ef*dt, imax), -imax);
    thr = 1 + (ap*ef + I) * 0.001 / 0.5;            % normalised to hover throttle (~0.5)
    thr = max(min(thr, 1.9), 0.1);
    acc = g*A.k*(thr - 1) - A.drag*v + A.bias + dist(k);
    v = v + acc*dt; z = z + v*dt; accMeas = acc;
    s.z(k) = z; s.thr(k) = thr;
end
end

%% =====================================================================
%% Helpers
%% =====================================================================
function v = paramAt(parm, name, t, def)
pn = string(parm.Name); tp = seconds(parm.Properties.RowTimes);
i = find(pn == name & tp <= t, 1, 'last');
if isempty(i), i = find(pn == name, 1, 'first'); end
if isempty(i), v = def; else, v = double(parm.Value(i)); end
end

function seg = maskSegments(t, mask)
d = diff([0; mask(:); 0]); s = find(d == 1); e = find(d == -1) - 1;
seg = [t(s) t(e)];
end

function f = fitPct(y, yh)
f = 100 * (1 - norm(y - yh) / max(norm(y - mean(y)), eps));
end

function m = stepMetrics(t, y, final, t0)
y = y(:); k = t >= t0; tt = t(k) - t0; yy = y(k);
i10 = find(yy >= 0.1*final, 1); i90 = find(yy >= 0.9*final, 1);
if isempty(i10) || isempty(i90), m.rise = NaN; else, m.rise = tt(i90) - tt(i10); end
m.overshoot = max(0, (max(yy) - final)/final*100);
out = find(abs(yy - final) > 0.05*abs(final), 1, 'last');
if isempty(out), m.settle = 0; elseif out == numel(yy), m.settle = NaN; else, m.settle = tt(out + 1); end
end

function tr = recoveryTime(t, y, t0, tol)
k = find(t >= t0 & abs(y) > tol, 1, 'last');
if isempty(k), tr = 0; else, tr = t(k) - t0; end
end

function x = coloredNoise(n, dt, fc, sd)
w = randn(n,1); a = lpAlpha(fc, dt); x = zeros(n,1);
for i = 2:n, x(i) = x(i-1) + a*(w(i) - x(i-1)); end
x = x / max(std(x), eps) * sd;
end

function [c, ca] = pick(v, cCur, cOpt, caCur, caOpt)
if v == "cur", c = cCur; ca = caCur; else, c = cOpt; ca = caOpt; end
end

function pct = better(cur, opt, lowerIsBetter)
if lowerIsBetter, pct = 100*(cur - opt)/max(abs(cur), eps); else, pct = 100*(opt - cur)/max(abs(cur), eps); end
end

%% =====================================================================
%% Output
%% =====================================================================
function printReport(axn, C, Copt, CA, CAopt, PL, ID, VAL, LN, TD, RP, MC, ALT, stepDeg, gust)
fprintf('\n\n######################## SIMULATION REPORT ########################\n');
fprintf('\nMODEL QUALITY (how well the simulated drone reproduces your flight)\n');
fprintf('  %-6s %10s %10s %12s\n', 'Axis', 'rate fit', 'out fit', 'angle fit');
for a = 1:3
    fprintf('  %-6s %9.1f%% %9.1f%% %11.1f%%\n', axn(a), ID(a).fitRate, ID(a).fitOut, VAL(a).fit);
end
fprintf('  Altitude: climb-rate fit %.1f %%, altitude replay fit %.1f %%\n', ALT.fitVz, ALT.fitAlt);

fprintf('\nIDENTIFIED AIRFRAME\n');
for a = 1:3
    fprintf('  %-6s control authority %.0f rad/s^2 (%.0f deg/s^2) at full output, motor response %.0f ms\n', ...
        axn(a), PL(a).K, rad2deg(PL(a).K), PL(a).tau*1e3);
end

for a = 1:3
    c = LN(a).cur; o = LN(a).opt;
    fprintf('\n--- %s ---------------------------------------------------------\n', upper(axn(a)));
    fprintf('  Gains        current: P %.4f I %.4f D %.5f ANG_P %.1f\n', C(a).P, C(a).I, C(a).D, CA(a).angP);
    fprintf('               optimized: P %.4f I %.4f D %.5f ANG_P %.1f\n', Copt(a).P, Copt(a).I, Copt(a).D, CAopt(a).angP);
    fprintf('  %-34s %10s %10s %10s\n', 'Metric', 'current', 'optimized', 'change');
    row('Rate-loop crossover (Hz)', c.wcHz, o.wcHz, false);
    row('Phase margin (deg)  [>45]', c.PM, o.PM, false);
    row('Gain margin (dB)    [>6]', c.GMdB, o.GMdB, false);
    row('Sensitivity peak Ms [<1.7]', c.Ms, o.Ms, true);
    row('Rate closed-loop bandwidth (Hz)', c.bwHz, o.bwHz, false);
    row('Rate step rise time (ms)', 1e3*c.rise, 1e3*o.rise, true);
    row('Rate step overshoot (%)', c.overshoot, o.overshoot, true);
    sc = TD(a).cur.stepInfo; so = TD(a).opt.stepInfo;
    row(sprintf('%d deg angle step rise (ms)', stepDeg(a)), 1e3*sc.rise, 1e3*so.rise, true);
    row(sprintf('%d deg angle step overshoot (%%)', stepDeg(a)), sc.overshoot, so.overshoot, true);
    row(sprintf('%d deg angle step settle 5%% (ms)', stepDeg(a)), 1e3*sc.settle, 1e3*so.settle, true);
    row(sprintf('Gust %g rad/s^2: peak tilt (deg)', gust(a)), TD(a).cur.gustPeak, TD(a).opt.gustPeak, true);
    row('Gust: recovery to 0.5 deg (ms)', 1e3*TD(a).cur.gustRecover, 1e3*TD(a).opt.gustRecover, true);
    row('Replay w/ turbulence: RMS error (deg)', RP(a).cur.rmsErr, RP(a).opt.rmsErr, true);
    row('Replay: max error (deg)', RP(a).cur.maxErr, RP(a).opt.maxErr, true);
    row('Motor-command noise (% RMS)', RP(a).cur.outNoise, RP(a).opt.outNoise, true);
    row('Noise gain @100 Hz', c.noise100, o.noise100, true);
    row('Monte-Carlo: poor-margin cases (%)', MC(a).cur.poor, MC(a).opt.poor, true);
    row('Monte-Carlo: 5th pct phase margin', MC(a).cur.pm5, MC(a).opt.pm5, false);
end
fprintf('\n--- ALTITUDE (current PSC gains) -------------------------------------\n');
fprintf('  1 m step: rise %.2f s, overshoot %.1f %%, settle %.2f s\n', ALT.stepInfo.rise, ALT.stepInfo.overshoot, ALT.stepInfo.settle);
fprintf('  2 m/s^2 downdraft: altitude drop %.2f m\n', ALT.gustDrop);
fprintf('####################################################################\n');

    function row(name, a1, a2, lowerBetter)
        ch = better(a1, a2, lowerBetter);
        if ~isfinite(ch), chs = '-'; else, chs = sprintf('%+.0f%%', ch); end
        fprintf('  %-34s %10.2f %10.2f %10s\n', name, a1, a2, chs);
    end
end

function closeIfHidden(f, vis)
if strcmp(vis, 'off'), delete(f); end
end

function writeParams(outDir, key, C, Copt, CA, CAopt)
fid = fopen(fullfile(outDir, "recommended_params.txt"), 'w');
fprintf(fid, '# Simulation-optimised rate/attitude gains (apply in steps, verify with test flights)\n');
fprintf(fid, '# name, current, suggested\n');
for a = 1:3
    k = key(a);
    fprintf(fid, 'ATC_RAT_%s_P, %.4f, %.4f\n', k, C(a).P, Copt(a).P);
    fprintf(fid, 'ATC_RAT_%s_I, %.4f, %.4f\n', k, C(a).I, Copt(a).I);
    fprintf(fid, 'ATC_RAT_%s_D, %.5f, %.5f\n', k, C(a).D, Copt(a).D);
    fprintf(fid, 'ATC_ANG_%s_P, %.2f, %.2f\n', k, CA(a).angP, CAopt(a).angP);
end
fclose(fid);
end

function makePlots(outDir, axn, ID, VAL, C, Copt, PL, Td, TD, RP, MC, ALT, dt, stepDeg)
% Figures open on screen in MATLAB desktop; hidden when run with -batch.
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
close = @(f) closeIfHidden(f, vis);
cCur = [0.85 0.33 0.1]; cOpt = [0 0.45 0.74]; cLog = [0.3 0.3 0.3];

f = figure('Visible', vis, 'Position', [50 50 1400 900], 'Color', 'w');
tl = tiledlayout(3, 2, 'TileSpacing', 'compact'); title(tl, 'Model validation: logged flight vs simulation');
for a = 1:3
    i = ID(a); w = i.t >= i.t(1) + 20 & i.t <= i.t(1) + 60;
    nexttile; plot(i.t(w), rad2deg(i.logRate(w)), 'Color', cLog); hold on; plot(i.t(w), rad2deg(i.simRate(w)), 'Color', cOpt);
    title(sprintf('%s rate (fit %.0f%%)', axn(a), i.fitRate)); ylabel('deg/s'); legend('log', 'sim'); grid on
    v = VAL(a); w = v.t >= v.t(1) + 20 & v.t <= v.t(1) + 60;
    nexttile; plot(v.t(w), rad2deg(v.log(w)), 'Color', cLog); hold on; plot(v.t(w), rad2deg(v.sim(w)), 'Color', cOpt);
    title(sprintf('%s angle (fit %.0f%%)', axn(a), v.fit)); ylabel('deg'); legend('log', 'sim'); grid on
end
exportgraphics(f, fullfile(outDir, "1_model_validation.png"), 'Resolution', 110); close(f);

f = figure('Visible', vis, 'Position', [50 50 1400 450], 'Color', 'w');
tl = tiledlayout(1, 3, 'TileSpacing', 'compact'); title(tl, 'Rate-loop open-loop frequency response');
for a = 1:3
    nexttile;
    mc = loopMetrics(C(a), PL(a), Td); mo = loopMetrics(Copt(a), PL(a), Td);
    w = 2*pi*logspace(-0.5, 2.4, 400);
    [m1, p1] = bode(mc.L, w); [m2, p2] = bode(mo.L, w);
    yyaxis left; semilogx(w/2/pi, 20*log10(squeeze(m1)), '-', 'Color', cCur); hold on
    semilogx(w/2/pi, 20*log10(squeeze(m2)), '-', 'Color', cOpt); yline(0, ':'); ylabel('gain (dB)');
    yyaxis right; semilogx(w/2/pi, squeeze(p1), '--', 'Color', cCur); semilogx(w/2/pi, squeeze(p2), '--', 'Color', cOpt);
    yline(-180, ':'); ylabel('phase (deg)'); xlabel('Hz'); grid on
    title(sprintf('%s: PM %.0f%s -> %.0f%s, GM %.1f -> %.1f dB', axn(a), mc.PM, char(176), mo.PM, char(176), mc.GMdB, mo.GMdB));
    legend('current gain', 'optimized gain', '', 'current phase', 'optimized phase', 'Location', 'southwest');
end
exportgraphics(f, fullfile(outDir, "2_frequency_response.png"), 'Resolution', 110); close(f);

f = figure('Visible', vis, 'Position', [50 50 1400 800], 'Color', 'w');
tl = tiledlayout(2, 3, 'TileSpacing', 'compact'); title(tl, 'Attitude step and gust rejection (nonlinear 400 Hz sim)');
for a = 1:3
    nexttile; t = (0:numel(TD(a).cur.step.ang)-1)*dt;
    plot(t, rad2deg(TD(a).cur.step.ang), 'Color', cCur, 'LineWidth', 1.4); hold on
    plot(t, rad2deg(TD(a).opt.step.ang), 'Color', cOpt, 'LineWidth', 1.4); yline(stepDeg(a), ':');
    title(sprintf('%s %d%s step', axn(a), stepDeg(a), char(176))); xlabel('s'); ylabel('deg'); legend('current', 'optimized', 'Location', 'southeast'); grid on; xlim([0 1.5])
end
for a = 1:3
    nexttile; t = (0:numel(TD(a).cur.gust.ang)-1)*dt;
    plot(t, rad2deg(TD(a).cur.gust.ang), 'Color', cCur, 'LineWidth', 1.4); hold on
    plot(t, rad2deg(TD(a).opt.gust.ang), 'Color', cOpt, 'LineWidth', 1.4);
    title(sprintf('%s gust rejection', axn(a))); xlabel('s'); ylabel('deg'); legend('current', 'optimized'); grid on
end
exportgraphics(f, fullfile(outDir, "3_step_and_gust.png"), 'Resolution', 110); close(f);

f = figure('Visible', vis, 'Position', [50 50 1400 800], 'Color', 'w');
tl = tiledlayout(3, 1, 'TileSpacing', 'compact'); title(tl, 'Replay of your stick inputs with turbulence + gyro noise: tracking error');
for a = 1:3
    nexttile; t = RP(a).t - RP(a).t(1); w = t >= 20 & t <= 80;
    e1 = rad2deg(RP(a).cur.sim.angT - RP(a).cur.sim.ang); e2 = rad2deg(RP(a).opt.sim.angT - RP(a).opt.sim.ang);
    plot(t(w), e1(w), 'Color', cCur); hold on; plot(t(w), e2(w), 'Color', cOpt);
    title(sprintf('%s: RMS error %.2f%s -> %.2f%s', axn(a), RP(a).cur.rmsErr, char(176), RP(a).opt.rmsErr, char(176)));
    ylabel('deg'); legend('current', 'optimized'); grid on
end
xlabel('s');
exportgraphics(f, fullfile(outDir, "4_replay_tracking.png"), 'Resolution', 110); close(f);

f = figure('Visible', vis, 'Position', [50 50 1400 450], 'Color', 'w');
tl = tiledlayout(1, 3, 'TileSpacing', 'compact'); title(tl, 'Monte-Carlo robustness: margins over airframe uncertainty');
for a = 1:3
    nexttile;
    scatter(MC(a).cur.gm, MC(a).cur.pm, 10, cCur, 'filled', 'MarkerFaceAlpha', 0.4); hold on
    scatter(MC(a).opt.gm, MC(a).opt.pm, 10, cOpt, 'filled', 'MarkerFaceAlpha', 0.4);
    xline(6, ':'); yline(45, ':'); xlabel('gain margin (dB)'); ylabel('phase margin (deg)');
    title(axn(a)); legend('current', 'optimized', 'Location', 'southeast'); grid on
end
exportgraphics(f, fullfile(outDir, "5_monte_carlo.png"), 'Resolution', 110); close(f);

f = figure('Visible', vis, 'Position', [50 50 1400 450], 'Color', 'w');
tl = tiledlayout(1, 3, 'TileSpacing', 'compact'); title(tl, 'Altitude loop');
nexttile; plot(ALT.t, ALT.logAlt, 'Color', cLog); hold on; plot(ALT.t, ALT.simAlt, 'Color', cOpt);
title(sprintf('Altitude replay (fit %.0f%%)', ALT.fitAlt)); ylabel('m'); legend('log', 'sim'); grid on
nexttile; t = (0:numel(ALT.step.z)-1)*dt; plot(t, ALT.step.z, 'Color', cOpt, 'LineWidth', 1.4); yline(1, ':');
title('1 m altitude step'); xlabel('s'); ylabel('m'); grid on
nexttile; t = (0:numel(ALT.gust.z)-1)*dt; plot(t, ALT.gust.z, 'Color', cCur, 'LineWidth', 1.4);
title(sprintf('2 m/s^2 downdraft: drop %.2f m', ALT.gustDrop)); xlabel('s'); ylabel('m'); grid on
exportgraphics(f, fullfile(outDir, "6_altitude.png"), 'Resolution', 110); close(f);
end
