function F = fleet_analysis(opts)
%FLEET_ANALYSIS Your FC flight-log analysis across every log of the drone.
%   1. Splits each log into flights (EV take-off -> landed), removes crash/upset
%      windows, and computes flight-performance metrics per flight.
%   2. Grey-box system identification of roll/pitch/yaw from 400 Hz RATE data:
%        d(omega)/dt = K * u(t - tau) - D * omega + c       (u = rate-controller output)
%   3. Loop analysis with the identified plant and ArduPilot's rate PID (as flown
%      and Tune C): crossover, phase margin, gain margin, plus a Monte-Carlo
%      robustness sweep over plant uncertainty.
%   F = fleet_analysis();

arguments
    opts.LogDir (1,1) string = "logs"
    opts.OutDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), "fleet_results")
end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end

% log list and windows to remove (crash / upset), seconds from log start
L = struct( ...
 'file', {"2026-11-09 16-30-17","2026-11-10 17-38-46","2026-11-10 18-09-06","2026-11-11 17-30-20", ...
          "2026-11-11 17-39-59","2026-11-12 08-56-06","2026-11-12 08-59-02","2026-11-12 17-04-34"}, ...
 'cut',  {zeros(0,2), [283.0 inf], zeros(0,2), zeros(0,2), zeros(0,2), zeros(0,2), zeros(0,2), [930.0 937.0]}, ...
 'note', {"first flights, 10 Hz logging", "loose motor; crash at 285.5 s removed", "ground: motor tests, compass refusals", ...
          "compass refusals; 1.7 s hop", "ground: armed at idle", "ground: compass refusal", "ground", ...
          "after mechanical fixes; upset at 934 s removed"});

flights = struct([]); logs = struct([]);
L = L(~ismember([L.file], "2026-11-10 17-38-46"));   % 10 Nov flight excluded from this report
for i = 1:numel(L)
    f = fullfile(opts.LogDir, L(i).file + ".bin");
    fprintf('\n=== %s (%s)\n', L(i).file, L(i).note);
    [lg, fl] = analyseLog(f, L(i));
    logs = [logs, lg]; %#ok<AGROW>
    for k = 1:numel(fl), flights = [flights, fl(k)]; end %#ok<AGROW>
end
F.logs = logs; F.flights = flights;

% ---- flight table
if ~isempty(flights)
    FT = struct2table(rmfield(flights, {'id', 'ratedata'}));
    FT.Properties.RowNames = cellstr([flights.id]);
    fprintf('\n================ PER-FLIGHT PERFORMANCE (crash windows removed) ================\n');
    disp(FT);
    writetable(FT, fullfile(opts.OutDir, "flights.csv"), 'WriteRowNames', true);
    F.FT = FT;
end
LT = struct2table(logs); writetable(LT, fullfile(opts.OutDir, "logs.csv")); F.LT = LT;

% ---- system identification on the 400 Hz flights (good ones)
good = flights([flights.rate_hz] > 300 & [flights.dur_s] > 40 & startsWith([flights.id], "2026-11-12"));   % healthy airframe, one tune
fprintf('\n================ SYSTEM IDENTIFICATION (400 Hz RATE, %d flights) ================\n', numel(good));
ax = ["roll", "pitch", "yaw"];
ID = struct();
for a = 1:3
    w = []; o = []; z = []; dt = []; seg = [];
    for k = 1:numel(good)
        r = good(k).ratedata;
        w = [w; r.w(:, a)]; o = [o; r.u(:, a)]; z = [z; r.wd(:, a)]; dt = [dt; r.dt]; seg = [seg; k * ones(size(r.dt))]; %#ok<AGROW>
    end
    ID.(ax(a)) = identify(w, o, z, dt, seg);
    fprintf('%-5s K = %6.0f rad/s^2 per unit,  motor lag %4.1f ms,  delay %4.1f ms,  gain fit %5.1f %%,  phase error %4.1f deg,  coherence %4.2f\n', ...
        ax(a), ID.(ax(a)).Krad, 1000 * ID.(ax(a)).tm, 1000 * ID.(ax(a)).tau, ID.(ax(a)).fit, ID.(ax(a)).phaseErr, ID.(ax(a)).fitVal);
    fprintf('       band %.1f-%.1f Hz, %d coherent bins, %d flights\n', ID.(ax(a)).fband, nnz(ID.(ax(a)).band), ID.(ax(a)).nseg);
end
F.ID = ID;

% ---- loop analysis
fprintf('\n================ RATE-LOOP MARGINS (identified plant) ================\n');
tunes = struct('name', {"As flown 9 Nov", "As flown 11-12 Nov", "Tune C"}, ...
    'P', {[0.135 0.135 0.135], [0.13 0.13 0.13], [0.135 0.135 0.34]}, ...
    'I', {[0.135 0.135 0.0704], [0.14 0.14 0.135], [0.135 0.135 0.034]}, ...
    'D', {[0.0044 0.0068 0], [0.004 0.004 0], [0.0044 0.0034 0]}, ...
    'gyro', {74, 91, 74});
rows = {};
for t = 1:numel(tunes)
    for a = 1:3
        [Gm, Pm, Wc, ok] = margins(ID.(ax(a)), tunes(t), a);
        mc = monteCarlo(ID.(ax(a)), tunes(t), a, 300);
        rows(end+1, :) = {tunes(t).name, ax(a), Wc / (2*pi), Pm, Gm, ok, mc}; %#ok<AGROW>
    end
end
MT = cell2table(rows, 'VariableNames', ["Tune", "Axis", "Crossover_Hz", "PhaseMargin_deg", "GainMargin_dB", "MeetsSpec", "MonteCarloPoorPct"]);
disp(MT);
writetable(MT, fullfile(opts.OutDir, "loop_margins.csv"));
F.MT = MT; F.tunes = tunes;
save(fullfile(opts.OutDir, "fleet.mat"), 'F');

plotFleet(F, opts.OutDir, vis);
end

%% =====================================================================
function [lg, fl] = analyseLog(f, spec)
lo = ardupilotreader(f);
av = string(lo.AvailableMessages.MsgName);
want = intersect(["ATT","RATE","RCOU","CTUN","VIBE","XKF4","MAG","GPS","BAT","MODE","EV","PARM","PM","PSCN","PSCE","IMU"], av);
T = readMessages(lo, MessageNames=cellstr(want(:)'));
G = @(n, inst) getm(T, n, inst);
tt = @(x) seconds(x.Properties.RowTimes);
P = G("PARM", 0); pv = @(n, d) paramLast(P, n, d);

lg.log = string(spec.file); lg.note = spec.note; lg.length_s = seconds(lo.EndTime - lo.StartTime);
lg.firmware = "ArduCopter 4.6.3";
lg.tune_P = pv("ATC_RAT_RLL_P", NaN); lg.tune_YawP = pv("ATC_RAT_YAW_P", NaN); lg.gyro_filter = pv("INS_GYRO_FILTER", NaN);
lg.compass_ofs = sprintf('%.0f/%.0f/%.0f', pv("COMPASS_OFS_X", NaN), pv("COMPASS_OFS_Y", NaN), pv("COMPASS_OFS_Z", NaN));
mg = G("MAG", 0); lg.mag_ground = NaN;
if ~isempty(mg)
    Fm = sqrt(double(mg.MagX).^2 + double(mg.MagY).^2 + double(mg.MagZ).^2); if median(Fm) < 5, Fm = 1000 * Fm; end
    lg.mag_ground = median(Fm);
end

% flights from events (28 = took off, 18 = landed); fallback none
fl = struct([]);
ev = G("EV", 0);
if isempty(ev), lg.flights = 0; return; end
te = tt(ev); id = double(ev.Id);
to = te(id == 28); ld = te(id == 18);
spans = zeros(0, 2);
for k = 1:numel(to)
    e = ld(find(ld > to(k), 1)); if isempty(e), e = seconds(lo.EndTime); end
    spans(end+1, :) = [to(k) e]; %#ok<AGROW>
end
% remove cut windows
keep = zeros(0, 2);
for k = 1:size(spans, 1)
    s = spans(k, :);
    for c = 1:size(spec.cut, 1)
        if s(1) < spec.cut(c, 1) && s(2) > spec.cut(c, 1), s(2) = spec.cut(c, 1); end
        if s(1) >= spec.cut(c, 1) && s(1) < spec.cut(c, 2), s(1) = min(spec.cut(c, 2), s(2)); end
    end
    if diff(s) > 5, keep(end+1, :) = s; end %#ok<AGROW>
end
lg.flights = size(keep, 1);

% motor channels
mch = strings(1, 4);
for n = 1:4
    for c = 1:16, if pv("SERVO" + c + "_FUNCTION", 0) == 32 + n, mch(n) = "C" + c; break; end, end
    if mch(n) == "", mch(n) = "C" + n; end
end

for k = 1:size(keep, 1)
    s = keep(k, :); s(1) = s(1) + 3;           % skip the take-off transient
    if diff(s) < 5, continue; end
    in = @(x) tt(x) >= s(1) & tt(x) <= s(2);
    r.id = string(spec.file) + sprintf(" F%d", k);
    r.dur_s = diff(s) + 3;
    % modes
    md = G("MODE", 0); r.modes = "";
    if ~isempty(md)
        mt = tt(md); mm = double(md.Mode);
        cur = mm(find(mt <= s(1), 1, 'last')); if isempty(cur), cur = mm(1); end
        r.modes = strjoin(unique([modeName(cur), arrayfun(@modeName, mm(mt > s(1) & mt < s(2)))'], 'stable'), "/");
    end
    % vibration / clipping
    v = G("VIBE", 0);
    vm = max([v.VibeX(in(v)) v.VibeY(in(v)) v.VibeZ(in(v))], [], 2);
    r.vibe_p95 = prctile(vm, 95); r.vibe_z_med = median(v.VibeZ(in(v)));
    c = double(v.Clip(in(v))); r.clips = max(c) - min(c);
    % attitude tracking
    at = G("ATT", 0); a = in(at);
    r.att_rms_roll = rms(double(at.DesRoll(a) - at.Roll(a)));
    r.att_rms_pitch = rms(double(at.DesPitch(a) - at.Pitch(a)));
    % rate tracking
    rt = G("RATE", 0); a = in(rt); trt = tt(rt);
    r.rate_hz = 1 / median(diff(trt));
    r.rate_rms_roll = rms(double(rt.RDes(a) - rt.R(a)));
    r.rate_rms_pitch = rms(double(rt.PDes(a) - rt.P(a)));
    r.rate_rms_yaw = rms(double(rt.YDes(a) - rt.Y(a)));
    % controller effort (output RMS, high-frequency part)
    u = double([rt.ROut(a) rt.POut(a) rt.YOut(a)]);
    r.ctrl_out_rms = rms(u(:, 1:2) - movmean(u(:, 1:2), max(3, round(0.5 * r.rate_hz))), 'all');
    % hover and altitude hold
    ct = G("CTUN", 0); a = in(ct);
    r.hover_thr = median(double(ct.ThO(a)));
    hold = a;
    if ~isempty(md), hold = a & inModes(tt(ct), md, [2 5 16]); end
    r.alt_hold_rms = rms(double(ct.DAlt(hold) - ct.Alt(hold)));
    % position hold (PHLD/LOITER) from position controller
    r.pos_hold_rms = NaN;
    pn = G("PSCN", 0); pe = G("PSCE", 0);
    if ~isempty(pn) && ~isempty(md)
        h = in(pn) & inModes(tt(pn), md, [5 16]);
        pe2 = interp1(tt(pe), double(pe.TPE) - double(pe.PE), tt(pn), 'nearest', 'extrap');
        if nnz(h) > 20, r.pos_hold_rms = rms(hypot(double(pn.TPN(h)) - double(pn.PN(h)), pe2(h))); end
    end
    % motors
    rc = G("RCOU", 0); a = in(rc);
    m = double([rc.(mch(1))(a) rc.(mch(2))(a) rc.(mch(3))(a) rc.(mch(4))(a)]);
    r.motor_spread = max(mean(m)) - min(mean(m));
    r.motor_sat_pct = 100 * mean(any(m >= 1940, 2));
    % EKF
    x4 = G("XKF4", 0); a = in(x4); sc = 1; if all(double(x4.SV) == round(double(x4.SV))), sc = 0.01; end
    r.ekf_vel = sc * prctile(double(x4.SV(a)), 95); r.ekf_pos = sc * prctile(double(x4.SP(a)), 95);
    r.ekf_hgt = sc * prctile(double(x4.SH(a)), 95); r.ekf_mag = sc * prctile(double(x4.SM(a)), 95);
    % compass in flight vs ground
    r.mag_change_pct = NaN;
    if ~isempty(mg)
        tm = tt(mg); Fm = sqrt(double(mg.MagX).^2 + double(mg.MagY).^2 + double(mg.MagZ).^2); if median(Fm) < 5, Fm = 1000 * Fm; end
        gnd = tm < s(1) - 5 & tm > s(1) - 60; if ~any(gnd), gnd = tm < s(1) - 3; end
        r.mag_change_pct = 100 * (median(Fm(tm >= s(1) & tm <= s(2))) / median(Fm(gnd)) - 1);
    end
    % battery
    bt = G("BAT", 0); a = in(bt);
    r.batt_start = double(bt.Volt(find(a, 1))); r.batt_min = min(double(bt.Volt(a)));
    % CPU
    pm = G("PM", 0); r.cpu_load = NaN; r.long_loop_pct = NaN;
    if ~isempty(pm) && any(in(pm))
        a = in(pm); r.cpu_load = mean(double(pm.Load(a))) / 10;
        r.long_loop_pct = 100 * sum(double(pm.NLon(a))) / max(1, sum(double(pm.NL(a))));
    end
    % data for identification
    a = in(rt);
    r.ratedata = struct('w', deg2rad(double([rt.R(a) rt.P(a) rt.Y(a)])), 'u', double([rt.ROut(a) rt.POut(a) rt.YOut(a)]), 'wd', deg2rad(double([rt.RDes(a) rt.PDes(a) rt.YDes(a)])), 'dt', [diff(trt(a)); 1 / r.rate_hz]);
    fprintf('  %s: %.0f s %s | vibe %.1f clips %d | att %.2f/%.2f deg | rate %.1f/%.1f/%.1f deg/s | hover %.3f | althold %.3f m | pos %.2f m | EKF v/p/h/m %.2f/%.2f/%.2f/%.2f | mag %+.0f%%\n', ...
        r.id, r.dur_s, r.modes, r.vibe_p95, r.clips, r.att_rms_roll, r.att_rms_pitch, r.rate_rms_roll, r.rate_rms_pitch, r.rate_rms_yaw, ...
        r.hover_thr, r.alt_hold_rms, r.pos_hold_rms, r.ekf_vel, r.ekf_pos, r.ekf_hgt, r.ekf_mag, r.mag_change_pct);
    fl = [fl, r]; %#ok<AGROW>
end
end

%% =====================================================================
function M = identify(w, u, z, dt, seg)
% Closed-loop frequency-response identification (indirect method, standard for flight
% test data): with the commanded rate z as the external excitation,
%     P(f) = S_wz(f) / S_uz(f)     (controller output u -> body rate w)
% is unbiased by the feedback. Only frequencies where both z->w and z->u are coherent
% (>= 0.6) are used. Model fitted to the complex response:
%     P(s) = K e^{-tau s} / (s + D)
fs = 1 / median(dt); nw = round(4 * fs);
fax = []; Swz = 0; Suz = 0; Szz = 0; Sww = 0; Suu = 0; nseg = 0;
for k = unique(seg)'
    m = seg == k;
    if nnz(m) < 3 * nw, continue; end
    x = detrend(z(m)); y = detrend(w(m)); v = detrend(u(m));
    [a1, fax] = cpsd(x, y, hann(nw), nw / 2, nw, fs);
    a2 = cpsd(x, v, hann(nw), nw / 2, nw, fs);
    Swz = Swz + a1; Suz = Suz + a2;
    Szz = Szz + pwelch(x, hann(nw), nw / 2, nw, fs); Sww = Sww + pwelch(y, hann(nw), nw / 2, nw, fs); Suu = Suu + pwelch(v, hann(nw), nw / 2, nw, fs);
    nseg = nseg + 1;
end
cw = abs(Swz).^2 ./ (Szz .* Sww); cu = abs(Suz).^2 ./ (Szz .* Suu);
P = Swz ./ Suz;
band = fax >= 0.5 & fax <= 15 & cw >= 0.6 & cu >= 0.6;
M = struct('f', fax, 'P', P, 'coh', min(cw, cu), 'band', band, 'nseg', nseg);
if nnz(band) < 4
    band = fax >= 0.5 & fax <= 15 & cw >= 0.4 & cu >= 0.4;     % relax if excitation is weak
    M.band = band;
end
% Model  P(s) = K e^{-tau s} / ( s (tm s + 1) )   rigid body + first-order motor/ESC lag.
% K and tm from the gain over 1.5-16 Hz (coherent bins); delay tau from the phase over
% 6-16 Hz only: below ~5 Hz the commanded rate comes from the attitude loop, which reacts
% to the same disturbances, so low-frequency phase is biased.
band = fax >= 1.5 & fax <= 16 & cw >= 0.6 & cu >= 0.6;
if nnz(band) < 6, band = fax >= 1.5 & fax <= 16 & cw >= 0.4 & cu >= 0.4; end
M.band = band;
if nnz(band) < 4
    M.Krad = NaN; M.K = NaN; M.tm = NaN; M.tau = NaN; M.D = 0; M.fit = NaN; M.phaseErr = NaN; M.fitVal = NaN; M.fband = [NaN NaN];
    return
end
fb = fax(band); Pb = P(band); om = 2 * pi * fb;
best = struct('err', inf);
for tm = 0.005:0.001:0.08
    lk = log(abs(Pb)) + log(om) + 0.5 * log(1 + (om * tm).^2);
    K = exp(mean(lk)); err = std(lk);
    if err < best.err, best = struct('err', err, 'K', K, 'tm', tm); end
end
hf = band & fax >= 6;
om6 = 2 * pi * fax(hf);
ph0 = -pi / 2 - atan(om6 * best.tm);
res = angle(exp(1i * (angle(P(hf)) - ph0)));                  % wrapped residual (rad)
tau = max(0, -sum(res .* om6) / sum(om6 .^ 2));
g = best.K * exp(-1i * om * tau) ./ (1i * om .* (1 + 1i * om * best.tm));
M.Krad = best.K; M.K = rad2deg(best.K); M.tm = best.tm; M.tau = tau; M.D = 0;
M.fit = 100 * (1 - norm(abs(Pb) - abs(g)) / norm(abs(Pb)));        % gain fit
phe = rad2deg(angle(exp(1i * (angle(P(hf)) - angle(best.K * exp(-1i * om6 * tau) ./ (1i * om6 .* (1 + 1i * om6 * best.tm)))))));
M.phaseErr = rms(phe);
M.fitVal = mean(M.coh(band)); M.fband = [min(fb) max(fb)];
end
function f = freeRunFit(W, U, S, V, a, b, c0, d, N, useVal)
% simulate N-step windows from the measured state, compare with measured rate
e = []; y = [];
for k = unique(S)'
    m = find(S == k & logical(V) == useVal);
    for s0 = 1:N:(numel(m) - N)
        idx = m(s0:s0 + N - 1);
        if any(idx - d < 1), continue; end
        x = W(idx(1)); xs = zeros(N, 1); xs(1) = x;
        for j = 2:N, x = a * x + b * U(idx(j - 1) - d) + c0; xs(j) = x; end
        e = [e; W(idx) - xs]; y = [y; W(idx) - mean(W(idx))]; %#ok<AGROW>
    end
end
f = 100 * (1 - norm(e) / norm(y));
end
function f = fitPct(y, yh)
f = 100 * (1 - norm(y - yh) / norm(y - mean(y)));
end

function [Gm, Pm, Wc, ok] = margins(id, tune, a)
L = openLoop(id, tune, a, 1, 1);
[gm, pm, ~, wc] = margin(L);
Gm = 20 * log10(gm); Pm = pm; Wc = wc;
ok = Gm >= 6 && Pm >= 45;
end

function pct = monteCarlo(id, tune, a, n)
rng(7); bad = 0;
for i = 1:n
    kS = exp(0.3 * randn); tS = max(0.3, 1 + 0.5 * randn);   % +-30 % authority, +-50 % delay
    L = openLoop(id, tune, a, kS, tS);
    [gm, pm] = margin(L);
    if 20 * log10(gm) < 6 || pm < 45 || ~isfinite(gm), bad = bad + 1; end
end
pct = 100 * bad / n;
end

function L = openLoop(id, tune, a, kS, tS)
s = tf('s');
K = id.Krad * kS; tau = id.tau * tS;
% identified from filtered gyro, so the plant already contains the gyro filter
plant = K / (s * (id.tm * s + 1)) * exp(-tau * s);
wd = 2 * pi * 20;
C = tune.P(a) + tune.I(a) / s + tune.D(a) * s * wd / (s + wd);  % ArduPilot PID, D filter 20 Hz
L = C * plant;
end

function m = inModes(t, md, modes)
mt = seconds(md.Properties.RowTimes); mm = double(md.Mode);
cur = interp1(mt, mm, t, 'previous', 'extrap');
m = ismember(cur, modes);
end

function x = getm(T, n, inst)
x = [];
k = find(string(T.MsgName) == n);
if isempty(k), return; end
if numel(k) > 1 && ismember('InstanceID', T.Properties.VariableNames)
    j = find(string(T.MsgName) == n & T.InstanceID == inst, 1); if ~isempty(j), k = j; end
end
x = T.MsgData{k(1)};
end

function v = paramLast(P, n, d)
v = d; if isempty(P), return; end
k = find(string(P.Name) == n, 1, 'last'); if ~isempty(k), v = double(P.Value(k)); end
end

function n = modeName(m)
k = [0 1 2 3 4 5 6 9 16 17]; v = ["STAB","ACRO","ALTH","AUTO","GUID","LOIT","RTL","LAND","PHLD","BRAKE"];
i = find(k == m, 1); if isempty(i), n = "M" + m; else, n = v(i); end
end

%% =====================================================================
function plotFleet(F, outDir, vis)
fl = F.flights; if isempty(fl), return; end
lab = strrep([fl.id], "2026-11-", ""); lab = strrep(lab, " F", "\nF"); lab = compose(lab);
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1400 820]); try theme(f, 'light'); catch, end
tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact');
title(tl, "Your FC: flight performance per flight (crash windows removed)", 'FontWeight', 'bold');
pp = {
 'vibe_p95', 'Vibration, 95th pct (m/s^2)', 30
 'att_rms_pitch', 'Pitch attitude error RMS (deg)', 3
 'rate_rms_roll', 'Roll rate error RMS (deg/s)', NaN
 'alt_hold_rms', 'Altitude-hold error RMS (m)', 0.3
 'motor_spread', 'Motor mean spread (us)', 100
 'mag_change_pct', 'Compass field change in flight (%)', 10};
for i = 1:size(pp, 1)
    ax = nexttile(tl); y = [fl.(pp{i, 1})];
    b = bar(ax, y, 0.6, 'FaceColor', [0.80 0.45 0.05], 'EdgeColor', 'none'); %#ok<NASGU>
    if ~isnan(pp{i, 3}), yline(ax, pp{i, 3}, 'r--', 'limit', 'LabelHorizontalAlignment', 'left'); end
    set(ax, 'XTick', 1:numel(fl), 'XTickLabel', lab, 'FontSize', 9); grid(ax, 'on'); title(ax, pp{i, 2});
end
exportgraphics(f, fullfile(outDir, "fleet_flights.png"), 'Resolution', 130);
if strcmp(vis, 'off'), delete(f); end

% identification check figure: measured vs model angular acceleration, roll
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1400 700]); try theme(f, 'light'); catch, end
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
title(tl, "System identification from 400 Hz flight data and rate-loop margins", 'FontWeight', 'bold');
ax3 = ["roll", "pitch", "yaw"];
for a = 1:2
    ax = nexttile(tl); id = F.ID.(ax3(a)); b = id.band; fz = id.f(b);
    g = id.Krad * exp(-1i * 2 * pi * fz * id.tau) ./ (1i * 2 * pi * fz .* (1 + 1i * 2 * pi * fz * id.tm));
    yyaxis(ax, 'left');
    semilogx(ax, fz, 20 * log10(abs(id.P(b))), 'o', 'Color', [0.35 0.35 0.35]); hold(ax, 'on');
    semilogx(ax, fz, 20 * log10(abs(g)), '-', 'Color', [0.80 0.45 0.05], 'LineWidth', 1.6); ylabel(ax, 'gain (dB)');
    yyaxis(ax, 'right');
    semilogx(ax, fz, rad2deg(unwrap(angle(id.P(b)))), 's', 'Color', [0.35 0.35 0.65]);
    semilogx(ax, fz, rad2deg(unwrap(angle(g))), '-', 'Color', [0.2 0.3 0.8], 'LineWidth', 1.2); ylabel(ax, 'phase (deg)');
    grid(ax, 'on'); xlabel(ax, 'Hz');
    legend(ax, ["measured gain", "model gain", "measured phase", "model phase"], 'Location', 'southwest');
    title(ax, sprintf('%s: controller output -> body rate, fit %.0f %%', ax3(a), id.fit));
end
ax = nexttile(tl, [1 2]);
MT = F.MT; tn = unique(MT.Tune, 'stable'); G = nan(3, numel(tn)); Pm = G;
for t = 1:numel(tn), for a = 1:3, k = MT.Tune == tn(t) & MT.Axis == ax3(a); G(a, t) = MT.GainMargin_dB(k); Pm(a, t) = MT.PhaseMargin_deg(k); end, end
yyaxis(ax, 'left'); bar(ax, categorical(ax3, ax3), G, 0.8); ylabel(ax, 'gain margin (dB)'); yline(ax, 6, 'r--', '6 dB');
yyaxis(ax, 'right'); hold(ax, 'on');
for t = 1:numel(tn), plot(ax, categorical(ax3, ax3), Pm(:, t), 'o', 'MarkerSize', 8, 'LineWidth', 1.5); end
ylabel(ax, 'phase margin (deg, markers)'); yline(ax, 45, 'k:', '45 deg');
legend(ax, [tn' + " GM", tn' + " PM"], 'Location', 'northeastoutside'); grid(ax, 'on');
title(ax, 'Rate-loop stability margins with the identified plant');
exportgraphics(f, fullfile(outDir, "fleet_identification.png"), 'Resolution', 130);
if strcmp(vis, 'off'), delete(f); end
end
