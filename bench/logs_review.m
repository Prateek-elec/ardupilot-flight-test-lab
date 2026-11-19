function S = logs_review(files, opts)
%LOGS_REVIEW Deep review of several ArduCopter flight logs: performance, compass
%   interference, motor balance and crash sequence. One figure per log + summary.
%   S = logs_review(["a.bin","b.bin"]);

arguments
    files (1,:) string
    opts.OutDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), "logs_review")
end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
S = struct([]);
for f = files
    s = reviewOne(f, opts.OutDir);
    S = [S, s]; %#ok<AGROW>
end
fn = cellfun(@fieldnames, {S.summary}, 'UniformOutput', false);
fn = unique(vertcat(fn{:}), 'stable');
V = NaN(numel(fn), numel(S));
for j = 1:numel(S)
    for i = 1:numel(fn)
        if isfield(S(j).summary, fn{i}), V(i, j) = S(j).summary.(fn{i}); end
    end
end
T = array2table(V, 'RowNames', fn, 'VariableNames', matlab.lang.makeValidName(cellstr([S.name])));
disp(T);
writetable(T, fullfile(opts.OutDir, "summary.csv"), 'WriteRowNames', true);
save(fullfile(opts.OutDir, "review.mat"), 'S');
end

function s = reviewOne(f, outDir)
[~, nm] = fileparts(f); s.name = string(nm);
fprintf('\n\n==================== %s ====================\n', nm);
lo = ardupilotreader(f);
want = {'ATT','RATE','RCOU','RCIN','CTUN','VIBE','XKF4','XKF3','MAG','GPS','BAT','MODE','ERR','MSG','PARM','PM','EV','IMU','POWR','BARO','XKY0'};
av = intersect(want, cellstr(lo.AvailableMessages.MsgName)); av = av(:)';
T = readMessages(lo, MessageNames=av);
G = @(n, i) getm(T, n, i);
t0 = lo.StartTime;
tt = @(x) seconds(x.Properties.RowTimes - t0);
fprintf('Log %.0f s, messages: %s\n', seconds(lo.EndTime - lo.StartTime), strjoin(av, ' '));

%% parameters of interest
P = G("PARM", []); pv = @(n) paramLast(P, n);
keyP = ["ATC_RAT_RLL_P","ATC_RAT_RLL_D","ATC_RAT_PIT_P","ATC_RAT_PIT_D","ATC_RAT_YAW_P","ATC_RAT_YAW_I", ...
    "RC2_REVERSED","ARMING_CHECK","LOG_BITMASK","INS_HNTCH_ENABLE","INS_GYRO_FILTER","MOT_THST_HOVER", ...
    "COMPASS_USE","COMPASS_USE2","COMPASS_ORIENT","COMPASS_EXTERNAL","COMPASS_MOTCT","COMPASS_OFS_X","COMPASS_OFS_Y","COMPASS_OFS_Z", ...
    "BATT_MONITOR","BATT_LOW_VOLT","FS_THR_ENABLE","FENCE_ENABLE","EK3_SRC1_YAW","GPS_TYPE","GPS1_TYPE"];
pvals = arrayfun(pv, keyP);
fprintf('Params: '); for i = 1:numel(keyP), if ~isnan(pvals(i)), fprintf('%s=%g  ', keyP(i), pvals(i)); end, end; fprintf('\n');
s.params = array2table(pvals, 'VariableNames', cellstr(keyP));

%% messages, errors, modes
msg = G("MSG", []);
if ~isempty(msg)
    txt = string(msg.Message);
    fprintf('--- MSG (%d):\n', numel(txt));
    tm = tt(msg);
    for i = 1:numel(txt), fprintf('  %7.1f  %s\n', tm(i), txt(i)); end
    s.msgs = table(tm, txt);
end
er = G("ERR", []);
if ~isempty(er)
    fprintf('--- ERR:\n'); te = tt(er);
    for i = 1:height(er), fprintf('  %7.1f  Subsys %d  ECode %d  (%s)\n', te(i), er.Subsys(i), er.ECode(i), errName(er.Subsys(i), er.ECode(i))); end
end
md = G("MODE", []);
if ~isempty(md)
    fprintf('--- MODE:'); tmd = tt(md);
    for i = 1:height(md), fprintf('  %.0fs:%s', tmd(i), modeName(md.Mode(i))); end; fprintf('\n');
end

%% airborne periods from CTUN (ThO>0.1 & armed) fallback RCOU
rc = G("RCOU", []); ct = G("CTUN", []);
% motor n = the RCOU channel whose SERVOx_FUNCTION is 32+n (ArduPilot motor 1..4 = 33..36)
mch = strings(1, 4);
for n = 1:4
    for c = 1:16
        if paramLast(P, "SERVO" + c + "_FUNCTION") == 32 + n, mch(n) = "C" + c; break; end
    end
    if mch(n) == "", mch(n) = "C" + n; end
end
fprintf('Motor outputs: M1..M4 on RCOU %s\n', strjoin(mch, ' '));
motors = @(r) double([r.(mch(1)) r.(mch(2)) r.(mch(3)) r.(mch(4))]);
air = []; tAir = [];
if ~isempty(rc)
    tr = tt(rc); m = motors(rc);
    spinning = mean(m, 2) > 1150;
    air = spinning; tAir = tr;
    s.summary.flight_s = sum(diff([tr; tr(end)]) .* spinning);
else
    s.summary.flight_s = 0;
end
fprintf('Motors spinning for %.0f s\n', s.summary.flight_s);
inAir = @(x) inMask(tt(x), tAir, air);

%% vibration
for k = 0:1
    v = G("VIBE", k);
    if isempty(v), continue; end
    sel = inAir(v); if ~any(sel), sel = true(height(v), 1); end
    vm = max([v.VibeX(sel) v.VibeY(sel) v.VibeZ(sel)], [], 2);
    s.summary.(sprintf('vibe%d_p95', k)) = prctile(vm, 95);
    s.summary.(sprintf('clip%d', k)) = double(max(v.Clip) - min(v.Clip));
    fprintf('VIBE IMU%d: 95th %.1f m/s^2, max %.1f, X/Y/Z median %.1f/%.1f/%.1f, clips %d\n', k, prctile(vm, 95), max(vm), ...
        median(v.VibeX(sel)), median(v.VibeY(sel)), median(v.VibeZ(sel)), max(v.Clip) - min(v.Clip));
end

%% attitude tracking
at = G("ATT", []);
if ~isempty(at)
    sel = inAir(at);
    eR = double(at.DesRoll(sel) - at.Roll(sel)); eP = double(at.DesPitch(sel) - at.Pitch(sel));
    eY = wrapTo180(double(at.DesYaw(sel) - at.Yaw(sel)));
    s.summary.att_err_roll = rms(eR); s.summary.att_err_pitch = rms(eP); s.summary.att_err_yaw = rms(eY);
    fprintf('Attitude error RMS roll %.2f, pitch %.2f, yaw %.2f deg (max %.1f / %.1f / %.1f)\n', rms(eR), rms(eP), rms(eY), max(abs(eR)), max(abs(eP)), max(abs(eY)));
end

%% motors
if ~isempty(rc)
    sel = air; m = motors(rc); m = m(sel, :);
    if ~isempty(m)
        mmax = max(m(:));
        fprintf('Motors (airborne) mean %s, std %s, %% time >= 1900: %s, max %d\n', mat2str(round(mean(m)), 4), mat2str(round(std(m)), 3), ...
            mat2str(round(100 * mean(m >= 1900), 1)), mmax);
        s.summary.motor_mean_spread = max(mean(m)) - min(mean(m));
        s.summary.motor_sat_pct = 100 * mean(any(m >= 1950, 2));
        % pairs: roll (M2+M3)-(M1+M4); pitch (M2+M4)-(M1+M3); yaw (M1+M2)-(M3+M4)
        mm = mean(m);
        fprintf('  trim offsets: roll %+.0f, pitch %+.0f, yaw %+.0f us (left-heavy +, nose-heavy... see note)\n', ...
            (mm(2) + mm(3) - mm(1) - mm(4)) / 2, (mm(2) + mm(4) - mm(1) - mm(3)) / 2, (mm(1) + mm(2) - mm(3) - mm(4)) / 2);
    end
end

%% throttle / hover
if ~isempty(ct)
    sel = inAir(ct);
    if any(sel)
        s.summary.hover_thr = median(double(ct.ThO(sel)));
        fprintf('Throttle out median %.3f (ThH %.3f), altitude max %.1f m\n', median(double(ct.ThO(sel))), median(double(ct.ThH(sel))), max(double(ct.Alt)));
    end
end

%% EKF
x4 = G("XKF4", 0);
if ~isempty(x4)
    sc = 1; if max(abs(double(x4.SV))) > 20, sc = 0.01; end
    sel = inAir(x4); if ~any(sel), sel = true(height(x4), 1); end
    for c = ["SV","SP","SH","SM"]
        v = sc * double(x4.(c)(sel));
        s.summary.("ekf_" + c + "_p95") = prctile(v, 95);
        fprintf('EKF %s: 95th %.2f, max %.2f\n', c, prctile(v, 95), max(v));
    end
    if ismember('SS', x4.Properties.VariableNames), fprintf('EKF SS (status bits) distinct: %s\n', mat2str(unique(double(x4.SS))')); end
end

%% compass: field strength vs throttle and current
for k = 0:1
    mg = G("MAG", k);
    if isempty(mg), continue; end
    F = sqrt(double(mg.MagX).^2 + double(mg.MagY).^2 + double(mg.MagZ).^2);
    if median(F) < 5, F = 1000 * F; end      % logged in Gauss -> mGauss
    tm = tt(mg); selA = inMask(tm, tAir, air); selG = ~selA;
    fprintf('MAG%d: field on ground %.0f mGauss, in air median %.0f (min %.0f max %.0f)\n', k, median(F(selG)), median(F(selA)), min(F), max(F));
    s.summary.(sprintf('mag%d_ground', k)) = median(F(selG));
    s.summary.(sprintf('mag%d_air', k)) = median(F(selA));
    if ~isempty(ct)
        thr = interp1(tt(ct), double(ct.ThO), tm, 'nearest', 'extrap');
        r = NaN; if nnz(selA) > 10, r = corr(F(selA), thr(selA), 'Rows', 'complete'); end
        fprintf('  correlation field vs throttle (airborne): %.2f\n', r);
        s.summary.(sprintf('mag%d_thr_corr', k)) = r;
        % % change of field from idle to hover
        if any(selA) && any(selG)
            s.summary.(sprintf('mag%d_change_pct', k)) = 100 * (median(F(selA)) - median(F(selG))) / median(F(selG));
            fprintf('  field change ground -> air: %+.0f %%\n', s.summary.(sprintf('mag%d_change_pct', k)));
        end
    end
end
m0 = G("MAG", 0); m1 = G("MAG", 1);
if ~isempty(m0) && ~isempty(m1)
    h0 = atan2d(-double(m0.MagY), double(m0.MagX)); h1 = interp1(tt(m1), atan2d(-double(m1.MagY), double(m1.MagX)), tt(m0), 'nearest', 'extrap');
    d = wrapTo180(h0 - h1);
    fprintf('Compass 1 vs 2 raw heading difference: median %.0f deg, spread (std) %.0f deg\n', median(d), std(d));
end

%% GPS
gp = G("GPS", 0);
if ~isempty(gp)
    sel = double(gp.Status) >= 3;
    fprintf('GPS: 3D fix %.0f %% of log, sats median %d (min %d), HDop median %.2f\n', 100 * mean(sel), median(gp.NSats), min(gp.NSats), median(double(gp.HDop)));
    s.summary.gps_sats = median(double(gp.NSats)); s.summary.gps_hdop = median(double(gp.HDop));
end

%% battery
bt = G("BAT", 0);
if ~isempty(bt)
    sel = inAir(bt); if ~any(sel), sel = true(height(bt), 1); end
    fprintf('Battery: %.2f V at start, min %.2f V airborne, current median %.1f A max %.1f A\n', bt.Volt(1), min(double(bt.Volt(sel))), median(double(bt.Curr(sel))), max(double(bt.Curr(sel))));
    s.summary.batt_min = min(double(bt.Volt(sel)));
end

%% crash / end-of-flight sequence
s.crash = [];
if ~isempty(at) && ~isempty(rc)
    big = abs(double(at.Roll)) > 60 | abs(double(at.Pitch)) > 60;
    ta = tt(at);
    k = find(big & inMask(ta, tAir, air), 1);
    if ~isempty(k)
        tc = ta(k);
        fprintf('\n*** Attitude exceeded 60 deg at t = %.2f s (roll %.0f, pitch %.0f) ***\n', tc, at.Roll(k), at.Pitch(k));
        tr = tt(rc); w = tr > tc - 4 & tr < tc + 1.5; m = motors(rc);
        fprintf('  t(s)     M1    M2    M3    M4   | roll  pitch  desR  desP\n');
        idx = find(w); idx = idx(1:max(1, round(numel(idx) / 30)):end);
        for i = idx'
            j = find(ta >= tr(i), 1); if isempty(j), j = numel(ta); end
            fprintf('  %6.2f  %5d %5d %5d %5d   | %5.0f %5.0f %5.0f %5.0f\n', tr(i) - tc, m(i, :), at.Roll(j), at.Pitch(j), at.DesRoll(j), at.DesPitch(j));
        end
        s.crash = struct('t', tc, 'window', w);
    end
end

%% figure
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
fg = figure('Visible', vis, 'Color', 'w', 'Position', [30 30 1400 950]); try theme(fg, 'light'); catch, end
tl = tiledlayout(fg, 5, 1, 'TileSpacing', 'compact'); title(tl, "Flight log " + nm, 'Interpreter', 'none');
ax = gobjects(5, 1);
ax(1) = nexttile(tl); hold on; grid on;
if ~isempty(ct), plot(tt(ct), double(ct.Alt), 'DisplayName', 'altitude (m)'); plot(tt(ct), 10 * double(ct.ThO), 'DisplayName', 'throttle x10'); end
if ~isempty(md), xline(tt(md), ':', arrayfun(@modeName, md.Mode), 'HandleVisibility', 'off', 'LabelOrientation', 'horizontal', 'FontSize', 8); end
legend('Location', 'northwest'); title('Altitude, throttle and flight modes');
ax(2) = nexttile(tl); hold on; grid on;
if ~isempty(at), plot(tt(at), double(at.Roll), 'DisplayName', 'roll'); plot(tt(at), double(at.DesRoll), '--', 'DisplayName', 'roll target');
    plot(tt(at), double(at.Pitch), 'DisplayName', 'pitch'); plot(tt(at), double(at.DesPitch), '--', 'DisplayName', 'pitch target'); end
ylim([-60 60]); legend('Location', 'northwest'); ylabel('deg'); title('Attitude vs target');
ax(3) = nexttile(tl); hold on; grid on;
if ~isempty(rc), plot(tt(rc), motors(rc)); legend(["M1 FR", "M2 RL", "M3 FL", "M4 RR"], 'Location', 'northwest'); end
ylabel('PWM us'); title('Motor outputs');
ax(4) = nexttile(tl); hold on; grid on;
v = G("VIBE", 0); if ~isempty(v), plot(tt(v), double([v.VibeX v.VibeY v.VibeZ])); legend(["X", "Y", "Z"], 'Location', 'northwest'); yline(30, 'r--', 'limit 30', 'HandleVisibility', 'off'); end
ylabel('m/s^2'); title('Vibration IMU0');
ax(5) = nexttile(tl); hold on; grid on;
for k = 0:1
    mg = G("MAG", k); if isempty(mg), continue; end
    Fp = sqrt(double(mg.MagX).^2 + double(mg.MagY).^2 + double(mg.MagZ).^2); if median(Fp) < 5, Fp = 1000 * Fp; end
    plot(tt(mg), Fp, 'DisplayName', sprintf('compass %d field', k + 1));
end
if ~isempty(bt), yyaxis right; plot(tt(bt), double(bt.Curr), 'DisplayName', 'current (A)'); ylabel('A'); yyaxis left; end
ylabel('mGauss'); legend('Location', 'northwest'); title('Compass field strength vs battery current'); xlabel('time from log start (s)');
linkaxes(ax, 'x');
if ~isempty(s.crash), for a = ax', xline(a, s.crash.t, 'r-', 'HandleVisibility', 'off'); end, end
p = fullfile(outDir, nm + ".png"); exportgraphics(fg, p, 'Resolution', 100); s.figure = p;
if ~isempty(s.crash)
    xlim(ax(1), s.crash.t + [-6 2]); exportgraphics(fg, fullfile(outDir, nm + "_crash_zoom.png"), 'Resolution', 100);
end
if strcmp(vis, 'off'), delete(fg); end
end

function x = getm(T, n, inst)
x = [];
k = find(string(T.MsgName) == n);
if isempty(k), return; end
x = T.MsgData{k(1)};
if ~isempty(inst)
    v = x.Properties.VariableNames;
    if ismember('I', v), x = x(double(x.I) == inst, :);
    elseif ismember('C', v), x = x(double(x.C) == inst, :);
    elseif ismember('Instance', v), x = x(double(x.Instance) == inst, :);
    elseif inst > 0, x = []; end
end
end

function v = paramLast(P, n)
v = NaN; if isempty(P), return; end
k = find(string(P.Name) == n, 1, 'last'); if ~isempty(k), v = double(P.Value(k)); end
end

function m = inMask(t, tAir, air)
if isempty(tAir), m = true(size(t)); return; end
m = interp1(tAir, double(air), t, 'previous', 0) > 0.5;
end

function n = modeName(m)
names = containers.Map('KeyType', 'double', 'ValueType', 'char');
k = [0 1 2 3 4 5 6 7 9 11 13 15 16 17 18 21]; v = {'STAB','ACRO','ALTH','AUTO','GUID','LOIT','RTL','CIRC','LAND','DRIFT','SPORT','ATUN','PHLD','BRAKE','THROW','SMRTL'};
for i = 1:numel(k), names(k(i)) = v{i}; end
if isKey(names, double(m)), n = string(names(double(m))); else, n = "M" + m; end
end

function s = errName(sub, code)
subs = containers.Map({2,3,5,6,8,10,11,12,13,16,17,18,22,24,25}, {'Radio','Compass','Radio FS','Battery FS','GCS FS','Flight mode','GPS','Crash check','Flip','EKF check','EKF FS','Baro','EKF primary','Thrust loss','Yaw imbalance'});
if isKey(subs, double(sub)), s = subs(double(sub)); else, s = sprintf('subsys %d', sub); end
if code == 0, s = s + " cleared"; end
end
