function report = fc_log_analysis(binFile, opts)
%FC_LOG_ANALYSIS Pass/fail health report from an ArduPilot DataFlash log (.BIN).
%
%   Checks vibration, clipping, CPU load, 5 V rail, battery sensor and sag,
%   EKF health, compass interference, attitude tracking, GPS and IMU noise.
%
%   Get a log: Mission Planner -> DATA -> DataFlash Logs ->
%              "Download DataFlash Log Via Mavlink" (or copy from the SD card).
%
%   USAGE
%     report = fc_log_analysis("00000012.BIN");
%     report = fc_log_analysis("00000012.BIN", Plot=false);
%     report = fc_log_analysis();   % opens a file picker
%
%   Limits follow ArduPilot guidance; tune them for your airframe.

arguments
    binFile (1,1) string = ""
    opts.Plot (1,1) logical = true
end

if binFile == ""
    [f, p] = uigetfile({'*.bin;*.BIN', 'ArduPilot logs (*.bin)'}, 'Select a DataFlash log');
    if isequal(f, 0), report = []; return; end
    binFile = string(fullfile(p, f));
end

fprintf('\nReading %s ...\n', binFile);
logObj = ardupilotreader(binFile);
want = {'VIBE','PM','POWR','BAT','XKF4','MAG','ATT','GPS','IMU','RCOU','MODE','ERR','ARM'};
avail = intersect(want, cellstr(logObj.AvailableMessages.MsgName));
avail = avail(:)';   % readMessages needs a row cell array
T = readMessages(logObj, MessageNames=avail);
fprintf('Log length %s, %d messages.\n', string(logObj.EndTime - logObj.StartTime), logObj.NumMessages);

R = table('Size', [0 4], 'VariableTypes', ["string","string","string","string"], ...
          'VariableNames', ["Check","Measured","Limit","Result"]);

%% Arming / flying periods from motor outputs
[armedAt, flyingAt] = motorState(getMsg(T, "RCOU", 0));
hasFlight = ~isempty(flyingAt);
if isempty(armedAt)
    fprintf('Motor outputs (RCOU) not found - treating the whole log as one period.\n');
end
R = addRow(R, "Flight detected", ternary(hasFlight, "yes", "no (bench log)"), "-", "INFO");

%% Vibration and clipping (per IMU)
for inst = 0:2
    v = getMsg(T, "VIBE", inst);
    if isempty(v), continue; end
    sel = inPeriod(v, flyingAt, hasFlight);
    vmax = prctile(max([v.VibeX(sel) v.VibeY(sel) v.VibeZ(sel)], [], 2), 95);
    R = addRow(R, sprintf("Vibration IMU%d (95th pct)", inst), sprintf('%.1f m/s^2', vmax), "< 30 (warn < 60)", grade(vmax, 30, 60));
    clips = max(v.Clip) - min(v.Clip);
    R = addRow(R, sprintf("Accel clipping IMU%d", inst), sprintf('%d', clips), "0", ternary(clips == 0, "PASS", "FAIL"));
end

%% CPU / scheduler
pm = getMsg(T, "PM", 0);
if ~isempty(pm)
    load = prctile(double(pm.Load), 95) / 10;
    R = addRow(R, "CPU load (95th pct)", sprintf('%.0f %%', load), "< 70 % (warn < 85)", grade(load, 70, 85));
    longPct = 100 * sum(double(pm.NLon)) / max(1, sum(double(pm.NL)));
    R = addRow(R, "Long loops", sprintf('%.2f %%', longPct), "< 5 % (warn < 15)", grade(longPct, 5, 15));
end

%% 5 V rail
pw = getMsg(T, "POWR", 0);
if ~isempty(pw) && ~all(isnan(double(pw.Vcc)) | double(pw.Vcc) < 0.5)
    vcc = double(pw.Vcc);
    R = addRow(R, "Board 5V rail min / max", sprintf('%.2f / %.2f V', min(vcc), max(vcc)), "4.8 - 5.4 V", ...
               ternary(min(vcc) >= 4.8 && max(vcc) <= 5.4, "PASS", ternary(min(vcc) >= 4.6, "WARN", "FAIL")));
end

%% Battery
bat = getMsg(T, "BAT", 0);
if ~isempty(bat)
    disarmedSel = ~inPeriod(bat, armedAt, ~isempty(armedAt));
    if any(disarmedSel)
        iIdle = median(double(bat.Curr(disarmedSel)));
        R = addRow(R, "Current while disarmed", sprintf('%.2f A', iIdle), "< 1 A (warn < 3)", grade(iIdle, 1, 3));
    end
    if hasFlight
        fl = inPeriod(bat, flyingAt, true);
        vRest = median(double(bat.Volt(disarmedSel | ~fl)));
        vMin = min(double(bat.Volt(fl)));
        sag = 100 * (vRest - vMin) / max(vRest, eps);
        R = addRow(R, "Battery sag under load", sprintf('%.1f %% (%.2f -> %.2f V)', sag, vRest, vMin), "< 15 % (warn < 25)", grade(sag, 15, 25));
        R = addRow(R, "Max current in flight", sprintf('%.1f A', max(double(bat.Curr(fl)))), "-", "INFO");
    end
end

%% EKF health (primary core)
k4 = getMsg(T, "XKF4", 0);
if ~isempty(k4)
    names = ["SV","SP","SH","SM"]; lbl = ["velocity","position","height","compass"];
    sel = inPeriod(k4, armedAt, ~isempty(armedAt));   % ignore EKF start-up before arming
    % Older ArduPilot logs store the ratios as integers x100; current (4.x) logs use floats.
    scale = 1;
    allInt = all(arrayfun(@(n) all(double(k4.(n)) == round(double(k4.(n)))), names));
    if allInt, scale = 0.01; fprintf('XKF4 uses the older x100 integer format - scaling by 0.01.\n'); end
    for j = 1:4
        val = prctile(double(k4.(names(j))(sel)), 95) * scale;
        R = addRow(R, "EKF " + lbl(j) + " test ratio", sprintf('%.2f', val), "< 0.5 (warn < 1)", grade(val, 0.5, 1));
    end
end

%% Compass interference vs current
mg = getMsg(T, "MAG", 0);
if ~isempty(mg) && ~isempty(bat)
    tM = seconds(mg.Properties.RowTimes); tB = seconds(bat.Properties.RowTimes);
    field = vecnorm([double(mg.MagX) double(mg.MagY) double(mg.MagZ)], 2, 2);
    curr = interp1(tB, double(bat.Curr), tM, 'linear', NaN);
    ok = ~isnan(curr);
    if any(ok) && range(curr(ok)) >= 3
        lo = field(ok & curr <= prctile(curr(ok), 10));
        hi = field(ok & curr >= prctile(curr(ok), 90));
        pct = 100 * abs(mean(hi) - mean(lo)) / mean(lo);
        R = addRow(R, "Compass interference", sprintf('%.0f %% field change', pct), "< 30 % (warn < 60)", grade(pct, 30, 60));
    else
        R = addRow(R, "Compass interference", "needs a log with throttle-up", "-", "SKIP");
    end
end

%% Attitude tracking in flight
att = getMsg(T, "ATT", 0);
if ~isempty(att) && hasFlight
    sel = inPeriod(att, flyingAt, true);
    eR = rms(double(att.DesRoll(sel) - att.Roll(sel)));
    eP = rms(double(att.DesPitch(sel) - att.Pitch(sel)));
    eY = rms(wrap180(double(att.DesYaw(sel) - att.Yaw(sel))));
    R = addRow(R, "Roll tracking error (RMS)", sprintf('%.2f deg', eR), "< 3 (warn < 6)", grade(eR, 3, 6));
    R = addRow(R, "Pitch tracking error (RMS)", sprintf('%.2f deg', eP), "< 3 (warn < 6)", grade(eP, 3, 6));
    R = addRow(R, "Yaw tracking error (RMS)", sprintf('%.1f deg', eY), "< 10 (warn < 20)", grade(eY, 10, 20));
end

%% GPS
gps = getMsg(T, "GPS", 0);
if ~isempty(gps)
    fixPct = 100 * mean(double(gps.Status) >= 3);
    R = addRow(R, "GPS 3D fix time", sprintf('%.0f %%', fixPct), "> 95 % (warn > 80)", gradeHigh(fixPct, 95, 80));
    has = double(gps.Status) >= 3;
    if any(has)
        R = addRow(R, "GPS satellites (median)", sprintf('%d', round(median(double(gps.NSats(has))))), ">= 10 (warn >= 6)", gradeHigh(median(double(gps.NSats(has))), 10, 6));
        R = addRow(R, "GPS HDOP (median)", sprintf('%.2f', median(double(gps.HDop(has)))), "< 1.5 (warn < 2.5)", grade(median(double(gps.HDop(has))), 1.5, 2.5));
    end
end

%% IMU noise while disarmed, and IMU agreement
imu0 = getMsg(T, "IMU", 0); imu1 = getMsg(T, "IMU", 1);
if ~isempty(imu0)
    % Quietest 1-second window while disarmed = sensor noise without handling/motion.
    sel = ~inPeriod(imu0, armedAt, ~isempty(armedAt));
    ti = seconds(imu0.Properties.RowTimes);
    gz = [double(imu0.GyrX) double(imu0.GyrY) double(imu0.GyrZ)];
    sec = floor(ti); u = unique(sec(sel)); s = nan(numel(u), 1);
    for j = 1:numel(u)
        m = sel & sec == u(j);
        if nnz(m) > 5, s(j) = max(std(gz(m, :))); end
    end
    if any(~isnan(s))
        gn = rad2deg(min(s));
        R = addRow(R, "Gyro noise at rest (IMU0)", sprintf('%.3f deg/s', gn), "< 0.3 (warn < 1)", grade(gn, 0.3, 1));
    end
end
if ~isempty(imu0) && ~isempty(imu1)
    d = norm(mean([double(imu0.AccX) double(imu0.AccY) double(imu0.AccZ)]) - ...
             mean([double(imu1.AccX) double(imu1.AccY) double(imu1.AccZ)]));
    R = addRow(R, "IMU0 vs IMU1 accel (mean)", sprintf('%.2f m/s^2 apart', d), "< 0.5 (warn < 1)", grade(d, 0.5, 1));
end

%% Forced arming (pre-arm checks bypassed)
arm = getMsg(T, "ARM", 0);
if ~isempty(arm) && ismember('Forced', arm.Properties.VariableNames)
    nForced = nnz(double(arm.Forced) == 1 & double(arm.ArmState) == 1);
    R = addRow(R, "Forced arms (checks bypassed)", sprintf('%d', nForced), "0", ternary(nForced == 0, "PASS", "FAIL"));
end

%% Errors and modes
er = getMsg(T, "ERR", 0);
if ~isempty(er)
    fprintf('\nERR events in log (Subsys / ECode):\n');
    disp(er(:, intersect({'Subsys','ECode'}, er.Properties.VariableNames)));
    R = addRow(R, "ERR events", sprintf('%d', height(er)), "0", "WARN");
else
    R = addRow(R, "ERR events", "0", "0", "PASS");
end
md = getMsg(T, "MODE", 0);
if ~isempty(md)
    fprintf('Flight modes used (ModeNum): %s\n', mat2str(unique(double(md.ModeNum))'));
end

%% Report
fprintf('\n================ FC LOG ANALYSIS REPORT ================\n');
disp(R);
nFail = sum(R.Result == "FAIL"); nWarn = sum(R.Result == "WARN");
fprintf('PASS %d   WARN %d   FAIL %d\n', sum(R.Result == "PASS"), nWarn, nFail);
if nFail > 0
    fprintf('Result: FIX THE FAIL ROWS BEFORE THE NEXT FLIGHT\n');
elseif nWarn > 0
    fprintf('Result: OK with warnings - review the WARN rows\n');
else
    fprintf('Result: ALL CHECKS PASSED\n');
end
report = R;

if opts.Plot
    plotLog(T, flyingAt);
end
end

%% ---------------- helpers ----------------
function tt = getMsg(T, name, inst)
row = find(T.MsgName == name & T.InstanceID == inst, 1);
if isempty(row), tt = []; else, tt = T.MsgData{row}; end
end

function [armed, flying] = motorState(rc)
% Returns [start end] rows (seconds) where motors are armed / producing lift.
armed = []; flying = [];
if isempty(rc), return; end
t = seconds(rc.Properties.RowTimes);
C = [];
for c = 1:14
    n = "C" + c;
    if ismember(n, rc.Properties.VariableNames)
        x = double(rc.(n));
        if range(x) > 150, C = [C x]; end %#ok<AGROW>
    end
end
if isempty(C), return; end
m = mean(C, 2);
armed  = toSegments(t, m > min(m) + 30);
flying = toSegments(t, m > min(m) + 250);
end

function seg = toSegments(t, mask)
d = diff([0; mask(:); 0]);
s = find(d == 1); e = find(d == -1) - 1;
seg = [t(s) t(e)];
end

function sel = inPeriod(tt, seg, useSeg)
t = seconds(tt.Properties.RowTimes);
if ~useSeg || isempty(seg), sel = true(size(t)); return; end
sel = false(size(t));
for k = 1:size(seg, 1)
    sel = sel | (t >= seg(k,1) & t <= seg(k,2));
end
end

function d = wrap180(d)
d = mod(d + 180, 360) - 180;
end

function r = grade(val, passBelow, warnBelow)
if val < passBelow, r = "PASS"; elseif val < warnBelow, r = "WARN"; else, r = "FAIL"; end
end

function r = gradeHigh(val, passAtLeast, warnAtLeast)
if val >= passAtLeast, r = "PASS"; elseif val >= warnAtLeast, r = "WARN"; else, r = "FAIL"; end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function R = addRow(R, check, measured, limit, result)
R(end+1, :) = {string(check), string(measured), string(limit), string(result)};
end

function plotLog(T, flyingAt)
figure('Name', 'FC log analysis', 'Color', 'w');
tiledlayout(3, 2, 'TileSpacing', 'compact');
v = getMsg(T, "VIBE", 0);
nexttile; if ~isempty(v), plot(v.Properties.RowTimes, [v.VibeX v.VibeY v.VibeZ]); yline(30, 'r--'); legend('X','Y','Z','limit'); end
title('Vibration (m/s^2)'); grid on
a = getMsg(T, "ATT", 0);
nexttile; if ~isempty(a), plot(a.Properties.RowTimes, [a.DesRoll a.Roll]); legend('desired','actual'); end
title('Roll (deg)'); grid on
nexttile; if ~isempty(a), plot(a.Properties.RowTimes, [a.DesPitch a.Pitch]); legend('desired','actual'); end
title('Pitch (deg)'); grid on
b = getMsg(T, "BAT", 0);
nexttile; if ~isempty(b), yyaxis left; plot(b.Properties.RowTimes, b.Volt); ylabel('V'); yyaxis right; plot(b.Properties.RowTimes, b.Curr); ylabel('A'); end
title('Battery'); grid on
k = getMsg(T, "XKF4", 0);
nexttile; if ~isempty(k), plot(k.Properties.RowTimes, [k.SV k.SP k.SH k.SM]); yline(1, 'r--'); legend('vel','pos','hgt','mag','limit'); end
title('EKF test ratios'); grid on
m = getMsg(T, "MAG", 0);
nexttile; if ~isempty(m) && ~isempty(b)
    f = vecnorm([double(m.MagX) double(m.MagY) double(m.MagZ)], 2, 2);
    c = interp1(seconds(b.Properties.RowTimes), double(b.Curr), seconds(m.Properties.RowTimes));
    scatter(c, f, 4, 'filled'); xlabel('current (A)'); ylabel('|mag| (mGauss)');
end
title('Compass vs current'); grid on
if ~isempty(flyingAt)
    sgtitle(sprintf('Flight segments: %d', size(flyingAt, 1)));
end
end
