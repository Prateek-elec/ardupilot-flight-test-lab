function T = fc_compare(logs, labels, opts)
%FC_COMPARE Compare flight controllers from their ArduPilot DataFlash logs.
%
%   T = fc_compare(["my_fc.bin","pixhawk.bin"], ["Your FC","Pixhawk 2.4.8"]);
%
%   Metrics are split into two groups:
%     BOARD     - mostly set by the flight controller itself: sensor noise at
%                 rest, IMU agreement, barometer noise, CPU load and loop
%                 timing, 5 V rail, EKF consistency.
%     AIRFRAME  - mostly set by frame, props, mounting and tune: vibration,
%                 clipping, attitude tracking. Only meaningful between logs
%                 from the same drone.
%   For a strict head-to-head, fly both boards on the same frame with the same
%   parameters (see fc_compare_procedure.txt).

arguments
    logs (1,:) string
    labels (1,:) string = logs
    opts.OutDir (1,1) string = fullfile(fileparts(mfilename('fullpath')), "results")
end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
n = numel(logs);
M = cell(1, n);
for k = 1:n
    fprintf('\n=== %s : %s\n', labels(k), logs(k));
    M{k} = metricsFromLog(logs(k));
end

% name, group, field, unit, lower-is-better
defs = {
 "Firmware",                              "INFO",     "fw",          "",          NaN
 "Board / target",                        "INFO",     "board",       "",          NaN
 "IMUs logged",                           "BOARD",    "nImu",        "",          false
 "Gyro noise at rest, IMU0",              "BOARD",    "gyroNoise0",  "deg/s",     true
 "Gyro noise at rest, best IMU",          "BOARD",    "gyroNoiseBest","deg/s",    true
 "Accel noise at rest, IMU0",             "BOARD",    "accNoise0",   "m/s^2",     true
 "IMU0 vs IMU1 accel difference",         "BOARD",    "imuDiff",     "m/s^2",     true
 "Barometer noise at rest",               "BOARD",    "baroNoise",   "m",         true
 "IMU sample rate in log",                "INFO",     "imuRate",     "Hz",        NaN
 "Main loop rate",                        "BOARD",    "loopHz",      "Hz",        false
 "CPU load (95th pct)",                   "BOARD",    "cpuLoad",     "%",         true
 "Long loops",                            "BOARD",    "longLoops",   "%",         true
 "Worst loop time",                       "BOARD",    "maxT",        "us",        true
 "Free memory",                           "BOARD",    "mem",         "bytes",     false
 "5 V rail min / max",                    "BOARD",    "vccRange",    "V",         NaN
 "5 V rail ripple (std)",                 "BOARD",    "vccStd",      "V",         true
 "EKF velocity test ratio (95th pct)",    "BOARD",    "ekfV",        "",          true
 "EKF height test ratio (95th pct)",      "BOARD",    "ekfH",        "",          true
 "EKF compass test ratio (95th pct)",     "BOARD",    "ekfM",        "",          true
 "Vibration in flight, worst axis",       "AIRFRAME", "vibe",        "m/s^2",     true
 "Accel clipping events",                 "AIRFRAME", "clips",       "",          true
 "Roll tracking error (RMS)",             "AIRFRAME", "rollRms",     "deg",       true
 "Pitch tracking error (RMS)",            "AIRFRAME", "pitchRms",    "deg",       true
 "Hover throttle",                        "AIRFRAME", "hover",       "",          NaN
 "Flight time analysed",                  "INFO",     "flightS",     "s",         NaN};

T = table('Size', [size(defs,1), n+3], 'VariableTypes', ["string","string","string", repmat("string",1,n)], ...
    'VariableNames', ["Metric","Group","Unit", matlab.lang.makeValidName(labels)]);
best = strings(size(defs,1), 1);
for r = 1:size(defs,1)
    T.Metric(r) = defs{r,1}; T.Group(r) = defs{r,2}; T.Unit(r) = defs{r,4};
    vals = nan(1, n);
    for k = 1:n
        v = M{k}.(defs{r,3});
        if isstring(v) || ischar(v), T{r, 3+k} = string(v);
        elseif numel(v) == 2, T{r, 3+k} = string(sprintf('%.2f / %.2f', v));
        elseif isnan(v), T{r, 3+k} = "-";
        else, T{r, 3+k} = string(fmt(v)); vals(k) = v; end
    end
    if ~isnan(defs{r,5}) && n > 1 && sum(~isnan(vals)) > 1
        if defs{r,5}, [bv, i] = min(vals); else, [bv, i] = max(vals); end
        if sum(abs(vals - bv) <= 0.02*abs(bv) + eps) > 1, best(r) = "tie";   % within 2 %
        else, best(r) = labels(i); end
    end
end
T.Better = best;
fprintf('\n=========================== FLIGHT CONTROLLER COMPARISON ===========================\n');
disp(T);
fprintf(['BOARD rows are mostly set by the flight controller. AIRFRAME rows depend on frame,\n' ...
         'props, mounting and tune, so they only compare boards fairly on the same drone.\n']);
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
writetable(T, fullfile(opts.OutDir, "fc_comparison.csv"));

% chart: board metrics, normalised to the best board (1.0 = best)
rows = find(T.Group == "BOARD" & ~ismember(T.Metric, ["IMUs logged","Main loop rate","Free memory","5 V rail min / max"]));
V = nan(numel(rows), n);
for i = 1:numel(rows)
    for k = 1:n, V(i,k) = M{k}.(defs{rows(i),3}); end
end
ok = sum(~isnan(V), 2) == n & all(V > 0, 2);
if any(ok) && n > 1
    R = V(ok,:) ./ min(V(ok,:), [], 2);
    f = figure('Visible', vis, 'Color', 'w', 'Position', [60 60 1100 520]);
    barh(categorical(T.Metric(rows(ok)), flip(T.Metric(rows(ok)))), R);
    xline(1, 'k:'); xlabel('times the best board (1.0 = best, lower is better)');
    legend(labels, 'Location', 'southeast'); grid on
    title('Board-level comparison from flight logs');
    exportgraphics(f, fullfile(opts.OutDir, "fc_comparison.png"), 'Resolution', 110);
    if strcmp(vis, 'off'), delete(f); end
    fprintf('Saved %s\n', fullfile(opts.OutDir, "fc_comparison.png"));
end
end

%% ---------------------------------------------------------------------
function M = metricsFromLog(file)
lo = ardupilotreader(file);
names = cellstr(lo.AvailableMessages.MsgName);
want = intersect({'IMU','VIBE','BARO','PM','POWR','XKF4','ATT','CTUN','ARM','RCOU','MSG','PARM'}, names); want = want(:)';
T = readMessages(lo, MessageNames=want);
g = @(n, i) getm(T, n, i);
ts = @(x) seconds(x.Properties.RowTimes);

% firmware / board from MSG
M.fw = "unknown"; M.board = "unknown";
msg = g("MSG", 0);
if ~isempty(msg)
    s = string(msg.Message);
    f = s(startsWith(s, ["ArduCopter","ArduPlane","PX4"])); if ~isempty(f), M.fw = f(1); end
    b = s(~startsWith(s, ["ArduCopter","ChibiOS","Param","RC","Frame","GPS","u-blox","New","PreArm","Arm","EKF","GCS","IMU","Barometer","AHRS","Field","Motor","Mode","Throttle","Fence","Batt","Comp","Vibr","Crash","Land","RTL","Disarm"]) & ...
          ~contains(s, ":") & strlength(s) < 60 & contains(s, " "));
    if ~isempty(b), M.board = extractBefore(b(1) + " ", " "); end
end

% flight / armed periods from CTUN
ct = g("CTUN", 0);
fly = [];
if ~isempty(ct)
    tc = ts(ct); air = double(ct.ThO) > 0.2;
    if ismember('Alt', ct.Properties.VariableNames), air = air & double(ct.Alt) > 0.5; end
    fly = seg(tc, air);
end
armed = [];
arm = g("ARM", 0);
if ~isempty(arm)
    ta = ts(arm); st = double(arm.ArmState);
    on = ta(st == 1); offT = ta(st == 0);
    for i = 1:numel(on)
        e = offT(find(offT > on(i), 1)); if isempty(e), e = inf; end
        armed = [armed; on(i) e]; %#ok<AGROW>
    end
end
M.flightS = sum(diff(fly, 1, 2));
inFly = @(t) inSeg(t, fly);
atRest = @(t) ~inSeg(t, armed);

% IMUs
M.nImu = 0; M.gyroNoise0 = NaN; M.gyroNoiseBest = NaN; M.accNoise0 = NaN; M.imuDiff = NaN; M.imuRate = NaN;
gn = [];
for i = 0:2
    im = g("IMU", i);
    if isempty(im), continue; end
    M.nImu = M.nImu + 1;
    t = ts(im); if i == 0, M.imuRate = 1/median(diff(t)); end
    G = [double(im.GyrX) double(im.GyrY) double(im.GyrZ)]; A = [double(im.AccX) double(im.AccY) double(im.AccZ)];
    [gq, aq] = quietest(t, G, A, atRest(t));
    gn(end+1) = rad2deg(gq); %#ok<AGROW>
    if i == 0, M.gyroNoise0 = rad2deg(gq); M.accNoise0 = aq; A0 = A; t0 = t; end
    if i == 1 && exist('A0', 'var')
        A1 = interp1(t, A, t0); k = atRest(t0) & ~any(isnan(A1), 2);
        if nnz(k) > 10, M.imuDiff = norm(mean(A0(k,:)) - mean(A1(k,:))); end
    end
end
if ~isempty(gn), M.gyroNoiseBest = min(gn); end

% barometer noise: quietest 5 s while disarmed, detrended
M.baroNoise = NaN;
br = g("BARO", 0);
if ~isempty(br)
    t = ts(br); a = double(br.Alt); k = atRest(t);
    w = floor(t/5); u = unique(w(k)); s = nan(numel(u),1);
    for j = 1:numel(u)
        m = k & w == u(j);
        if nnz(m) > 10, s(j) = std(detrend(a(m))); end
    end
    M.baroNoise = min(s);
end

% CPU / scheduler
M.loopHz = NaN; M.cpuLoad = NaN; M.longLoops = NaN; M.maxT = NaN; M.mem = NaN;
pm = g("PM", 0);
if ~isempty(pm)
    v = pm.Properties.VariableNames;
    tp = ts(pm);
    if ismember('NL', v) && numel(tp) > 1, M.loopHz = median(double(pm.NL(2:end)) ./ diff(tp)); end
    if ismember('Load', v), M.cpuLoad = prctile(double(pm.Load), 95) / 10; end
    nl = intersect({'NL','NLoop'}, v);
    if ismember('NLon', v) && ~isempty(nl), M.longLoops = 100*sum(double(pm.NLon))/max(1, sum(double(pm.(nl{1})))); end
    if ismember('MaxT', v), M.maxT = prctile(double(pm.MaxT), 95); end
    if ismember('Mem', v), M.mem = median(double(pm.Mem)); end
end

% 5 V rail
M.vccRange = NaN; M.vccStd = NaN;
pw = g("POWR", 0);
if ~isempty(pw)
    vc = double(pw.Vcc); vc = vc(vc > 0.5);
    if ~isempty(vc), M.vccRange = [min(vc) max(vc)]; M.vccStd = std(vc); end
end

% EKF consistency in flight
M.ekfV = NaN; M.ekfH = NaN; M.ekfM = NaN;
k4 = g("XKF4", 0);
if ~isempty(k4)
    t = ts(k4); k = inFly(t); if ~any(k), k = true(size(t)); end
    sc = 1;
    if all(double(k4.SV) == round(double(k4.SV))) && all(double(k4.SM) == round(double(k4.SM))), sc = 0.01; end
    M.ekfV = sc*prctile(double(k4.SV(k)), 95); M.ekfH = sc*prctile(double(k4.SH(k)), 95); M.ekfM = sc*prctile(double(k4.SM(k)), 95);
end

% airframe-dependent
M.vibe = NaN; M.clips = NaN; M.rollRms = NaN; M.pitchRms = NaN; M.hover = NaN;
vb = g("VIBE", 0);
if ~isempty(vb)
    t = ts(vb); k = inFly(t);
    if any(k)
        M.vibe = prctile(max([double(vb.VibeX(k)) double(vb.VibeY(k)) double(vb.VibeZ(k))], [], 2), 95);
        M.clips = max(double(vb.Clip)) - min(double(vb.Clip));
    end
end
at = g("ATT", 0);
if ~isempty(at)
    t = ts(at); k = inFly(t);
    if any(k)
        M.rollRms = rms(double(at.DesRoll(k)) - double(at.Roll(k)));
        M.pitchRms = rms(double(at.DesPitch(k)) - double(at.Pitch(k)));
    end
end
if ~isempty(ct) && ~isempty(fly), M.hover = median(double(ct.ThO(inFly(ts(ct))))); end
fprintf('  %s | %s | %d IMU | %.0f s flying\n', M.fw, M.board, M.nImu, M.flightS);
end

function x = getm(T, n, i)
k = find(T.MsgName == n & T.InstanceID == i, 1); if isempty(k), x = []; else, x = T.MsgData{k}; end
end

function s = seg(t, mask)
d = diff([0; mask(:); 0]); a = find(d == 1); b = find(d == -1) - 1;
s = [t(a) t(b)];
end

function k = inSeg(t, s)
k = false(size(t));
for i = 1:size(s, 1), k = k | (t >= s(i,1) & t <= s(i,2)); end
end

function [gq, aq] = quietest(t, G, A, ok)
% noise = std in the quietest 1-second window (sensor floor, no handling)
w = floor(t); u = unique(w(ok)); gs = nan(numel(u),1); as = gs;
for j = 1:numel(u)
    m = ok & w == u(j);
    if nnz(m) > 5, gs(j) = max(std(G(m,:))); as(j) = max(std(A(m,:))); end
end
[gq, i] = min(gs); if isempty(i) || isnan(gq), gq = NaN; aq = NaN; else, aq = as(i); end
end

function s = fmt(v)
if abs(v) >= 1000 || v == round(v), s = sprintf('%.0f', v);
elseif abs(v) >= 10, s = sprintf('%.1f', v);
elseif abs(v) >= 0.1, s = sprintf('%.2f', v);
else, s = sprintf('%.4f', v); end
end
