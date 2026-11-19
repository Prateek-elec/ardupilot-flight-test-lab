function B = fc_benchmark2(opts)
%FC_BENCHMARK2 Your FC (all its ArduPilot logs) vs 11 commercial flight controllers
%(public PX4 Flight Review logs). Same rules for every board and every log.
%
% Sensor quality (disarmed, level and still 1-s windows, median per log):
%   gyro noise density  = std(gyro) / sqrt(ENBW)           [deg/s/sqrt(Hz)]
%   accel noise density = std(accel) / g / sqrt(ENBW)      [ug/sqrt(Hz)]
%     ENBW: ArduPilot 1.11 x INS_GYRO/ACCEL_FILTER; PX4 1/(2 x integration interval)
%   baro noise          = std of detrended altitude in 2-s windows [m]
% Flight (airborne only, crash windows removed):
%   rate tracking  = RMS(rate setpoint - rate) / RMS(rate setpoint), roll+pitch [%]
%   attitude error = RMS roll/pitch attitude error [deg]
%   EKF velocity / height / compass consistency = sqrt(test ratio), 95th pct
%   processor load in flight [%]
% Board value = median over its logs; per-board spread = 25th-75th percentile.
%   B = fc_benchmark2();

arguments
    opts.OwnDir (1,1) string = "logs"
    opts.Px4Dir (1,1) string = fullfile(fileparts(mfilename('fullpath')), "px4_logs", "ulg")
    opts.OutDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), "fleet_results")
    opts.FromSaved (1,1) logical = false
    opts.ReuseMarket (1,1) logical = false   % keep the public-log results in benchmark2.mat, recompute Your FC only
    opts.Exclude (1,:) string = "2026-11-10 17-38-46"   % flights left out of the analysis
end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
fld = ["gyroDen","accDen","baro","rateTrk","attErr","ekfV","ekfH","ekfM","cpu"];
if opts.FromSaved
    S = load(fullfile(opts.OutDir, "benchmark2.mat")); R = S.R; B = S.B;
    plotAll(R, fld, opts.OutDir, vis); return
end
names = containers.Map( ...
    {'PX4_FMU_V3','PX4_FMU_V6C','CUBEPILOT_CUBEORANGE','MICOAIR_H743','PX4_FMU_V6X','PX4_FMU_V5','CUBEPILOT_CUBEORANGEPLUS','CUAV_X7PRO','HOLYBRO_KAKUTEH7','PX4_FMU_V4','PX4_FMU_V5X'}, ...
    {'Pixhawk 2.4.8','Pixhawk 6C','Cube Orange','MicoAir H743','Pixhawk 6X','Pixhawk 4','Cube Orange+','CUAV X7 Pro','Kakute H7','Pixracer','Pixhawk 5X'});

% ---- Your FC: every log; crash/upset windows removed
own = struct('file', {"2026-11-09 16-30-17","2026-11-10 17-38-46","2026-11-10 18-09-06","2026-11-11 17-30-20", ...
    "2026-11-11 17-39-59","2026-11-12 08-56-06","2026-11-12 08-59-02","2026-11-12 17-04-34"}, ...
    'cut', {zeros(0,2), [283 inf], zeros(0,2), zeros(0,2), zeros(0,2), zeros(0,2), zeros(0,2), [930 937]});
own = own(~ismember([own.file], opts.Exclude));
R = struct('board', {}, 'label', {}, 'M', {});
fprintf('\n=== Your FC (%d ArduPilot logs)\n', numel(own));
Ms = [];
for i = 1:numel(own)
    try
        m = apMetrics(fullfile(opts.OwnDir, own(i).file + ".bin"), own(i).cut);
        m.file = own(i).file; Ms = [Ms; m]; %#ok<AGROW>
        printM(own(i).file, m);
    catch e
        fprintf('  %s skipped (%s)\n', own(i).file, e.message);
    end
end
R(1) = struct('board', "MY_FC", 'label', "Your FC", 'M', Ms);

% ---- public logs
if opts.ReuseMarket
    S0 = load(fullfile(opts.OutDir, "benchmark2.mat"), 'R'); R = [R, S0.R(2:end)];
    d = [];
else
d = dir(opts.Px4Dir); d = d([d.isdir] & ~startsWith({d.name}, '.'));
for i = 1:numel(d)
    files = dir(fullfile(d(i).folder, d(i).name, "*.ulg"));
    [~, u] = unique([files.bytes], 'stable'); files = files(u);          % drop duplicate uploads
    lab = string(d(i).name); if isKey(names, d(i).name), lab = string(names(d(i).name)); end
    fprintf('\n=== %s: %d logs\n', lab, numel(files));
    Ms = [];
    for f = 1:numel(files)
        try
            m = px4Metrics(fullfile(files(f).folder, files(f).name));
            m.file = string(files(f).name); Ms = [Ms; m]; %#ok<AGROW>
            printM(files(f).name, m);
        catch e
            fprintf('  %-42s skipped (%s)\n', files(f).name, e.message);
        end
    end
    if ~isempty(Ms), R(end+1) = struct('board', string(d(i).name), 'label', lab, 'M', Ms); end %#ok<AGROW>
end
end

% ---- table
rows = ["Gyro noise density (deg/s/rtHz)", "Accel noise density (ug/rtHz)", "Baro noise at rest (m)", ...
    "Rate tracking error (% of command)", "Attitude error RMS (deg)", ...
    "EKF velocity consistency", "EKF height consistency", "EKF compass consistency", "Processor load in flight (%)", ...
    "IMUs", "Logs: noise / flight / total"];
V = strings(numel(rows), numel(R)); N = nan(numel(fld), numel(R)); Q = nan(numel(fld), numel(R), 2);
for k = 1:numel(R)
    for j = 1:numel(fld)
        x = [R(k).M.(fld(j))]; x = x(isfinite(x));
        if isempty(x), V(j, k) = "-"; continue; end
        N(j, k) = median(x); Q(j, k, :) = prctile(x, [25 75]);
        if numel(x) > 2, V(j, k) = sprintf('%s (%s-%s)', fmt(N(j, k)), fmt(Q(j, k, 1)), fmt(Q(j, k, 2))); else, V(j, k) = fmt(N(j, k)); end
    end
    ni = [R(k).M.nImu]; V(end-1, k) = fmt(median(ni(isfinite(ni))));
    V(end, k) = sprintf('%d / %d / %d', sum(isfinite([R(k).M.gyroDen])), sum(isfinite([R(k).M.rateTrk])), numel(R(k).M));
end
B = array2table(V, 'RowNames', cellstr(rows), 'VariableNames', cellstr(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName([R.label]))));
fprintf('\n=========== BENCHMARK: Your FC vs commercial flight controllers (real flight logs) ===========\n');
disp(B);
% rank of Your FC per metric (1 = best, lower is better)
fprintf('\nYour FC rank among %d boards (1 = best):\n', numel(R));
for j = 1:numel(fld)
    if isnan(N(j, 1)), continue; end
    v = N(j, :); ok = ~isnan(v); r = sum(v(ok) < v(1)) + 1;
    fprintf('  %-36s %2d of %d\n', rows(j), r, nnz(ok));
end
writetable(B, fullfile(opts.OutDir, "benchmark2.csv"), 'WriteRowNames', true);
save(fullfile(opts.OutDir, "benchmark2.mat"), 'R', 'B', 'N', 'Q', 'rows', 'fld');
plotAll(R, fld, opts.OutDir, vis);
end

function printM(name, m)
fprintf('  %-42s gyro %7.4f accel %6.0f baro %6.3f | track %5.1f %% att %4.2f | EKF %.2f/%.2f/%.2f | cpu %4.1f | IMUs %g\n', ...
    name, m.gyroDen, m.accDen, m.baro, m.rateTrk, m.attErr, m.ekfV, m.ekfH, m.ekfM, m.cpu, m.nImu);
end

%% ===================== PX4 =====================
function M = px4Metrics(file)
u = ulogreader(file);
av = u.AvailableTopics.TopicNames;
want = intersect(["sensor_combined","vehicle_status","vehicle_air_data","estimator_status","vehicle_imu_status", ...
    "vehicle_rates_setpoint","vehicle_angular_velocity","vehicle_land_detected","vehicle_attitude","vehicle_attitude_setpoint","cpuload"], av);
m = readTopicMsgs(u, 'TopicNames', cellstr(want(:)'), 'InstanceID', num2cell(zeros(1, numel(want))));
get = @(n) m.TopicMessages{find(m.TopicNames == n, 1)};
has = @(n) any(want == n);
M = blank();
M.nImu = sum(av == "vehicle_imu_status"); if M.nImu == 0, M.nImu = NaN; end
sc = get("sensor_combined"); t = seconds(sc.timestamp);
vs = get("vehicle_status"); armed = segs(seconds(vs.timestamp), double(vs.arming_state) == 2, t(end));
rest = ~inSeg(t, armed);
dt = median(double(sc.gyro_integral_dt)) * 1e-6; if ~(dt > 0), dt = median(diff(t)); end
enbw = 1 / (2 * dt);
[gs, as, nr] = stillNoise(t, double(sc.gyro_rad), double(sc.accelerometer_m_s2), rest);
M.gyroDen = rad2deg(gs) / sqrt(enbw); M.accDen = as / 9.80665 / sqrt(enbw) * 1e6; M.nRest = nr;
if has("vehicle_air_data")
    ad = get("vehicle_air_data"); ta = seconds(ad.timestamp);
    M.baro = baroNoise(ta, double(ad.baro_alt_meter), ~inSeg(ta, armed));
end
% flying = armed and not landed
fly = armed;
if has("vehicle_land_detected")
    ld = get("vehicle_land_detected");
    fly = intersectSegs(armed, segs(seconds(ld.timestamp), ~logical(ld.landed), t(end)));
end
if has("estimator_status")
    es = get("estimator_status"); k = inSeg(seconds(es.timestamp), fly);
    if any(k)
        M.ekfV = ratio(es, "vel_test_ratio", k); M.ekfH = ratio(es, "hgt_test_ratio", k);
        M.ekfM = ratio(es, ["mag_test_ratio", "heading_test_ratio"], k);
    end
end
if has("vehicle_rates_setpoint") && has("vehicle_angular_velocity")
    sp = get("vehicle_rates_setpoint"); av2 = get("vehicle_angular_velocity");
    tw = seconds(av2.timestamp); k = inSeg(tw, fly);
    if nnz(k) > 200
        ts = seconds(sp.timestamp); [ts, iu] = unique(ts);
        des = [interp1(ts, double(sp.roll(iu)), tw(k), 'previous') interp1(ts, double(sp.pitch(iu)), tw(k), 'previous')];
        act = double(av2.xyz(k, 1:2));
        ok = all(isfinite(des), 2);
        M.rateTrk = 100 * rms(des(ok, :) - act(ok, :), 'all') / rms(des(ok, :), 'all');
    end
end
if has("vehicle_attitude") && has("vehicle_attitude_setpoint")
    va = get("vehicle_attitude"); vsp = get("vehicle_attitude_setpoint");
    tq = seconds(va.timestamp); k = inSeg(tq, fly);
    if nnz(k) > 100
        [r, p] = q2rp(double(va.q(k, :)));
        tsp = seconds(vsp.timestamp); [tsp, iu] = unique(tsp);
        [rd, pd] = q2rp(double(vsp.q_d(iu, :)));
        rd = interp1(tsp, rd, tq(k), 'previous'); pd = interp1(tsp, pd, tq(k), 'previous');
        e = rad2deg([wrapToPi(rd - r) wrapToPi(pd - p)]); e = e(all(isfinite(e), 2), :);
        M.attErr = rms(e, 'all');
    end
end
if has("cpuload")
    cl = get("cpuload"); k = inSeg(seconds(cl.timestamp), fly);
    if any(k), M.cpu = 100 * mean(double(cl.load(k))); end
end
end

function [r, p] = q2rp(q)
r = atan2(2 * (q(:,1).*q(:,2) + q(:,3).*q(:,4)), 1 - 2 * (q(:,2).^2 + q(:,3).^2));
p = asin(max(-1, min(1, 2 * (q(:,1).*q(:,3) - q(:,4).*q(:,2)))));
end

function r = ratio(es, names, k)
r = NaN;
for n = names
    if ismember(n, es.Properties.VariableNames)
        x = sqrt(max(double(es.(n)(k)), 0)); x = x(isfinite(x));
        if ~isempty(x), r = prctile(x, 95); end
        return
    end
end
end

%% ===================== ArduPilot =====================
function M = apMetrics(file, cut)
lo = ardupilotreader(file);
av = string(lo.AvailableMessages.MsgName);
want = intersect(["IMU","BARO","ARM","XKF4","PARM","EV","RATE","ATT","PM"], av);
T = readMessages(lo, MessageNames=cellstr(want(:)'));
g = @(n) T.MsgData{find(T.MsgName == n & T.InstanceID == 0, 1)};
hasm = @(n) any(T.MsgName == n);
P = g("PARM"); pn = string(P.Name); pv = double(P.Value);
par = @(n, d) pickParam(pn, pv, n, d);
M = blank();
tEnd = seconds(lo.EndTime);
armed = zeros(0, 2);
if hasm("ARM"), a = g("ARM"); armed = segs(seconds(a.Properties.RowTimes), double(a.ArmState) == 1, tEnd); end
fly = zeros(0, 2);
if hasm("EV")
    ev = g("EV"); te = seconds(ev.Properties.RowTimes); id = double(ev.Id);
    to = te(id == 28); ld = te(id == 18);
    for k = 1:numel(to)
        e = ld(find(ld > to(k), 1)); if isempty(e), e = tEnd; end
        s = [to(k) + 3, e];
        for c = 1:size(cut, 1), if s(1) < cut(c, 1) && s(2) > cut(c, 1), s(2) = cut(c, 1); end, end
        if diff(s) > 5, fly(end+1, :) = s; end %#ok<AGROW>
    end
end
im = g("IMU"); t = seconds(im.Properties.RowTimes);
rest = ~inSeg(t, armed);
[gs, as, nr] = stillNoise(t, [double(im.GyrX) double(im.GyrY) double(im.GyrZ)], [double(im.AccX) double(im.AccY) double(im.AccZ)], rest);
M.gyroDen = rad2deg(gs) / sqrt(1.11 * par("INS_GYRO_FILTER", 20));
M.accDen = as / 9.80665 / sqrt(1.11 * par("INS_ACCEL_FILTER", 20)) * 1e6;
M.nRest = nr;
M.nImu = numel(unique(T.InstanceID(T.MsgName == "IMU")));
br = g("BARO"); tb = seconds(br.Properties.RowTimes);
M.baro = baroNoise(tb, double(br.Alt), ~inSeg(tb, armed));
if isempty(fly), return; end
k4 = g("XKF4"); k = inSeg(seconds(k4.Properties.RowTimes), fly);
sc = 1; if all(double(k4.SV) == round(double(k4.SV))), sc = 0.01; end
M.ekfV = sc * prctile(double(k4.SV(k)), 95); M.ekfH = sc * prctile(double(k4.SH(k)), 95); M.ekfM = sc * prctile(double(k4.SM(k)), 95);
if hasm("RATE")
    r = g("RATE"); k = inSeg(seconds(r.Properties.RowTimes), fly);
    if 1 / median(diff(seconds(r.Properties.RowTimes))) > 100        % only 400 Hz logs (10 Hz is too coarse)
        des = [double(r.RDes(k)) double(r.PDes(k))]; act = [double(r.R(k)) double(r.P(k))];
        M.rateTrk = 100 * rms(des - act, 'all') / rms(des, 'all');
    end
end
if hasm("ATT")
    at = g("ATT"); k = inSeg(seconds(at.Properties.RowTimes), fly);
    M.attErr = rms([double(at.DesRoll(k) - at.Roll(k)) double(at.DesPitch(k) - at.Pitch(k))], 'all');
end
if hasm("PM")
    pm = g("PM"); k = inSeg(seconds(pm.Properties.RowTimes), fly);
    if any(k), M.cpu = mean(double(pm.Load(k))) / 10; end
end
end

%% ===================== shared =====================
function M = blank()
M = struct('gyroDen', NaN, 'accDen', NaN, 'baro', NaN, 'rateTrk', NaN, 'attErr', NaN, 'ekfV', NaN, 'ekfH', NaN, 'ekfM', NaN, ...
    'cpu', NaN, 'nImu', NaN, 'nRest', 0, 'file', "");
end

function v = pickParam(pn, pv, n, d)
i = find(pn == n, 1, 'last'); if isempty(i), v = d; else, v = pv(i); end
end

function s = segs(t, on, tEnd)
s = zeros(0, 2); st = false;
for i = 1:numel(t)
    if on(i) && ~st, s(end+1, 1) = t(i); st = true; %#ok<AGROW>
    elseif ~on(i) && st, s(end, 2) = t(i); st = false; end
end
if st, s(end, 2) = tEnd; end
end

function c = intersectSegs(a, b)
c = zeros(0, 2);
for i = 1:size(a, 1)
    for j = 1:size(b, 1)
        lo = max(a(i, 1), b(j, 1)); hi = min(a(i, 2), b(j, 2));
        if hi > lo, c(end+1, :) = [lo hi]; end %#ok<AGROW>
    end
end
end

function k = inSeg(t, s)
k = false(size(t));
for i = 1:size(s, 1), k = k | (t >= s(i, 1) & t <= s(i, 2)); end
end

function [gs, as, nr] = stillNoise(t, G, A, rest)
w = floor(t); u = unique(w(rest)); gv = nan(numel(u), 1); av = gv;
for j = 1:numel(u)
    m = rest & w == u(j);
    if nnz(m) < 10, continue; end
    a = mean(A(m, :));
    if abs(norm(a) - 9.81) > 0.6 || max(std(G(m, :))) > deg2rad(1.5), continue; end
    gv(j) = max(std(G(m, :))); av(j) = max(std(A(m, :)));
end
nr = sum(~isnan(gv));
if nr == 0, gs = NaN; as = NaN; return; end
gs = median(gv, 'omitnan'); as = median(av, 'omitnan');
end

function b = baroNoise(t, alt, rest)
w = floor(t / 2); u = unique(w(rest)); s = nan(numel(u), 1);
for j = 1:numel(u)
    m = rest & w == u(j);
    if nnz(m) >= 8, s(j) = std(detrend(alt(m))); end
end
b = median(s, 'omitnan'); if isempty(b), b = NaN; end
end

function s = fmt(v)
if isnan(v), s = "-"; elseif v >= 100, s = sprintf('%.0f', v); elseif v >= 10, s = sprintf('%.1f', v); elseif v >= 1, s = sprintf('%.2f', v); else, s = sprintf('%.3f', v); end
end

function plotAll(R, ~, outDir, vis)
% rate tracking and processor load are not plotted: they depend on flying style and on how
% each firmware defines load, so they do not compare boards fairly
fld = ["gyroDen","accDen","baro","attErr","ekfV","ekfH","ekfM"];
ttl = ["Gyro noise density (deg/s/\surdHz)", "Accel noise density (\mug/\surdHz)", "Baro noise at rest (m)", ...
    "Attitude error RMS in flight (deg)", "EKF velocity consistency", "EKF height consistency", "EKF compass consistency"];
labels = [R.label];
f = figure('Visible', vis, 'Color', 'w', 'Position', [30 30 1500 1050]); try theme(f, 'light'); catch, end
tl = tiledlayout(f, 3, 3, 'TileSpacing', 'compact');

title(tl, "Your FC vs 11 commercial flight controllers: real flight logs (lower is better)", 'FontWeight', 'bold');
subtitle(tl, "Bars: median per board. Dots: individual logs. Your FC: own ArduPilot logs; others: public PX4 Flight Review logs.");
rng(1);
for j = 1:numel(fld)
    ax = nexttile(tl); hold(ax, 'on');
    med = nan(1, numel(R));
    for k = 1:numel(R), x = [R(k).M.(fld(j))]; x = x(isfinite(x)); if ~isempty(x), med(k) = median(x); end, end
    [~, ord] = sort(med, 'ascend', 'MissingPlacement', 'last');
    b = bar(ax, 1:numel(R), med(ord), 0.6, 'EdgeColor', 'none');
    b.FaceColor = 'flat'; b.CData = repmat([0.68 0.71 0.76], numel(R), 1);
    b.CData(ord == 1, :) = [0.80 0.45 0.05];
    for i = 1:numel(ord)
        x = [R(ord(i)).M.(fld(j))]; x = x(isfinite(x));
        if ~isempty(x), scatter(ax, i + 0.1 * randn(size(x)), x, 12, [0.2 0.25 0.33], 'filled', 'MarkerFaceAlpha', 0.55); end
    end
    set(ax, 'XTick', 1:numel(R), 'XTickLabel', labels(ord), 'FontSize', 8.5, 'Box', 'off'); xtickangle(ax, 40);
    grid(ax, 'on'); ax.XGrid = 'off'; title(ax, ttl(j));
    all = arrayfun(@(r) [r.M.(fld(j))], R, 'UniformOutput', false); all = [all{:}]; all = all(isfinite(all));
    if ~isempty(all), ylim(ax, [0 prctile(all, 95) * 1.25 + eps]); end
end
exportgraphics(f, fullfile(outDir, "benchmark2.png"), 'Resolution', 130, 'BackgroundColor', 'white');
if strcmp(vis, 'off'), delete(f); end
fprintf('Saved %s\n', fullfile(outDir, "benchmark2.png"));
end
