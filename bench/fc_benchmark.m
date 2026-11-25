function B = fc_benchmark(opts)
%FC_BENCHMARK Benchmark Your FC against market flight controllers using real
%public flight logs (PX4 Flight Review) and your ArduPilot log.
%
%   Board-level metrics are normalised so PX4 and ArduPilot logs compare:
%     gyro noise density  = std(gyro at rest) / sqrt(ENBW)   [deg/s/sqrt(Hz)]
%     accel noise density = std(accel at rest)/ g / sqrt(ENBW) [ug/sqrt(Hz)]
%       ArduPilot: ENBW = 1.11*INS_GYRO_FILTER (2nd-order LPF), accel 1.11*INS_ACCEL_FILTER
%       PX4:       ENBW = 1/(2*integration dt) of sensor_combined
%     baro noise          = std of detrended altitude in 2 s windows [m]
%   Noise uses the same rule for every board: all disarmed 1-s windows where the
%   drone sits level and still (before arming or after landing), and the MEDIAN
%   of those windows, so a log with many still seconds gets no advantage.
%     EKF velocity / height / compass consistency: sqrt(test ratio), 95th pct in flight
%   Values are medians over all usable logs per board; logs without a
%   still, disarmed period are skipped for the noise metrics.
%
%   B = fc_benchmark();

arguments
    opts.OwnLog (1,1) string = lab_default_log()
    opts.Px4Dir (1,1) string = fullfile(fileparts(mfilename('fullpath')), "px4_logs", "ulg")
    opts.OutDir (1,1) string = fullfile(fileparts(mfilename('fullpath')), "results")
    opts.FromSaved (1,1) logical = false   % redraw from results\fc_benchmark.mat without re-reading logs
end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
if opts.FromSaved
    S = load(fullfile(opts.OutDir, "fc_benchmark.mat"), 'A', 'R', 'B');
    A = S.A; R = S.R; B = S.B;
    labels = ["Your FC", string({R.label})];
    fld = ["gyroDen","accDen","baro"];
    N = nan(3, numel(labels));
    for j = 1:3
        N(j,1) = A.(fld(j));
        for k = 1:numel(R), x = [R(k).M.(fld(j))]; x = x(~isnan(x)); if ~isempty(x), N(j,k+1) = median(x); end, end
    end
    plotBenchmark(A, R, N, labels, fld, opts.OutDir, vis);
    return
end
names = containers.Map( ...
    {'PX4_FMU_V3','PX4_FMU_V6C','CUBEPILOT_CUBEORANGE','MICOAIR_H743'}, ...
    {'Pixhawk 2.4.8 (fmu-v3)','Pixhawk 6C','Cube Orange','MicoAir H743 (PX4)'});

fprintf('\n=== Your FC (ArduPilot log)\n');
A = apMetrics(opts.OwnLog);
fprintf('  gyro %.4f deg/s/rtHz (IMU0), accel %.0f ug/rtHz, baro %.3f m, rest windows %d\n', A.gyroDen, A.accDen, A.baro, A.nRest);

d = dir(opts.Px4Dir); d = d([d.isdir] & ~startsWith({d.name}, '.'));
R = struct('board', {}, 'label', {}, 'M', {});
for i = 1:numel(d)
    files = dir(fullfile(d(i).folder, d(i).name, "*.ulg"));
    lab = d(i).name; if isKey(names, lab), lab = names(lab); end
    fprintf('\n=== %s: %d logs\n', lab, numel(files));
    Ms = [];
    for f = 1:numel(files)
        try
            m = px4Metrics(fullfile(files(f).folder, files(f).name));
            fprintf('  %-40s gyro %7.4f  accel %6.0f  baro %6.3f  rest %3d s  EKF v/h/m %.2f/%.2f/%.2f  IMUs %d\n', ...
                files(f).name, m.gyroDen, m.accDen, m.baro, m.nRest, m.ekfV, m.ekfH, m.ekfM, m.nImu);
            Ms = [Ms; m]; %#ok<AGROW>
        catch e
            fprintf('  %-40s skipped (%s)\n', files(f).name, e.message);
        end
    end
    if ~isempty(Ms), R(end+1) = struct('board', d(i).name, 'label', lab, 'M', Ms); end %#ok<AGROW>
end

% ---- summary table
rows = ["Gyro noise density (deg/s/rtHz)", "Accel noise density (ug/rtHz)", "Baro noise at rest (m)", ...
        "EKF velocity consistency (95th pct)", "EKF height consistency (95th pct)", "EKF compass consistency (95th pct)", ...
        "IMUs", "Logs used for noise / total"];
fld = ["gyroDen","accDen","baro","ekfV","ekfH","ekfM","nImu"];
labels = ["Your FC (your log)", string({R.label})];
V = strings(numel(rows), numel(labels)); N = nan(numel(fld), numel(labels));
for j = 1:numel(fld)
    N(j,1) = A.(fld(j)); V(j,1) = fmt(A.(fld(j)));
    for k = 1:numel(R)
        x = [R(k).M.(fld(j))]; x = x(~isnan(x));
        if isempty(x), V(j,k+1) = "-"; continue; end
        N(j,k+1) = median(x);
        if numel(x) > 2, V(j,k+1) = sprintf('%s (%s-%s)', fmt(median(x)), fmt(prctile(x,25)), fmt(prctile(x,75)));
        else, V(j,k+1) = fmt(median(x)); end
    end
end
V(end,1) = sprintf('%d / 1', A.nRest > 0);
for k = 1:numel(R), V(end,k+1) = sprintf('%d / %d', sum(~isnan([R(k).M.gyroDen])), numel(R(k).M)); end
B = array2table(V, 'RowNames', cellstr(rows), 'VariableNames', cellstr(matlab.lang.makeUniqueStrings(matlab.lang.makeValidName(labels))));
fprintf('\n=================== FLIGHT CONTROLLER BENCHMARK (real flight logs) ===================\n');
fprintf('Values: median (25th-75th percentile) across public logs. Lower is better except IMUs.\n');
disp(B);
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
writetable(B, fullfile(opts.OutDir, "fc_benchmark.csv"), 'WriteRowNames', true);
save(fullfile(opts.OutDir, "fc_benchmark.mat"), 'A', 'R', 'B');

plotBenchmark(A, R, N, labels, fld, opts.OutDir, vis);
end

function plotBenchmark(A, R, N, labels, fld, outDir, vis) %#ok<INUSD>
% medians as bars (your FC highlighted), every public log as a dot
f = figure('Visible', vis, 'Color', 'w', 'Position', [60 60 1250 500]);
try, theme(f, 'light'); catch, end
tl = tiledlayout(1, 3, 'TileSpacing', 'compact');
title(tl, 'Sensor noise measured from real flight logs (lower is better)', 'FontWeight', 'bold');
subtitle(tl, 'Bars: median per board. Dots: individual public logs (PX4 Flight Review). Your FC: own ArduPilot log.');
ttl = ["Gyro noise density (deg/s/\surdHz)", "Accel noise density (\mug/\surdHz)", "Baro noise at rest (m)"];
rng(1);
for j = 1:3
    ax = nexttile; hold(ax, 'on');
    b = bar(ax, 1:numel(labels), N(j,:), 0.55, 'EdgeColor', 'none');
    b.FaceColor = 'flat'; b.CData = repmat([0.62 0.66 0.72], numel(labels), 1); b.CData(1,:) = [0.80 0.45 0.05];
    for k = 1:numel(R)
        x = [R(k).M.(fld(j))]; x = x(~isnan(x));
        if ~isempty(x), scatter(ax, k+1 + 0.08*randn(size(x)), x, 16, [0.25 0.3 0.38], 'filled', 'MarkerFaceAlpha', 0.6); end
    end
    for k = 1:numel(labels)
        if ~isnan(N(j,k))
            if N(j,k) >= 100, s = sprintf('  %.0f', N(j,k)); else, s = sprintf('  %.3g', N(j,k)); end
            text(ax, k, N(j,k), s, 'Rotation', 90, 'FontSize', 9, 'Color', [0.15 0.15 0.15]);
        end
    end
    set(ax, 'XTick', 1:numel(labels), 'XTickLabel', labels, 'FontSize', 10, 'Box', 'off');
    xtickangle(ax, 30); grid(ax, 'on'); ax.YGrid = 'on'; ax.XGrid = 'off'; title(ax, ttl(j));
    ylim(ax, [0, max([N(j,:), 0]) * 1.6 + eps]);
end
exportgraphics(f, fullfile(outDir, "fc_benchmark.png"), 'Resolution', 150, 'BackgroundColor', 'white');
if strcmp(vis, 'off'), delete(f); end
fprintf('Saved %s\n', fullfile(outDir, "fc_benchmark.png"));
end

%% ---------------------------------------------------------------------
function M = px4Metrics(file)
u = ulogreader(file);
av = u.AvailableTopics.TopicNames;
want = intersect(["sensor_combined","vehicle_status","vehicle_air_data","estimator_status","vehicle_imu_status"], av);
m = readTopicMsgs(u, 'TopicNames', cellstr(want), 'InstanceID', num2cell(zeros(1, numel(want))));
get = @(n) m.TopicMessages{find(m.TopicNames == n, 1)};
M = blank();
M.nImu = sum(av == "vehicle_imu_status"); if M.nImu == 0, M.nImu = NaN; end
sc = get("sensor_combined"); t = seconds(sc.timestamp);
vs = get("vehicle_status"); armed = armedSegments(seconds(vs.timestamp), double(vs.arming_state) == 2, t(end));
rest = preArm(t, armed);
dt = median(double(sc.gyro_integral_dt)) * 1e-6; if ~(dt > 0), dt = median(diff(t)); end
enbw = 1 / (2*dt);
G = double(sc.gyro_rad); A = double(sc.accelerometer_m_s2);
[gs, as, nr] = quietest(t, G, A, rest);
M.gyroDen = rad2deg(gs) / sqrt(enbw);
M.accDen = as / 9.80665 / sqrt(enbw) * 1e6;
M.nRest = nr;
if any(want == "vehicle_air_data")
    ad = get("vehicle_air_data"); ta = seconds(ad.timestamp);
    M.baro = baroNoise(ta, double(ad.baro_alt_meter), preArm(ta, armed));
end
if any(want == "estimator_status")
    es = get("estimator_status"); te = seconds(es.timestamp); k = inSeg(te, armed);
    if any(k)
        M.ekfV = ratio(es, "vel_test_ratio", k);
        M.ekfH = ratio(es, "hgt_test_ratio", k);
        M.ekfM = ratio(es, ["mag_test_ratio", "heading_test_ratio"], k);
    end
end
end

function r = ratio(es, names, k)
% sqrt(test ratio), 95th pct in flight; field names differ between PX4 versions
r = NaN;
for n = names
    if ismember(n, es.Properties.VariableNames)
        x = sqrt(max(double(es.(n)(k)), 0)); x = x(isfinite(x));
        if ~isempty(x), r = prctile(x, 95); end
        return
    end
end
end

function k = preArm(t, armed)
% every disarmed moment (before arming or after landing); the still check in
% quietest() then keeps only 1-s windows where the drone sits level and still
k = ~inSeg(t, armed);
end

function M = apMetrics(file)
lo = ardupilotreader(file);
T = readMessages(lo, MessageNames={'IMU','BARO','ARM','XKF4','PARM','CTUN'});
g = @(n) T.MsgData{find(T.MsgName == n & T.InstanceID == 0, 1)};
P = g("PARM"); pn = string(P.Name); pv = double(P.Value);
par = @(n, d) pickParam(pn, pv, n, d);
M = blank();
arm = g("ARM"); ta = seconds(arm.Properties.RowTimes); st = double(arm.ArmState);
armed = armedSegments(ta, st == 1, inf);
im = g("IMU"); t = seconds(im.Properties.RowTimes);
G = [double(im.GyrX) double(im.GyrY) double(im.GyrZ)]; A = [double(im.AccX) double(im.AccY) double(im.AccZ)];
[gs, as, nr] = quietest(t, G, A, preArm(t, armed));
M.gyroDen = rad2deg(gs) / sqrt(1.11 * par("INS_GYRO_FILTER", 20));
M.accDen = as / 9.80665 / sqrt(1.11 * par("INS_ACCEL_FILTER", 20)) * 1e6;
M.nRest = nr;
M.nImu = sum(T.MsgName == "IMU");
br = g("BARO"); tb = seconds(br.Properties.RowTimes);
M.baro = baroNoise(tb, double(br.Alt), preArm(tb, armed));
k4 = g("XKF4"); tk = seconds(k4.Properties.RowTimes); k = inSeg(tk, armed);
sc = 1; if all(double(k4.SV) == round(double(k4.SV))), sc = 0.01; end
M.ekfV = sc*prctile(double(k4.SV(k)), 95); M.ekfH = sc*prctile(double(k4.SH(k)), 95); M.ekfM = sc*prctile(double(k4.SM(k)), 95);
end

%% ---------------------------------------------------------------------
function M = blank()
M = struct('gyroDen', NaN, 'accDen', NaN, 'baro', NaN, 'ekfV', NaN, 'ekfH', NaN, 'ekfM', NaN, 'nImu', NaN, 'nRest', 0);
end

function v = pickParam(pn, pv, n, d)
i = find(pn == n, 1, 'last'); if isempty(i), v = d; else, v = pv(i); end
end

function s = armedSegments(t, isArmed, tEnd)
s = zeros(0, 2); on = false;
for i = 1:numel(t)
    if isArmed(i) && ~on, s(end+1, 1) = t(i); on = true; %#ok<AGROW>
    elseif ~isArmed(i) && on, s(end, 2) = t(i); on = false; end
end
if on, s(end, 2) = tEnd; end
end

function k = inSeg(t, s)
k = false(size(t));
for i = 1:size(s, 1), k = k | (t >= s(i,1) & t <= s(i,2)); end
end

function [gs, as, nr] = quietest(t, G, A, rest)
% sensor floor: quietest 1-s window while disarmed and sitting level and still
w = floor(t); u = unique(w(rest)); gv = nan(numel(u),1); av = gv;
for j = 1:numel(u)
    m = rest & w == u(j);
    if nnz(m) < 10, continue; end
    a = mean(A(m,:));
    if abs(norm(a) - 9.81) > 0.6 || max(std(G(m,:))) > deg2rad(1.5), continue; end   % handled / moving
    gv(j) = max(std(G(m,:))); av(j) = max(std(A(m,:)));
end
nr = sum(~isnan(gv));
if nr == 0, gs = NaN; as = NaN; return; end
% median, not minimum: a log with many still seconds must not get a
% best-of-many advantage over a log with only one or two
gs = median(gv, 'omitnan'); as = median(av, 'omitnan');
end

function b = baroNoise(t, alt, rest)
w = floor(t/2); u = unique(w(rest)); s = nan(numel(u),1);
for j = 1:numel(u)
    m = rest & w == u(j);
    if nnz(m) >= 8, s(j) = std(detrend(alt(m))); end
end
b = median(s, 'omitnan');
if isempty(b), b = NaN; end
end

function s = fmt(v)
if isnan(v), s = "-"; elseif v >= 100, s = sprintf('%.0f', v); elseif v >= 1, s = sprintf('%.2f', v); else, s = sprintf('%.4f', v); end
end
