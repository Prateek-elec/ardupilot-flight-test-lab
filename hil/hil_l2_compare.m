function T = hil_l2_compare(hilLog, refLog, opts)
%HIL_L2_COMPARE Your FC Simulation-on-Hardware flight vs PC SITL reference.
%   Same firmware tune, same built-in physics, same scripted mission: any difference
%   comes from running the flight code on the flight controller's processor in real time.
%
%   T = hil_l2_compare("...\00000003.BIN", ref.log)
%   Writes l2_comparison.csv and l2_comparison.png next to the HIL log.

arguments
    hilLog (1,1) string
    refLog (1,1) string
    opts.OutDir (1,1) string = ""
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(fileparts(here), "bench"));
if opts.OutDir == "", opts.OutDir = fileparts(hilLog); end
H = sitl_analysis(hilLog, [], OutDir=fullfile(opts.OutDir, "analysis_hil"));
R = sitl_analysis(refLog, [], OutDir=fullfile(opts.OutDir, "analysis_ref"));
pH = perf(hilLog); pR = perf(refLog);

% metric | PC SITL | Your FC HIL | tolerance on |HIL - SITL| (NaN = limit on HIL value)
M = {
 "Controller replica error, roll (% FS)",  100*R.ctrl(1).rmse, 100*H.ctrl(1).rmse, NaN, -1, "HIL < 1 % of full scale"
 "Controller replica error, pitch (% FS)", 100*R.ctrl(2).rmse, 100*H.ctrl(2).rmse, NaN, -1, "HIL < 1 % of full scale"
 "Controller replica error, yaw (% FS)",   100*R.ctrl(3).rmse, 100*H.ctrl(3).rmse, NaN, -1, "HIL < 1 % of full scale"
 "Roll/pitch step settling (ms)",         R.stepSettleMs,    H.stepSettleMs,    150, NaN,  "|diff| <= 150 ms"
 "Roll/pitch step overshoot (%)",         R.stepOvershoot,   H.stepOvershoot,   5,   NaN,  "|diff| <= 5 pts"
 "Roll/pitch tracking RMS (deg)",         R.stepTrackRms,    H.stepTrackRms,    0.5, NaN,  "|diff| <= 0.5 deg"
 "Yaw rate achieved / commanded (%)",     100*R.yaw.rateAch/R.yaw.rateCmd, 100*H.yaw.rateAch/H.yaw.rateCmd, 10, NaN, "|diff| <= 10 pts"
 "Altitude hold RMS (m)",                 R.altHoldRms,      H.altHoldRms,      0.1, NaN,  "|diff| <= 0.1 m"
 "Loiter position RMS (m)",               R.loiter.rms,      H.loiter.rms,      0.3, NaN,  "|diff| <= 0.3 m"
 "CPU load, mean (%)",                    pR.load,           pH.load,           NaN, -80,  "HIL < 80 %"
 "Long loops (% of loops)",               pR.longPct,        pH.longPct,        NaN, -5,   "HIL < 5 %"
 "Max loop time (ms)",                    pR.maxT,           pH.maxT,           NaN, -10,  "HIL < 10 ms"
};
n = size(M, 1); res = strings(n, 1);
for i = 1:n
    r = M{i,2}; h = M{i,3}; tol = M{i,4}; lim = M{i,5};
    if isnan(h), res(i) = "NO DATA";
    elseif ~isnan(tol), res(i) = passStr(abs(h - r) <= tol);
    elseif lim >= 0, res(i) = passStr(h >= lim);
    else, res(i) = passStr(h < -lim);
    end
end
T = table(string(M(:,1)), cell2mat(M(:,2)), cell2mat(M(:,3)), string(M(:,6)), res, ...
    'VariableNames', ["Metric", "PC_SITL", "FC_HIL", "Criterion", "Result"]);
fprintf('\n=========== HIL Level 2: Your FC Simulation on Hardware vs PC SITL ===========\n');
disp(T);
fprintf('%d of %d checks passed.\n', sum(res == "PASS"), n);
writetable(T, fullfile(opts.OutDir, "l2_comparison.csv"));

% figure: step responses and loop timing side by side
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1300 760]);
try theme(f, 'light'); catch, end
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
title(tl, "HIL Level 2: same tune and physics, flight code on Your FC vs on PC");
names = ["ROLL right", "PITCH forward"];
for k = 1:2
    ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
    [tr, yr, dr] = stepTrace(R, names(k)); [th, yh] = stepTrace(H, names(k));
    if ~isempty(tr), plot(ax, tr, dr, 'k--', 'DisplayName', 'target'); plot(ax, tr, yr, 'LineWidth', 1.5, 'DisplayName', 'PC SITL'); end
    if ~isempty(th), plot(ax, th, yh, 'LineWidth', 1.5, 'DisplayName', 'Your FC HIL'); end
    title(ax, names(k) + " step"); xlabel(ax, 's from stick step'); ylabel(ax, 'deg'); legend(ax, 'Location', 'best');
end
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
if ~isempty(pR.t), plot(ax, pR.t - pR.t(1), pR.loadSeries, 'DisplayName', 'PC SITL'); end
if ~isempty(pH.t), plot(ax, pH.t - pH.t(1), pH.loadSeries, 'DisplayName', 'Your FC HIL'); end
title(ax, 'Scheduler load (PM.Load, %)'); xlabel(ax, 's'); legend(ax, 'Location', 'best');
ax = nexttile(tl);
b = [cell2mat(M(4:9, 2)) cell2mat(M(4:9, 3))];
bn = b ./ max(abs(b), [], 2); bn(isnan(bn)) = 0;
barh(ax, categorical(string(M(4:9, 1)), string(M(4:9, 1))), bn); grid(ax, 'on');
legend(ax, ["PC SITL", "Your FC HIL"], 'Location', 'southeast'); title(ax, 'Flight metrics (normalised to the larger value)');
exportgraphics(f, fullfile(opts.OutDir, "l2_comparison.png"), 'Resolution', 120);
fprintf('Saved %s\n', fullfile(opts.OutDir, "l2_comparison.png"));
if strcmp(vis, 'off'), delete(f); end
end

function p = perf(binFile)
p = struct('load', NaN, 'longPct', NaN, 'maxT', NaN, 't', [], 'loadSeries', []);
lo = ardupilotreader(binFile);
if ~any(lo.AvailableMessages.MsgName == "PM"), return; end
Tm = readMessages(lo, MessageNames={'PM'}); P = Tm.MsgData{1};
p.t = seconds(P.Properties.RowTimes);
v = P.Properties.VariableNames;
if ismember('Load', v), p.loadSeries = double(P.Load) / 10; p.load = mean(p.loadSeries); end
if all(ismember({'NLon', 'NL'}, v)), p.longPct = 100 * sum(double(P.NLon)) / max(1, sum(double(P.NL))); end
if ismember('MaxT', v), p.maxT = max(double(P.MaxT)) / 1000; end
end

function [t, y, d] = stepTrace(A, name)
t = []; y = []; d = [];
if ~isfield(A, 'S') || isempty(A.S), return; end
for i = 1:numel(A.S)
    if ~isempty(A.S(i).name) && contains(string(A.S(i).name), name)
        t = A.S(i).t; y = A.S(i).act; d = A.S(i).des; return;
    end
end
end

function s = passStr(ok)
if ok, s = "PASS"; else, s = "FAIL"; end
end
