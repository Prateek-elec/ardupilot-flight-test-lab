function R = hil_l1_analysis(resultDir)
%HIL_L1_ANALYSIS Pass/fail report for a Level 1 real-board I/O HIL run.
%   R = hil_l1_analysis("...\hil\results\L1_20261110_141627")
%   With no argument, the newest results\L1_* folder is used.
%   Writes l1_report.png and l1_results.csv into the same folder.

if nargin < 1 || resultDir == ""
    d = dir(fullfile(fileparts(mfilename('fullpath')), "results", "L1_*"));
    if isempty(d), error("No results\\L1_* folder found - run hil_l1_io_test.py first."); end
    [~, k] = max([d.datenum]); resultDir = fullfile(d(k).folder, d(k).name);
end
resultDir = string(resultDir);
S = readtable(fullfile(resultDir, "samples.csv"), 'TextType', 'string');
E = readtable(fullfile(resultDir, "events.csv"), 'TextType', 'string', 'Delimiter', ',');
J = jsondecode(fileread(fullfile(resultDir, "summary.json")));
fprintf('\n=== HIL Level 1 (real-board I/O) - %s\n', resultDir);
if isfield(J, 'firmware'), fprintf('Firmware ArduCopter %s, %d samples\n', J.firmware, J.samples); end

rows = cell(0, 4);   % Check | Measured | Limit | Result

%% 1. Rest (disarmed, board still)
rest = S(S.phase == "rest", :);
if height(rest) > 20
    dt = (rest.t_host(end) - rest.t_host(1)) / (height(rest) - 1);
    attStd = rad2deg(std([rest.roll rest.pitch]));
    rateStd = rad2deg(std([rest.p rest.q rest.r]));
    rows = add(rows, "Sample rate at rest", sprintf('%.0f Hz', 1/dt), "> 40 Hz", 1/dt > 40);
    rows = add(rows, "Attitude noise roll / pitch (std)", sprintf('%.3f / %.3f deg', attStd), "< 0.1 deg", all(attStd < 0.1));
    rows = add(rows, "Gyro rate noise p / q / r (std)", sprintf('%.2f / %.2f / %.2f deg/s', rateStd), "< 0.5 deg/s", all(rateStd < 0.5));
    rows = add(rows, "CPU load at rest", sprintf('%.0f %%', mean(rest.load_pct)), "< 70 %", mean(rest.load_pct) < 70);
    rows = add(rows, "Vibration at rest (x/y/z)", sprintf('%.2f / %.2f / %.2f m/s^2', mean(rest.vibx), mean(rest.viby), mean(rest.vibz)), "< 3 m/s^2", all([mean(rest.vibx) mean(rest.viby) mean(rest.vibz)] < 3));
end

%% 2. Parameters that must be right before flight
if isfield(J, 'params')
    P = J.params;
    rows = add(rows, "Pitch channel reversed (RC2_REVERSED)", num2str(P.RC2_REVERSED), "1 on this drone", isequal(P.RC2_REVERSED, 1));
    if ~isempty(P.ARMING_CHECK), rows = add(rows, "Pre-arm checks (ARMING_CHECK)", num2str(P.ARMING_CHECK), "1 (all)", isequal(P.ARMING_CHECK, 1)); end
    fn = [P.SERVO1_FUNCTION P.SERVO2_FUNCTION P.SERVO3_FUNCTION P.SERVO4_FUNCTION];
    rows = add(rows, "Motor outputs 1-4 (SERVOx_FUNCTION)", mat2str(fn), "[33 34 35 36]", isequal(fn, [33 34 35 36]));
end

%% 3. Stick steps: motor direction and latency
lat = [];
if isfield(J, 'armed_ok') && J.armed_ok && isfield(J, 'steps')
    f = fieldnames(J.steps);
    for i = 1:numel(f)
        s = J.steps.(f{i});
        name = strrep(f{i}, '_', ' '); if isfield(s, 'name'), name = s.name; end
        if ~isfield(s, 'differential_us'), rows = add(rows, "Step " + name, "no data", "-", false); continue; end
        rows = add(rows, "Step " + name + ": " + s.expected, sprintf('%+.0f us', s.differential_us), "> +8 us", startsWith(s.result, "PASS"));
        if ~isempty(s.latency_ms), lat(end+1) = s.latency_ms; end %#ok<AGROW>
    end
    if ~isempty(lat)
        rows = add(rows, "Stick-to-motor latency, median (incl. USB, 100 Hz stream)", sprintf('%.0f ms', median(lat)), "< 50 ms", median(lat) < 50);
    end
elseif isfield(J, 'armed_ok')
    rows = add(rows, "Arming in STABILIZE", "refused", "arms", false);
end
if isfield(J, 'tilt') && isstruct(J.tilt)
    f = fieldnames(J.tilt);
    for i = 1:numel(f)
        s = J.tilt.(f{i});
        rows = add(rows, "Hand tilt " + strrep(f{i}, '_', ' ') + ": " + s.expected, sprintf('%.0f deg, %+.0f us', s.max_tilt_deg, s.differential_us), "> 8 deg, > +8 us", s.result == "PASS");
    end
end
if isfield(J, 'rc_failsafe')
    rows = add(rows, "RC loss failsafe", strjoin(string(J.rc_failsafe.events), "; "), "disarm or mode change", startsWith(J.rc_failsafe.result, "PASS"));
end
if isfield(J, 'still_armed_at_exit')
    rows = add(rows, "Disarmed at end of test", string(~J.still_armed_at_exit), "true", ~J.still_armed_at_exit);
end

R = cell2table(rows, 'VariableNames', ["Check", "Measured", "Limit", "Result"]);
disp(R);
nPass = sum(R.Result == "PASS"); fprintf('%d of %d checks passed.\n', nPass, height(R));
writetable(R, fullfile(resultDir, "l1_results.csv"));

%% Figure
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
fig = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1300 820]);
try theme(fig, 'light'); catch, end
tl = tiledlayout(fig, 3, 1, 'TileSpacing', 'compact');
title(tl, "Your FC HIL Level 1: real board, real firmware, props off");
t = S.t_host;
ax1 = nexttile(tl); hold(ax1, 'on');
plot(ax1, t, [S.m1 S.m2 S.m3 S.m4], 'LineWidth', 1.1); ylabel(ax1, 'motor PWM (us)');
legend(ax1, ["M1 front-right", "M2 rear-left", "M3 front-left", "M4 rear-right"], 'Location', 'northwest'); grid(ax1, 'on');
shadeSteps(ax1, S);
title(ax1, 'Motor outputs during stick steps and tilt tests (shaded = step active)');
ax2 = nexttile(tl);
plot(ax2, t, [S.ov1 S.ov2 S.ov3 S.ov4], 'LineWidth', 1.1); ylabel(ax2, 'RC override (us)');
legend(ax2, ["roll", "pitch", "throttle", "yaw"], 'Location', 'northwest'); grid(ax2, 'on');
title(ax2, 'Stick commands sent by MATLAB/Python pilot');
ax3 = nexttile(tl);
plot(ax3, t, rad2deg([S.roll S.pitch]), 'LineWidth', 1.1); ylabel(ax3, 'deg'); xlabel(ax3, 'time (s)');
legend(ax3, ["roll", "pitch"], 'Location', 'northwest'); grid(ax3, 'on');
title(ax3, 'Board attitude measured by the board EKF');
linkaxes([ax1 ax2 ax3], 'x');
exportgraphics(fig, fullfile(resultDir, "l1_report.png"), 'Resolution', 120);
fprintf('Saved %s\n', fullfile(resultDir, "l1_report.png"));
if strcmp(vis, 'off'), delete(fig); end
end

function rows = add(rows, c, m, l, ok)
rows(end+1, :) = {string(c), string(m), string(l), passStr(ok)};
end

function s = passStr(ok)
if ok, s = "PASS"; else, s = "FAIL"; end
end

function shadeSteps(ax, S)
ph = S.phase; on = startsWith(ph, "step:") | startsWith(ph, "tilt:");
d = diff([false; on; false]); st = find(d == 1); en = find(d == -1) - 1;
yl = ylim(ax);
for i = 1:numel(st)
    p = patch(ax, S.t_host([st(i) en(i) en(i) st(i)]), yl([1 1 2 2]), [0.85 0.9 1], 'EdgeColor', 'none', ...
        'FaceAlpha', 0.5, 'HandleVisibility', 'off');
    text(ax, S.t_host(st(i)), yl(2), extractAfter(ph(st(i)), ":"), 'FontSize', 8, 'VerticalAlignment', 'top', 'Rotation', 0);
end
uistack(findobj(ax, 'Type', 'patch'), 'bottom');
end
