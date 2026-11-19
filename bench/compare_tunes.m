function T = compare_tunes(res, tag)
%COMPARE_TUNES Side-by-side table and plots for tunes flown in SITL.
%   compare_tunes(res)        -> results/tune_comparison.png/.csv
%   compare_tunes(res, tag)   -> results/tune_comparison_<tag>.png/.csv
if nargin < 2 || strlength(tag) == 0, sfx = ""; else, sfx = "_" + tag; end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
n = numel(res); lab = strrep(string({res.label}), "_", " ");
rows = ["Roll/pitch step settle (ms)", "Roll/pitch step overshoot (%)", "Roll/pitch step tracking RMS (deg)", ...
        "Yaw: achieved / commanded (deg/s)", "Yaw 90% rise time (s)", "Yaw rate tracking RMS (deg/s)", ...
        "Altitude hold error RMS (m)", "Loiter position error RMS (m)", "Loiter position error max (m)", ...
        "Motor noise roll (%)", "Motor noise pitch (%)", "Motor noise yaw (%)", ...
        "Controller replica fit roll/pitch/yaw (%)"];
V = strings(numel(rows), n);
for k = 1:n
    A = res(k).A;
    V(:,k) = string({sprintf('%.0f', A.stepSettleMs); sprintf('%.1f', A.stepOvershoot); sprintf('%.2f', A.stepTrackRms); ...
        sprintf('%.0f / %.0f', A.yaw.rateAch, A.yaw.rateCmd); sprintf('%.2f', A.yaw.rise90); sprintf('%.1f', A.yaw.trackRms); ...
        sprintf('%.3f', A.altHoldRms); sprintf('%.2f', A.loiter.rms); sprintf('%.2f', A.loiter.max); ...
        sprintf('%.2f', A.outNoise(1)); sprintf('%.2f', A.outNoise(2)); sprintf('%.2f', A.outNoise(3)); ...
        sprintf('%.0f/%.0f/%.0f', A.ctrl(1).fit, A.ctrl(2).fit, A.ctrl(3).fit)});
end
T = array2table(V, 'RowNames', cellstr(rows), 'VariableNames', cellstr(lab));
fprintf('\n=================== TUNE COMPARISON (real ArduCopter firmware, same conditions) ===================\n');
disp(T);

outDir = fullfile(fileparts(mfilename('fullpath')), "results");
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1400 800]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact');
title(tl, 'Tune comparison on ArduCopter SITL with realistic vibration and turbulence');
cols = lines(n);
nexttile; hold on
for k = 1:n, y = res(k).A.yaw; plot(y.t, y.act, 'Color', cols(k,:), 'LineWidth', 1.6); end
plot(res(1).A.yaw.t, res(1).A.yaw.des, 'k--'); title('Yaw rate step'); xlabel('s'); ylabel('deg/s'); legend([lab "target"], 'Interpreter', 'none'); grid on
nexttile; hold on
for k = 1:n
    S = res(k).A.S; j = find(contains(stepNames(S), "PITCH forward"), 1);
    if ~isempty(j), plot(S(j).t, -S(j).act, 'Color', cols(k,:), 'LineWidth', 1.6); end
end
S = res(1).A.S; j = find(contains(stepNames(S), "PITCH forward"), 1);
if ~isempty(j), plot(S(j).t, -S(j).des, 'k--'); end
title('Pitch-forward step (nose down shown positive)'); xlabel('s'); ylabel('deg'); legend([lab "target"], 'Interpreter', 'none'); grid on
nexttile; b = cell2mat(arrayfun(@(r) r.A.outNoise(:)', res(:), 'UniformOutput', false));
bar(categorical(["roll","pitch","yaw"]), b'); title('Motor-command noise (> 20 Hz, % RMS)'); legend(lab, 'Interpreter', 'none'); grid on
nexttile; b = [arrayfun(@(r) r.A.altHoldRms, res(:)), arrayfun(@(r) r.A.loiter.rms, res(:))];
bar(categorical(["altitude hold RMS (m)","loiter RMS (m)"]), b'); title('Hold accuracy'); legend(lab, 'Interpreter', 'none', 'Location', 'northwest'); grid on
exportgraphics(f, fullfile(outDir, "tune_comparison" + sfx + ".png"), 'Resolution', 110);
if strcmp(vis, 'off'), delete(f); end
writetable(T, fullfile(outDir, "tune_comparison" + sfx + ".csv"), 'WriteRowNames', true);
fprintf('Saved %s\n', fullfile(outDir, "tune_comparison" + sfx + ".png"));
end

function n = stepNames(S)
n = strings(1, numel(S));
for i = 1:numel(S), if ~isempty(S(i).name), n(i) = string(S(i).name); end, end
end
