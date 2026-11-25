function fleet_figures()
%FLEET_FIGURES Report figures from fleet.mat and loop_margins.csv.
here = fileparts(fileparts(mfilename('fullpath'))); out = fullfile(here, "fleet_results");
F = lab_campaign_data(); fl = F.flights;
MT = readtable(fullfile(out, "loop_margins.csv"), 'TextType', 'string');
or = [0.80 0.45 0.05]; gr = [0.55 0.58 0.63];

% ---------- per-flight performance
lab = strings(1, numel(fl));
for i = 1:numel(fl)
    p = split(fl(i).id); d = p(1); fn = p(end);
    lab(i) = string(str2double(extractAfter(d, 8))) + " Nov " + fn;
end
f = figure('Visible', 'off', 'Color', 'w', 'Position', [40 40 1400 760]); try, theme(f, 'light'); catch, end
tl = tiledlayout(f, 2, 3, 'TileSpacing', 'compact');
title(tl, "Your FC flight performance, every flight (crash and upset windows removed)", 'FontWeight', 'bold');
pp = {'vibe_p95', 'Vibration, 95th percentile (m/s^2)', 30; 'att_rms_pitch', 'Pitch attitude error RMS (deg)', 3; ...
      'clips', 'Accelerometer clipping events', 0; 'alt_hold_rms', 'Altitude-hold error RMS (m)', 0.3; ...
      'motor_spread', 'Motor output spread (us)', 100; 'mag_change_pct', 'Compass field change in flight (%)', 10};
for i = 1:size(pp, 1)
    ax = nexttile(tl); y = [fl.(pp{i, 1})];
    c = repmat(gr, numel(y), 1); c(startsWith([fl.id], "2026-11-12"), :) = repmat(or, nnz(startsWith([fl.id], "2026-11-12")), 1);
    b = bar(ax, y, 0.6, 'FaceColor', 'flat', 'EdgeColor', 'none'); b.CData = c;
    yline(ax, pp{i, 3}, 'r--', "limit " + pp{i, 3}, 'LabelHorizontalAlignment', 'left', 'FontSize', 8);
    set(ax, 'XTick', 1:numel(y), 'XTickLabel', lab, 'FontSize', 9); xtickangle(ax, 35); grid(ax, 'on'); ax.XGrid = 'off';
    title(ax, pp{i, 2});
    if i == 3, set(ax, 'YScale', 'log'); ylim(ax, [0.5 5000]); end
end
subtitle(tl, "Orange: 12 Nov flights after the mechanical fixes. Grey: earlier flights.");
exportgraphics(f, fullfile(out, "fig_flights.png"), 'Resolution', 140); delete(f);

% ---------- identification + margins
f = figure('Visible', 'off', 'Color', 'w', 'Position', [40 40 1400 700]); try, theme(f, 'light'); catch, end
tl = tiledlayout(f, 2, 2, 'TileSpacing', 'compact');
title(tl, "Rate dynamics identified from 400 Hz flight data, and rate-loop margins", 'FontWeight', 'bold');
axn = ["roll", "pitch"];
for a = 1:2
    ax = nexttile(tl); id = F.ID.(axn(a)); b = id.band; fz = id.f(b);
    g = id.Krad ./ (1i * 2 * pi * fz .* (1 + 1i * 2 * pi * fz * id.tm));
    semilogx(ax, fz, 20 * log10(abs(id.P(b))), 'o', 'Color', [0.3 0.3 0.3], 'MarkerSize', 5); hold(ax, 'on');
    semilogx(ax, fz, 20 * log10(abs(g)), '-', 'Color', or, 'LineWidth', 1.8);
    grid(ax, 'on'); xlabel(ax, 'frequency (Hz)'); ylabel(ax, 'gain (dB)');
    legend(ax, ["measured (4 flights, 12 Nov)", sprintf('model K/(s(t_m s+1)): K = %.0f rad/s^2, t_m = %.0f ms', id.Krad, 1000 * id.tm)], 'Location', 'southwest');
    title(ax, sprintf('%s: controller output to body rate, gain fit %.0f %%', axn(a), id.fit));
end
ax = nexttile(tl, [1 2]);
k = MT.DelayCase == "component delay 6 ms";
T = MT(k, :); tn = unique(T.Tune, 'stable');
PM = zeros(2, numel(tn)); GM = PM;
for t = 1:numel(tn), for a = 1:2, r = T.Tune == tn(t) & T.Axis == axn(a); PM(a, t) = T.PhaseMargin_deg(r); GM(a, t) = T.GainMargin_dB(r); end, end
cats = categorical(["roll PM (deg)", "pitch PM (deg)", "roll GM (dB x5)", "pitch GM (dB x5)"], ["roll PM (deg)", "pitch PM (deg)", "roll GM (dB x5)", "pitch GM (dB x5)"]);
bar(ax, cats, [PM; 5 * GM], 0.8, 'EdgeColor', 'none'); hold(ax, 'on');
yline(ax, 45, 'r--', 'PM target 45 deg', 'LabelHorizontalAlignment', 'left'); yline(ax, 30, 'b:', 'GM target 6 dB (x5)', 'LabelHorizontalAlignment', 'right');
legend(ax, tn, 'Location', 'northeastoutside'); grid(ax, 'on');
title(ax, 'Rate-loop margins with the identified plant (component delay 6 ms; gain margin shown x5 to share the axis)');
exportgraphics(f, fullfile(out, "fig_identification.png"), 'Resolution', 140); delete(f);
fprintf('saved fig_flights.png, fig_identification.png\n');
end
