function T = pid_design_space()
%PID_DESIGN_SPACE Roll/pitch rate-PID values that meet PM >= 45 deg and GM >= 6 dB on the
%plant identified from the 12 Nov flights (K, motor lag) with a 6 ms component delay.
%Also checks robustness (K +-30 %, motor lag +-30 %, delay +-50 %, 200 cases).
%I = P (ArduPilot roll/pitch convention), D filter 20 Hz.
here = fileparts(fileparts(mfilename('fullpath')));
F = lab_campaign_data(); ID = F.ID;
s = tf('s'); warning('off', 'all');
Pg = 0.06:0.01:0.20; Dg = 0.0010:0.0005:0.0070;
ax = ["roll", "pitch"]; current = [0.13 0.004; 0.13 0.004]; tuneC = [0.135 0.0044; 0.135 0.0034];
rows = {};
for a = 1:2
    id = ID.(ax(a));
    G = id.Krad / (s * (id.tm * s + 1)) * exp(-0.006 * s);
    PM = nan(numel(Pg), numel(Dg)); GM = PM; WC = PM;
    for i = 1:numel(Pg)
        for j = 1:numel(Dg)
            C = Pg(i) + Pg(i) / s + Dg(j) * s * (2*pi*20) / (s + 2*pi*20);
            [gm, pm, ~, wc] = margin(C * G);
            PM(i, j) = pm; GM(i, j) = 20 * log10(gm); WC(i, j) = wc / (2*pi);
        end
    end
    ok = PM >= 45 & GM >= 6;
    % best = highest crossover that meets both; then robustness check
    W = WC; W(~ok) = -inf; [~, k] = max(W(:)); [i, j] = ind2sub(size(W), k);
    rob = robust(id, Pg(i), Dg(j));
    rows(end+1, :) = {ax(a), "recommended (max bandwidth meeting 45 deg / 6 dB)", Pg(i), Dg(j), WC(i, j), PM(i, j), GM(i, j), rob}; %#ok<AGROW>
    % most robust: max of min(PM-45, 7.5*(GM-6)) margin
    R = min(PM - 45, 7.5 * (GM - 6)); R(~ok) = -inf; [~, k] = max(R(:)); [i2, j2] = ind2sub(size(R), k);
    rows(end+1, :) = {ax(a), "most robust (largest margins)", Pg(i2), Dg(j2), WC(i2, j2), PM(i2, j2), GM(i2, j2), robust(id, Pg(i2), Dg(j2))}; %#ok<AGROW>
    for c = 1:2
        if c == 1, v = current(a, :); nm = "as flown 11-12 Nov"; else, v = tuneC(a, :); nm = "Tune C"; end
        C = v(1) + v(1) / s + v(2) * s * (2*pi*20) / (s + 2*pi*20);
        [gm, pm, ~, wc] = margin(C * G);
        rows(end+1, :) = {ax(a), nm, v(1), v(2), wc / (2*pi), pm, 20 * log10(gm), robust(id, v(1), v(2))}; %#ok<AGROW>
    end
    % ranges that pass, for the report
    [ii, jj] = find(ok);
    fprintf('%s: %d of %d P/D pairs pass. P range %.2f-%.2f, D range %.4f-%.4f\n', ax(a), nnz(ok), numel(ok), min(Pg(ii)), max(Pg(ii)), min(Dg(jj)), max(Dg(jj)));
    for p = Pg(unique(ii))
        dk = Dg(ok(Pg == p, :));
        fprintf('   P %.2f: D %.4f - %.4f\n', p, min(dk), max(dk));
    end
end
T = cell2table(rows, 'VariableNames', ["Axis", "Case", "P_and_I", "D", "Crossover_Hz", "PM_deg", "GM_dB", "RobustFailPct"]);
disp(T);
writetable(T, fullfile(here, "fleet_results", "pid_design_space.csv"));
end

function pct = robust(id, P, D)
s = tf('s'); rng(3); bad = 0; n = 200;
C = P + P / s + D * s * (2*pi*20) / (s + 2*pi*20);
for k = 1:n
    G = id.Krad * exp(0.3 * randn) / (s * (id.tm * max(0.4, 1 + 0.3 * randn) * s + 1)) * exp(-0.006 * max(0.3, 1 + 0.5 * randn) * s);
    [gm, pm] = margin(C * G);
    if ~(20 * log10(gm) >= 6 && pm >= 45), bad = bad + 1; end
end
pct = 100 * bad / n;
end
