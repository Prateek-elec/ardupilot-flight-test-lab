function MT = fleet_margins()
%FLEET_MARGINS Rate-loop margins from the identified plant, for two delay assumptions:
%   identified delay (biased high by closed-loop phase error -> worst case) and a
%   component-based delay (400 Hz loop 2.5 ms + ESC/DShot ~1 ms + filter group delay ~2.5 ms).
%   Gain K and motor lag tm are the well-identified parameters (95-97 % gain fit).
here = fileparts(fileparts(mfilename('fullpath')));
S = load(fullfile(here, "fleet_results", "fleet.mat")); F = S.F;
ax = ["roll", "pitch"];
cases = struct('name', {"component delay 6 ms", "identified delay (worst case)"}, 'tau', {0.006, NaN});
rows = {};
s = tf('s');
for t = 1:numel(F.tunes)
    tu = F.tunes(t);
    for a = 1:2
        id = F.ID.(ax(a));
        for c = 1:numel(cases)
            tau = cases(c).tau; if isnan(tau), tau = id.tau; end
            P = id.Krad / (s * (id.tm * s + 1)) * exp(-tau * s);
            C = tu.P(a) + tu.I(a) / s + tu.D(a) * s * (2*pi*20) / (s + 2*pi*20);
            [gm, pm, ~, wc] = margin(C * P);
            % Monte-Carlo: K +-30 %, motor lag +-30 %, delay +-50 %
            rng(7); bad = 0; n = 300;
            for i = 1:n
                Pi = id.Krad * exp(0.3 * randn) / (s * (id.tm * max(0.4, 1 + 0.3 * randn) * s + 1)) * exp(-tau * max(0.3, 1 + 0.5 * randn) * s);
                [g2, p2] = margin(C * Pi);
                if ~(20 * log10(g2) >= 6 && p2 >= 45), bad = bad + 1; end
            end
            rows(end+1, :) = {tu.name, ax(a), cases(c).name, 1000 * tau, wc / (2*pi), pm, 20 * log10(gm), 100 * bad / n}; %#ok<AGROW>
        end
    end
end
warning('off', 'all');
MT = cell2table(rows, 'VariableNames', ["Tune", "Axis", "DelayCase", "Delay_ms", "Crossover_Hz", "PhaseMargin_deg", "GainMargin_dB", "MonteCarloPoorPct"]);
disp(MT);
writetable(MT, fullfile(here, "fleet_results", "loop_margins.csv"));
end
