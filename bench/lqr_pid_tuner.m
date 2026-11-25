function R = lqr_pid_tuner(opts)
%LQR_PID_TUNER  Your FC rate and angle gains from LQR (LQI) on the flight-identified plant.
%
%   Plant per axis (gain K and motor lag tm identified from the 400 Hz flights of 12 Nov by
%   fleet_analysis.m, stored in fleet_results/fleet.mat):
%       G(s) = K e^{-tau s} / ( s (tm s + 1) )     u = rate-PID output (-1..1), w = body rate
%
%   Delay tau. The phase-based estimate in fleet.mat (12.6 ms roll, 20.3 ms pitch) predicts
%   that the tune actually flown on 12 Nov is unstable, which it was not, so it is biased
%   high. Default (Delay="replay"): tau is re-estimated by closed-loop output error: the
%   logged rate commands of the 12 Nov flights drive the model with the flown PID and tau is
%   the value whose predicted body rate best matches the measured one (1-15 Hz band).
%
%   LQI design model, states x = [z; w; m]:
%       z' = w_d - w        integral of rate error
%       w' = K m            rigid body
%       m' = (u - m) / tm   motor / ESC lag
%   State feedback u = -k x is exactly a PID, because m = w'/K:
%       P = k_w,   I = -k_z,   D = k_m / K
%   R = K^2 normalises the input to angular acceleration, so the same Q gives the same
%   closed-loop bandwidth on every axis. Each Q = diag(q_z, q_w, q_m) gives one PID.
%
%   The LQR margin guarantee (>= 60 deg) is lost to the delay and ArduPilot's 20 Hz D filter,
%   so each candidate PID is evaluated on the DELAYED plant with the D filter, and kept if
%     1. PM >= PMmin and GM >= GMmin at the design delay AND at the 6 ms component delay,
%     2. its crossover lies inside the band the identification is valid for,
%     3. at most MCmaxFail % of Monte-Carlo plants (K +-30 %, motor lag +-30 %,
%        delay +-50 %) fall below PMmin / GMmin.
%   The highest-bandwidth survivor is picked. The angle P is then chosen on the closed rate
%   loop. All tunes are replayed against the commanded rates of the 12 Nov flights.
%
%   Yaw: 2-state LQI (z, w), D = 0. Its identification is only coherent over 1.5-2.3 Hz,
%   so treat the yaw result as low confidence.
%
%   R = lqr_pid_tuner();                         % delay from flight replay (default)
%   R = lqr_pid_tuner(Delay="identified");       % worst case: phase-based delay
%   R = lqr_pid_tuner(Delay="component");        % 6 ms (loop + ESC + filters)
%   R = lqr_pid_tuner(MCmaxFail=10, PMmin=50);   % more conservative
%
%   Outputs in fleet_results/lqr_tuner: gains.csv, design_space.csv, replay.csv,
%   fc_lqr_tune.param and figures 1_design_space.png ... 5_delay_fit.png.

arguments
    opts.Delay (1,1) string {mustBeMember(opts.Delay, ["replay", "identified", "component"])} = "replay"
    opts.PMmin (1,1) double = 50            % deg, nominal model (well damped: ~10-15 % overshoot)
    opts.GMmin (1,1) double = 6             % dB, nominal model
    opts.PMrobust (1,1) double = 35         % deg: a what-if plant below this (or GMrobust) counts as poorly damped
    opts.GMrobust (1,1) double = 4          % dB
    opts.MCmaxFail (1,1) double = 10        % % of Monte-Carlo plants allowed to be poorly damped
    opts.MsMax (1,1) double = 1.8           % peak sensitivity limit (nominal model)
    opts.MaxDecrease (1,1) double = 0.7     % step 1 may lower a gain to at most this factor (-30 %)
    opts.LocalSearch (1,1) logical = true   % also search P/I/D directly around the current tune
    opts.Dmax (1,1) double = 0.006          % D cap: D amplifies gyro noise into the motors
    opts.DFilterHz (1,1) double = 20        % ATC_RAT_xxx_FLTD
    opts.TargetFilterHz (1,1) double = 20   % ATC_RAT_xxx_FLTT
    opts.MaxIncrease (1,1) double = 1.5     % step 1 may raise a gain by at most this factor
    opts.IoverP (1,2) double = [0.8 2]      % allowed I/P, roll and pitch (ArduPilot AutoTune sets I = P)
    opts.IoverPYaw (1,2) double = [0.1 0.5] % allowed I/P, yaw (ArduPilot default I = P/10)
    opts.Current struct = struct('P', [0.13 0.13 0.13], 'I', [0.14 0.14 0.135], 'D', [0.004 0.004 0], 'AngP', [4.5 4.5 4.5])
    opts.OutDir (1,1) string = fullfile(fileparts(fileparts(mfilename('fullpath'))), "fleet_results", "lqr_tuner")
    opts.F struct = struct()                % identified data (fields ID, flights); empty = fleet_results/fleet.mat
    opts.KeepAxes string = strings(0)       % axes written to the .param unchanged (e.g. "yaw" when its model is unusable)
    opts.Plots (1,1) logical = true
    opts.Progress = []                      % optional @(fraction, message) callback (used by fc_tuner_app)
    opts.FigureVisible (1,1) string {mustBeMember(opts.FigureVisible, ["auto", "on", "off"])} = "auto"   % "off": draw to PNG only (no windows)
    opts.CurrentLabel (1,1) string = "as flown 12 Nov"   % name of the tune that was flying, in tables and legends
    opts.CompareTuneC (1,1) logical = true               % also evaluate Tune C (post_crash_fix.param)
end
warning('off', 'all');
here = fileparts(fileparts(mfilename('fullpath')));
if isempty(fieldnames(opts.F))
    F = lab_campaign_data();
else
    F = opts.F;
end
prog = @(f, msg) []; if ~isempty(opts.Progress), prog = opts.Progress; end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
if opts.FigureVisible == "auto"
    if usejava('desktop'), vis = 'on'; else, vis = 'off'; end
else
    vis = char(opts.FigureVisible);
end
tauComp = 0.006;                % component estimate: loop + ESC + filters
tauMin = 0.004;                 % physical floor: 400 Hz loop (~2.5 ms) + ESC/DShot (~1 ms)
axn = ["roll", "pitch", "yaw"];
tuneC = struct('P', [0.135 0.135 0.34], 'I', [0.135 0.135 0.034], 'D', [0.0044 0.0034 0]);   % post_crash_fix.param

fprintf('\n================ LQR -> PID TUNER (design delay: %s) ================\n', opts.Delay);
R = struct(); rows = {}; dsRows = {};
for a = 1:3
    prog(0.1 + 0.25 * (a - 1), "LQR design: " + axn(a));
    id = F.ID.(axn(a));
    cur = struct('P', opts.Current.P(a), 'I', opts.Current.I(a), 'D', opts.Current.D(a));
    M.K = id.Krad; M.tm = id.tm; M.fmax = id.fband(2); M.wd = 2 * pi * opts.DFilterHz; M.tau = tauComp;

    % ---- 0. model: joint closed-loop output-error fit of K, motor lag and delay to the flights
    [tauRep, dfit] = delayFromReplay(F, M, cur, opts, a);
    Mfd = M; Mfd.tau = tauRep;
    [Mfit, fitInfo] = refineModel(F, Mfd, cur, opts, a);
    fitInfo.used = fitInfo.fitAll >= fitInfo.fitStart + 3;    % accept the refit only if it clearly explains the flight better
    switch opts.Delay
        case "replay"
            if fitInfo.used, M = Mfit; else, M = Mfd; end
            M.tau = max(M.tau, tauMin);                   % never below loop + ESC delay
        case "identified", M.tau = max(id.tau, tauComp);
        case "component",  M.tau = tauComp;
    end
    fprintf('\n--- %s: model valid to %.1f Hz\n', axn(a), M.fmax);
    fprintf('    frequency-domain fit: K %4.0f  motor lag %4.1f ms  delay %4.1f ms (phase) / %4.1f ms (replay)\n', ...
        id.Krad, 1000 * id.tm, 1000 * id.tau, 1000 * tauRep);
    fprintf('    flight-fitted model : K %4.0f  motor lag %4.1f ms  delay %4.1f ms | flight fit %4.1f %% (was %4.1f %%), hold-out flight %4.1f %%\n', ...
        Mfit.K, 1000 * Mfit.tm, 1000 * Mfit.tau, fitInfo.fitAll, fitInfo.fitStart, fitInfo.fitHold);
    fprintf('    design model        : K %4.0f  motor lag %4.1f ms  delay %4.1f ms (%s)\n', M.K, 1000 * M.tm, 1000 * M.tau, ...
        string(fitInfo.used).replace(["true", "false"], ["flight-fitted", "frequency-domain; refit not clearly better"]));

    % ---- 1. candidates: LQR family + direct P/I/D grid around the current tune
    C = sweepLQR(M, a == 3); C.src = repmat("LQR", numel(C.P), 1);
    if opts.LocalSearch
        Lg = localGrid(cur, a == 3); Lg.src = repmat("local", numel(Lg.P), 1);
        for fn = string(fieldnames(C))', C.(fn) = [C.(fn); Lg.(fn)]; end
    end
    n = numel(C.P);
    [C.PM, C.GM, C.wc, C.stab, C.J, C.Ms] = deal(nan(n, 1), nan(n, 1), nan(n, 1), false(n, 1), nan(n, 1), nan(n, 1));
    for i = 1:n
        [C.PM(i), C.GM(i), C.wc(i), C.stab(i)] = fastMargins(M, C.P(i), C.I(i), C.D(i), M.tau, 1, 1);
        [C.J(i), C.Ms(i)] = perfIndex(M, C.P(i), C.I(i), C.D(i), M.tau);
    end
    ok = C.stab & C.PM >= opts.PMmin & C.GM >= opts.GMmin & C.Ms <= opts.MsMax & C.D <= opts.Dmax ...
        & C.P > 0 & C.I > 0 & C.wc / (2*pi) <= M.fmax;
    % integrator strength: enough I to hold attitude against wind and CG offset
    if a == 3, ior = opts.IoverPYaw; else, ior = opts.IoverP; end
    ok = ok & C.I >= ior(1) * C.P & C.I <= ior(2) * C.P;
    fprintf('    %d candidates (%d LQR, %d local) -> %d meet the nominal spec (PM >= %g deg, GM >= %g dB)\n', ...
        n, nnz(C.src == "LQR"), nnz(C.src == "local"), nnz(ok), opts.PMmin, opts.GMmin);

    % ---- 2. robustness (Monte Carlo), best disturbance rejection / tracking first
    %         (J = mean log|S| over 0.5-5 Hz, lower = better; set mainly by P and I);
    %         stop at the first candidate that passes
    C.MC = nan(n, 1);
    cand = find(ok); [~, o] = sort(C.J(cand), 'ascend'); cand = cand(o);
    pick = [];
    for i = cand'
        C.MC(i) = monteCarlo(M, C.P(i), C.I(i), C.D(i), opts, 200, 11);
        if C.MC(i) <= opts.MCmaxFail, pick = i; break; end
    end
    if isempty(pick) && ~isempty(cand)
        [~, j] = min(C.MC(cand)); pick = cand(j);
        fprintf('    WARNING: no PID meets the %.0f %% Monte-Carlo limit; taking the most robust (%.0f %%)\n', opts.MCmaxFail, C.MC(pick));
    end
    if isempty(pick), error('lqr_pid_tuner:none', '%s: no candidate gives an acceptable PID. Relax PMmin/GMmin.', axn(a)); end
    [J0, Ms0] = perfIndex(M, cur.P, cur.I, cur.D, M.tau);
    fprintf('    picked: %s candidate, P %.3f I %.3f D %.4f, %.1f Hz, PM %.1f deg, GM %.1f dB, Ms %.2f, rejection %.1f dB (current %.1f dB, Ms %.2f), MC poorly damped %.0f %%\n', ...
        C.src(pick), C.P(pick), C.I(pick), C.D(pick), C.wc(pick) / (2*pi), C.PM(pick), C.GM(pick), C.Ms(pick), ...
        20 * C.J(pick) / log(10), 20 * J0 / log(10), Ms0, C.MC(pick));
    for i = 1:n
        dsRows(end+1, :) = {axn(a), C.qw(i), C.qz(i), C.qm(i), C.P(i), C.I(i), C.D(i), C.wc(i)/(2*pi), C.PM(i), C.GM(i), C.stab(i), ok(i), C.MC(i), i == pick}; %#ok<AGROW>
    end

    % ---- 3. round to parameter resolution
    T.P = round(C.P(pick), 3); T.I = round(C.I(pick), 3); T.D = round(C.D(pick), 4);
    T.q = [C.qz(pick) C.qw(pick) C.qm(pick)];
    Tg = rmfield(T, 'q');

    % ---- 4. step 1: move from the current tune toward the target along a geometric path,
    %         no gain changing by more than MaxIncrease / MaxDecrease per flight
    S1 = stepToward(cur, Tg, opts, a == 3);
    [pm1, gm1, ~, st1] = fastMargins(M, S1.P, S1.I, S1.D, M.tau, 1, 1);
    [pm0, gm0] = fastMargins(M, cur.P, cur.I, cur.D, M.tau, 1, 1);
    if ~st1 || (pm1 < min(opts.PMmin, pm0) - 1) || (gm1 < min(opts.GMmin, gm0) - 0.5)
        fprintf('    note: limited step 1 would be less stable than the current tune; using the target directly\n');
        S1 = Tg;
    end

    % ---- 5. angle loop on the closed rate loop
    [angP, angRaw, angPM] = angleP(M, Tg, opts, opts.Current.AngP(a));

    % ---- 6. evaluate all cases (exact allmargin as a cross-check of the fast margins)
    cases = {opts.CurrentLabel, cur; "Tune C (post_crash_fix)", struct('P', tuneC.P(a), 'I', tuneC.I(a), 'D', tuneC.D(a)); ...
             "LQR step 1", S1; "LQR target", Tg};
    for c = 1:size(cases, 1)
        if c == 2 && ~opts.CompareTuneC, continue; end
        g = cases{c, 2};
        [pm, gm, wc, st] = loopMargins(M, g.P, g.I, g.D, M.tau);
        [pm6, gm6] = loopMargins(M, g.P, g.I, g.D, tauComp);
        [pmI, gmI] = loopMargins(M, g.P, g.I, g.D, max(id.tau, tauComp));
        mc = monteCarlo(M, g.P, g.I, g.D, opts, 500, 7);
        [ov, tr, dpk] = stepMetrics(M, g, opts);
        rows(end+1, :) = {axn(a), cases{c, 1}, g.P, g.I, g.D, 1000 * M.tau, wc/(2*pi), pm, gm, st, pm6, gm6, pmI, gmI, mc, tr, ov, dpk}; %#ok<AGROW>
        fprintf('    %-24s P %.3f I %.3f D %.4f | %5.1f Hz  PM %5.1f  GM %5.1f dB %-8s | 6 ms: PM %5.1f GM %5.1f | phase-delay: PM %6.1f GM %5.1f | MC fail %3.0f %% | rise %3.0f ms  overshoot %3.0f %%\n', ...
            cases{c, 1}, g.P, g.I, g.D, wc/(2*pi), pm, gm, string(st).replace(["true","false"], ["stable","UNSTABLE"]), pm6, gm6, pmI, gmI, mc, 1000*tr, ov);
    end
    fprintf('    angle P: %.2f (unclamped %.2f, outer PM %.0f deg)\n', angP, angRaw, angPM);

    R.(axn(a)) = struct('model', M, 'sweep', C, 'pick', pick, 'target', T, 'step1', S1, 'current', cur, ...
        'tuneC', cases{2, 2}, 'angP', angP, 'angPraw', angRaw, 'delayFit', dfit, 'tauPhase', id.tau, ...
        'fitInfo', fitInfo, 'modelFD', Mfd, 'source', C.src(pick));
end

GT = cell2table(rows, 'VariableNames', ["Axis", "Case", "P", "I", "D", "DesignDelay_ms", "Crossover_Hz", "PM_deg", "GM_dB", "Stable", ...
    "PM_6ms_deg", "GM_6ms_dB", "PM_phaseDelay_deg", "GM_phaseDelay_dB", "MonteCarloFailPct", "Rise_s", "Overshoot_pct", "DistPeak_degps"]);
DS = cell2table(dsRows, 'VariableNames', ["Axis", "q_w", "q_z", "q_m", "P", "I", "D", "Crossover_Hz", "PM_deg", "GM_dB", "Stable", "PassNominal", "MonteCarloFailPct", "Picked"]);

% ---- replay on the 12 Nov flights
prog(0.85, "Replaying the logged flights");
fprintf('\n================ REPLAY: commanded rates of the 400 Hz flights ================\n');
RP = replay(F, R, opts);
disp(RP);
R.gains = GT; R.replay = RP;
writetable(GT, fullfile(opts.OutDir, "gains.csv"));
writetable(DS, fullfile(opts.OutDir, "design_space.csv"));
writetable(RP, fullfile(opts.OutDir, "replay.csv"));
writeParams(R, opts, fullfile(opts.OutDir, "fc_lqr_tune.param"));
if opts.Plots, prog(0.92, "Drawing figures"); plotAll(R, F, opts, vis); end
fprintf('\nWritten to %s\n', opts.OutDir);
end

%% =====================================================================
function C = sweepLQR(M, isYaw)
qw = logspace(0, 6, 61);                  % rate weight -> bandwidth
al = logspace(-2.5, 1, 8);                % q_z / q_w  -> integral strength (I/P ~ sqrt)
be = [0 logspace(-6, -2, 5)];             % q_m / q_w  -> motor-state (D) weight
if isYaw, be = 0; end
[QW, AL, BE] = ndgrid(qw, al, be);
n = numel(QW);
C = struct('qw', QW(:), 'qz', QW(:) .* AL(:), 'qm', QW(:) .* BE(:), 'P', nan(n, 1), 'I', nan(n, 1), 'D', nan(n, 1));
Rw = M.K ^ 2;                              % input weight on K*u (angular acceleration)
for i = 1:n
    if isYaw
        A = [0 -1; 0 0]; B = [0; M.K];                         % x = [z; w]; 5 ms motor lag ignored in design
        k = lqr(A, B, diag([C.qz(i) C.qw(i)]), Rw);
        C.P(i) = k(2); C.I(i) = -k(1); C.D(i) = 0;
    else
        A = [0 -1 0; 0 0 M.K; 0 0 -1/M.tm]; B = [0; 0; 1/M.tm];
        k = lqr(A, B, diag([C.qz(i) C.qw(i) C.qm(i) * M.K^2]), Rw);
        C.P(i) = k(2); C.I(i) = -k(1); C.D(i) = k(3) / M.K;
    end
end
end

function Cc = pidTF(M, P, I, D)
s = tf('s');
Cc = P + I / s + D * s * M.wd / (s + M.wd);
end

function G = plantTF(M, tau)
s = tf('s');
G = M.K / (s * (M.tm * s + 1));
G.InputDelay = tau;
end

function [PM, GM, wc, stab] = fastMargins(M, P, I, D, tau, kS, tS)
% Margins from the analytic frequency response on a dense grid. The phase is built
% analytically (no unwrap ambiguity): Re{C(jw)} > 0, so angle(C) is in (-90, 90).
persistent w
if isempty(w), w = logspace(-1, log10(2 * pi * 200), 5000)'; end
jw = 1i * w;
Cw = P + I ./ jw + D * jw * M.wd ./ (jw + M.wd);
mag = abs(Cw) * M.K * kS ./ (w .* sqrt(1 + (M.tm * tS * w).^2));
ph = rad2deg(angle(Cw)) - 90 - rad2deg(atan(M.tm * tS * w)) - rad2deg(w * tau);
gx = find(diff(sign(mag - 1)) ~= 0);                  % gain crossings
if isempty(gx), PM = inf; wc = NaN; else, [PM, j] = min(180 + ph(gx)); wc = w(gx(j)); end
r = mod(ph + 180, 360) - 180;                           % phase relative to -180 (mod 360)
px = find(diff(sign(r)) ~= 0 & abs(diff(r)) < 180);     % -180 (mod 360) crossings
if isempty(px), GM = inf; else, GM = min(-20 * log10(mag(px))); end
stab = PM > 0 && GM > 0;
end

function [PM, GM, wc, stab] = loopMargins(M, P, I, D, tau)
L = pidTF(M, P, I, D) * plantTF(M, tau);
Sm = allmargin(L);
g = Sm.GainMargin; g = g(g > 1);
if isempty(g), GM = inf; else, GM = 20 * log10(min(g)); end
if isempty(Sm.PhaseMargin), PM = -inf; wc = NaN;
else, [PM, j] = min(Sm.PhaseMargin); wc = Sm.PMFrequency(j); end
stab = Sm.Stable == 1;
end

function pct = monteCarlo(M, P, I, D, opts, n, seed)
% share of what-if plants (K +-30 %, motor lag +-30 %, delay +-50 %, 1 sigma) that would be
% unstable or poorly damped (PM < PMrobust or GM < GMrobust)
rng(seed); bad = 0;
for i = 1:n
    kS = exp(0.3 * randn); tS = max(0.4, 1 + 0.3 * randn); dS = max(0.3, 1 + 0.5 * randn);
    [pm, gm, ~, st] = fastMargins(M, P, I, D, M.tau * dS, kS, tS);
    if ~st || pm < opts.PMrobust || gm < opts.GMrobust, bad = bad + 1; end
end
pct = 100 * bad / n;
end

function [J, Ms] = perfIndex(M, P, I, D, tau)
% J : mean log|S(jw)| over 0.5-5 Hz (pilot commands, wind, CG offset live here; set by P and I).
%     More negative = rejects disturbances / follows commands better.
% Ms: peak sensitivity max|S| (robustness: 1/Ms = closest approach to -1).
persistent wp wa
if isempty(wp), wp = 2 * pi * logspace(log10(0.5), log10(5), 40)'; wa = 2 * pi * logspace(-1, log10(200), 3000)'; end
Lf = @(w) (P + I ./ (1i * w) + D * (1i * w) * M.wd ./ (1i * w + M.wd)) .* M.K .* exp(-1i * w * tau) ./ ((1i * w) .* (M.tm * 1i * w + 1));
J = mean(log(abs(1 ./ (1 + Lf(wp)))));
Ms = max(abs(1 ./ (1 + Lf(wa))));
end

function C = localGrid(cur, isYaw)
% direct P/I/D candidates around the tune that flew (the LQR family has a fixed D/P shape)
f = [0.4 0.5 0.6 0.7 0.8 0.9 1 1.1 1.2 1.35 1.5 1.7 2];
if isYaw
    d = unique([0 cur.D]);
else
    d0 = cur.D; if d0 <= 0, d0 = 0.003; end
    d = d0 * [0.5 0.6 0.7 0.8 0.9 1 1.1 1.2 1.35 1.5];
end
[Pf, If, Dv] = ndgrid(cur.P * f, cur.I * f, d);
n = numel(Pf);
C = struct('qw', nan(n, 1), 'qz', nan(n, 1), 'qm', nan(n, 1), 'P', round(Pf(:), 3), 'I', round(If(:), 3), 'D', round(Dv(:), 4));
end

function S1 = stepToward(cur, T, opts, isYaw)
% geometric path cur -> T; alpha in (0,1] chosen so no gain changes by more than the limits
c = [cur.P cur.I cur.D]; t = [T.P T.I T.D];
alpha = 1;
for k = 1:3
    if c(k) <= 0 || t(k) <= 0, continue; end          % D appearing/disappearing handled below
    r = t(k) / c(k);
    if r > opts.MaxIncrease, alpha = min(alpha, log(opts.MaxIncrease) / log(r)); end
    if r < opts.MaxDecrease, alpha = min(alpha, log(opts.MaxDecrease) / log(r)); end
end
s = c;
for k = 1:3
    if c(k) > 0 && t(k) > 0, s(k) = c(k) * (t(k) / c(k)) ^ alpha;
    elseif t(k) == 0, s(k) = 0;                          % e.g. yaw D -> 0
    else, s(k) = t(k) * alpha; end
end
if isYaw && t(3) == 0, s(3) = 0; end
S1 = struct('P', round(s(1), 3), 'I', round(s(2), 3), 'D', round(s(3), 4));
end

function [M, info] = refineModel(F, M0, g, opts, a)
% Joint closed-loop output-error fit of K, motor lag tm and delay tau: the tune that flew,
% driven by the logged rate commands, must reproduce the measured body rate (1-15 Hz).
% Fitted on all flights but one, checked on the held-out flight, then refitted on all.
fl = F.flights([F.flights.rate_hz] > 300);
[b, aa] = butter(2, [1 15] / 200, 'bandpass');
for k = 1:numel(fl)
    d(k).wd = fl(k).ratedata.wd(:, a); d(k).w = fl(k).ratedata.w(:, a); %#ok<AGROW>
    d(k).wf = filtfilt(b, aa, d(k).w); %#ok<AGROW>
end
x0 = [log(M0.K), log(M0.tm), M0.tau];
lb = [log(M0.K) - log(3), log(0.003), 0.002]; ub = [log(M0.K) + log(3), log(0.08), 0.03];
o = optimset('Display', 'off', 'MaxFunEvals', 160, 'TolX', 1e-3, 'TolFun', 1e-4);
cost = @(x, idx) oeCost(x, idx, M0, g, opts, d, b, aa, lb, ub);
info.fitStart = 100 * (1 - cost(x0, 1:numel(d)));
info.fitHold = NaN;
if numel(d) >= 2
    hIdx = numel(d); train = 1:numel(d) - 1;
    xh = fminsearch(@(x) cost(x, train), x0, o);
    info.fitHold = 100 * (1 - cost(xh, hIdx));
end
x = fminsearch(@(x) cost(x, 1:numel(d)), x0, o);
info.fitAll = 100 * (1 - cost(x, 1:numel(d)));
if info.fitAll < info.fitStart, x = x0; info.fitAll = info.fitStart; end    % never worse than the start
M = M0; M.K = exp(x(1)); M.tm = exp(x(2)); M.tau = x(3);
info.x = x;
end

function c = oeCost(x, idx, M0, g, opts, d, b, aa, lb, ub)
if any(x < lb) || any(x > ub), c = 10; return; end
M = M0; M.K = exp(x(1)); M.tm = exp(x(2)); M.tau = x(3);
[pm, ~, ~, st] = fastMargins(M, g.P, g.I, g.D, M.tau, 1, 1);
if ~st || pm <= 2, c = 10; return; end                 % the flown tune flew: model must keep it stable
Td = c2d(pade(closedLoops(M, g, opts), 3), 1 / 400);
num = 0; den = 0;
for k = idx
    y = lsim(Td, d(k).wd - d(k).wd(1)) + d(k).wd(1);
    e = filtfilt(b, aa, d(k).w - y);
    num = num + sum(e.^2); den = den + sum(d(k).wf.^2);
end
c = sqrt(num / den);
end

function [T, Tdist] = closedLoops(M, g, opts)
% ArduPilot rate PID acts on error (target low-passed by FLTT); D on error, filtered
s = tf('s');
Cc = pidTF(M, g.P, g.I, g.D); G = plantTF(M, M.tau);
Ft = 1 / (s / (2 * pi * opts.TargetFilterHz) + 1);
T = Ft * feedback(Cc * G, 1);
Tdist = feedback(G, Cc);                 % disturbance at the plant input
end

function [ov, tr, dpk] = stepMetrics(M, g, opts)
[T, Td] = closedLoops(M, g, opts);
t = (0:0.0005:1.5)';
y = step(T, t); si = stepinfo(y, t, 1);
ov = si.Overshoot; tr = si.RiseTime;
yd = step(Td * 0.05, t);                 % 5 % motor-output disturbance
dpk = rad2deg(max(abs(yd)));
end

function [angP, raw, pmo] = angleP(M, T, opts, curAng)
% outer loop: rate target = angP * angle error; inner = closed rate loop.
% The linear model ignores ArduPilot's input shaping and sqrt controller, so it is only
% allowed to LOWER the angle P (when the rate loop is too slow for it), never raise it.
s = tf('s');
Tr = closedLoops(M, T, opts);
best = NaN; pmo = NaN;
for p = 2:0.25:12
    Sm = allmargin(p / s * Tr);
    g = Sm.GainMargin; g = g(g > 1); gm = inf; if ~isempty(g), gm = 20 * log10(min(g)); end
    if Sm.Stable == 1 && ~isempty(Sm.PhaseMargin) && min(Sm.PhaseMargin) >= 60 && gm >= 10
        best = p; pmo = min(Sm.PhaseMargin);
    end
end
raw = best;
angP = max(min(best, curAng), 2.5);
if isnan(best), angP = 2.5; end
end

function [y, w, wd] = simFlight(Td, fl, a)
wd = fl.ratedata.wd(:, a); w = fl.ratedata.w(:, a);
y = lsim(Td, wd - wd(1)) + wd(1);
end

function [tauHat, out] = delayFromReplay(F, M, g, opts, a)
% Closed-loop output-error delay estimate: the flown PID, driven by the logged rate
% commands, must reproduce the measured body rate. Compared in the 1-15 Hz band, where
% the rate loop (not wind or the attitude loop) sets the response.
fl = F.flights([F.flights.rate_hz] > 300);
[b, aa] = butter(2, [1 15] / 200, 'bandpass');
taus = (0.002:0.001:0.030)'; fitv = nan(size(taus));
for i = 1:numel(taus)
    M.tau = taus(i);
    [pm, ~, ~, st] = fastMargins(M, g.P, g.I, g.D, taus(i), 1, 1);
    if ~st || pm <= 2, continue; end                    % the flown tune flew: delay cannot destabilise it
    Td = c2d(pade(closedLoops(M, g, opts), 3), 1 / 400);
    num = 0; den = 0;
    for k = 1:numel(fl)
        [y, w] = simFlight(Td, fl(k), a);
        e = filtfilt(b, aa, w - y); wf = filtfilt(b, aa, w);
        num = num + sum(e.^2); den = den + sum(wf.^2);
    end
    fitv(i) = 100 * (1 - sqrt(num / den));
end
[~, j] = max(fitv); tauHat = taus(j);
if isempty(j) || isnan(fitv(j)), tauHat = 0.006; end
out = struct('tau', taus, 'fit', fitv);
end

function RP = replay(F, R, opts)
% Drive each closed-loop model with the commanded rate RDes of every 400 Hz flight.
% ModelFit: predicted vs measured body rate with the tune that was flown (model check).
fl = F.flights([F.flights.rate_hz] > 300);
axn = ["roll", "pitch", "yaw"]; rows = {};
for a = 1:3
    r = R.(axn(a)); M = r.model;
    cs = {opts.CurrentLabel, r.current; "Tune C (post_crash_fix)", r.tuneC; "LQR step 1", r.step1; "LQR target", rmfield(r.target, 'q')};
    for c = 1:size(cs, 1)
        if c == 2 && ~opts.CompareTuneC, continue; end
        Td = c2d(pade(closedLoops(M, cs{c, 2}, opts), 3), 1 / 400);
        e2 = 0; n = 0; m2 = 0; d2 = 0;
        for k = 1:numel(fl)
            [y, w, wd] = simFlight(Td, fl(k), a);
            e2 = e2 + sum((wd - y).^2); n = n + numel(y);
            m2 = m2 + sum((w - y).^2); d2 = d2 + sum((w - mean(w)).^2);
        end
        mfit = NaN; if c == 1, mfit = 100 * (1 - sqrt(m2 / d2)); end
        rows(end+1, :) = {axn(a), cs{c, 1}, rad2deg(sqrt(e2 / n)), mfit}; %#ok<AGROW>
    end
end
RP = cell2table(rows, 'VariableNames', ["Axis", "Case", "PredTrackErrRMS_degps", "ModelFitToMeasured_pct"]);
end

function writeParams(R, opts, file)
fid = fopen(file, 'w'); c = onCleanup(@() fclose(fid));
nm = ["RLL", "PIT", "YAW"]; axn = ["roll", "pitch", "yaw"];
fprintf(fid, '# Your FC - rate/angle gains from bench/lqr_pid_tuner.m (LQR on the flight-identified plant)\n');
fprintf(fid, '# Design delay: %s. Required: PM >= %.0f deg, GM >= %.0f dB, Monte-Carlo fail <= %.0f %%.\n', opts.Delay, opts.PMmin, opts.GMmin, opts.MCmaxFail);
fprintf(fid, '# STEP 1 values: increases limited to x%.1f of the flown tune; reductions go straight in.\n', opts.MaxIncrease);
fprintf(fid, '# Fly a STABILIZE/ALT_HOLD hover 1-2 min, check the log, then move to the LQR target (comments).\n');
fprintf(fid, '# Load in Mission Planner: CONFIG > Full Parameter List > Load from file > Write Params.\n#\n');
for a = 1:3
    r = R.(axn(a)); S1 = r.step1; T = r.target;
    if any(opts.KeepAxes == axn(a))
        fprintf(fid, '# %s: model not reliable enough - current values kept (LQR would give P %.3f I %.3f D %.4f)\n', axn(a), T.P, T.I, T.D);
        S1 = r.current; r.angP = opts.Current.AngP(a);
    elseif a == 3
        fprintf(fid, '# yaw: lower confidence than roll/pitch (yaw identification band is narrow)\n');
    end
    fprintf(fid, '# %s target: P %.3f  I %.3f  D %.4f   (flown: P %.3f I %.3f D %.4f), design delay %.1f ms\n', ...
        axn(a), T.P, T.I, T.D, r.current.P, r.current.I, r.current.D, 1000 * r.model.tau);
    fprintf(fid, 'ATC_RAT_%s_P,%.3f\n', nm(a), S1.P);
    fprintf(fid, 'ATC_RAT_%s_I,%.3f\n', nm(a), S1.I);
    fprintf(fid, 'ATC_RAT_%s_D,%.4f\n', nm(a), S1.D);
    fprintf(fid, 'ATC_ANG_%s_P,%.2f\n', nm(a), r.angP);
end
fprintf(fid, '# D filter assumed %g Hz, target filter %g Hz (ATC_RAT_xxx_FLTD / FLTT): keep those as set.\n', opts.DFilterHz, opts.TargetFilterHz);
end

%% =====================================================================
function plotAll(R, F, opts, vis)
axn = ["roll", "pitch", "yaw"]; col = [0.45 0.45 0.45; 0.20 0.45 0.80; 0.85 0.50 0.10; 0.80 0.15 0.15];
cn = [opts.CurrentLabel, "Tune C (post_crash_fix)", "LQR step 1", "LQR target"];
show = 1:4; if ~opts.CompareTuneC, show = [1 3 4]; end
save1 = @(f, nm) exportgraphics(f, fullfile(opts.OutDir, nm), 'Resolution', 130);

% 1. design space
f = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1500 480]); try theme(f, 'light'); catch, end
tl = tiledlayout(f, 1, 3, 'TileSpacing', 'compact');
title(tl, sprintf("LQR weight sweep -> PID on the delayed plant (design delay: %s)", opts.Delay), 'FontWeight', 'bold');
for a = 1:3
    ax = nexttile(tl); C = R.(axn(a)).sweep; tst = isfinite(C.MC);
    scatter(ax, C.wc/(2*pi), C.PM, 8, [0.8 0.8 0.8], 'filled'); hold(ax, 'on');
    if any(tst), scatter(ax, C.wc(tst)/(2*pi), C.PM(tst), 22, C.MC(tst), 'filled'); colormap(ax, flipud(autumn)); clim(ax, [0 100]); cb = colorbar(ax); cb.Label.String = 'Monte-Carlo fail %'; end
    p = R.(axn(a)).pick; plot(ax, C.wc(p)/(2*pi), C.PM(p), 'kp', 'MarkerSize', 16, 'MarkerFaceColor', 'g');
    yline(ax, opts.PMmin, 'r--'); xline(ax, R.(axn(a)).model.fmax, 'b:', 'model valid to');
    ylim(ax, [-30 100]); xlabel(ax, 'crossover (Hz)'); ylabel(ax, 'phase margin (deg)'); grid(ax, 'on'); title(ax, axn(a) + " (star = picked)");
end
save1(f, "1_design_space.png"); if vis == "off", delete(f); end

% 2. Nichols and step
f2 = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1500 820]); try theme(f2, 'light'); catch, end
tl2 = tiledlayout(f2, 2, 3, 'TileSpacing', 'compact'); title(tl2, "Open loop with the design delay, and closed-loop rate step", 'FontWeight', 'bold');
w = logspace(log10(2*pi*0.3), log10(2*pi*60), 400);
for a = 1:3
    r = R.(axn(a)); M = r.model; cs = {r.current, r.tuneC, r.step1, rmfield(r.target, 'q')};
    ax = nexttile(tl2, a); hold(ax, 'on');
    for c = show
        h = squeeze(freqresp(pidTF(M, cs{c}.P, cs{c}.I, cs{c}.D) * plantTF(M, M.tau), w));
        plot(ax, rad2deg(unwrap(angle(h))), 20*log10(abs(h)), 'Color', col(c, :), 'LineWidth', 1.4);
    end
    plot(ax, -180, 0, 'kx', 'MarkerSize', 12, 'LineWidth', 2);
    xlabel(ax, 'open-loop phase (deg)'); ylabel(ax, 'open-loop gain (dB)'); grid(ax, 'on'); xlim(ax, [-300 -60]); ylim(ax, [-30 30]);
    title(ax, axn(a) + ": Nichols (stay away from the x)");
    if a == 1, legend(ax, cn(show), 'Location', 'southeast', 'Interpreter', 'none'); end
    ax = nexttile(tl2, 3 + a); hold(ax, 'on'); t = (0:0.0005:0.6)';
    for c = show
        y = step(closedLoops(M, cs{c}, opts), t);
        plot(ax, 1000*t, y, 'Color', col(c, :), 'LineWidth', 1.4);
    end
    xlabel(ax, 'ms'); ylabel(ax, 'rate / commanded'); grid(ax, 'on'); title(ax, axn(a) + ": rate step response"); ylim(ax, [0 1.8]);
end
save1(f2, "2_loops.png"); if vis == "off", delete(f2); end

% 3. margins summary
GT = R.gains;
f3 = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1500 420]); try theme(f3, 'light'); catch, end
tl3 = tiledlayout(f3, 1, 3, 'TileSpacing', 'compact'); title(tl3, "Margins at the design delay (bars) and Monte-Carlo fail rate (markers)", 'FontWeight', 'bold');
for a = 1:3
    ax = nexttile(tl3); m = GT.Axis == axn(a); cat = categorical(GT.Case(m), GT.Case(m));
    yyaxis(ax, 'left'); bar(ax, cat, [GT.PM_deg(m) 5*GT.GM_dB(m)], 0.8);
    yline(ax, opts.PMmin, 'r--'); ylabel(ax, 'PM (deg) | 5 x GM (dB)');
    yyaxis(ax, 'right'); plot(ax, cat, GT.MonteCarloFailPct(m), 'ko', 'MarkerSize', 9, 'LineWidth', 1.5); ylabel(ax, 'MC fail %'); ylim(ax, [0 100]);
    grid(ax, 'on'); title(ax, axn(a)); legend(ax, ["PM", "5 x GM"], 'Location', 'northwest');
end
save1(f3, "3_margins.png"); if vis == "off", delete(f3); end

% 4. replay (first 400 Hz flight, 10 s window)
fl = F.flights([F.flights.rate_hz] > 300); fl = fl(1);
f4 = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 1500 700]); try theme(f4, 'light'); catch, end
tl4 = tiledlayout(f4, 2, 1, 'TileSpacing', 'compact'); title(tl4, "Replay of " + fl.id + ": measured rate vs model (" + opts.CurrentLabel + ") and LQR target", 'FontWeight', 'bold', 'Interpreter', 'none');
nfl = size(fl.ratedata.w, 1); i0 = max(1, min(8001, nfl - 4000));
idx = (i0:min(nfl, i0 + 3999))'; t = (idx - idx(1)) / 400;
for a = 1:2
    r = R.(axn(a)); ax = nexttile(tl4); hold(ax, 'on');
    for c = [1 4]
        g = r.current; if c == 4, g = rmfield(r.target, 'q'); end
        y = simFlight(c2d(pade(closedLoops(r.model, g, opts), 3), 1/400), fl, a);
        plot(ax, t, rad2deg(y(idx)), 'Color', col(c, :), 'LineWidth', 1.2);
    end
    plot(ax, t, rad2deg(fl.ratedata.w(idx, a)), 'k:', 'LineWidth', 1); plot(ax, t, rad2deg(fl.ratedata.wd(idx, a)), 'Color', [0.2 0.7 0.3]);
    legend(ax, ["model, " + opts.CurrentLabel, "model, LQR target", "measured", "commanded"], 'Location', 'northeastoutside', 'Interpreter', 'none');
    ylabel(ax, axn(a) + " rate (deg/s)"); grid(ax, 'on');
end
xlabel(ax, 's');
save1(f4, "4_replay.png"); if vis == "off", delete(f4); end

% 5. delay fit
f5 = figure('Visible', vis, 'Color', 'w', 'Position', [40 40 900 420]); try theme(f5, 'light'); catch, end
ax = axes(f5); hold(ax, 'on');
for a = 1:3
    d = R.(axn(a)).delayFit; plot(ax, 1000 * d.tau, d.fit, 'o-', 'LineWidth', 1.3);
end
xlabel(ax, 'assumed delay (ms)'); ylabel(ax, 'replay fit, 1-15 Hz (%)'); grid(ax, 'on'); legend(ax, axn, 'Location', 'best');
title(ax, 'Closed-loop delay estimate: flown tune driven by logged rate commands');
save1(f5, "5_delay_fit.png"); if vis == "off", delete(f5); end
end
