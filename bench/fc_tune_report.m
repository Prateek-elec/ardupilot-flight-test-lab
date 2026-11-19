function file = fc_tune_report(out, file)
%FC_TUNE_REPORT PDF tuning report from the result of fc_tune_from_log (MATLAB Report Generator).
%   file = fc_tune_report(out)                 % -> <out.outDir>/fc_tuning_report.pdf
%   file = fc_tune_report(out, "C:\x\report.pdf")
import mlreportgen.dom.*
import mlreportgen.report.*
if nargin < 2, file = fullfile(out.outDir, "fc_tuning_report.pdf"); end
rpt = Report(char(file), 'pdf');
rpt.Layout.Landscape = false;
open(rpt);

GREEN = '#D5EED5'; AMBER = '#FFEBB3'; RED = '#FFD1CC'; GREY = '#EEEEEE'; INK = '#1A1A1A';
axn = ["roll", "pitch", "yaw"]; nm = ["RLL", "PIT", "YAW"];

% ------------------------------------------------------------- title page
tp = TitlePage;
tp.Title = 'FC PID Tuning Report';
tp.Subtitle = 'Rate-controller gains from flight-log system identification, LQR and robust PID search';
tp.Author = 'FC PID Tuner (fc_tuner_app)';
tp.PubDate = char(string(datetime('now', 'Format', 'd MMMM yyyy, HH:mm')));
append(rpt, tp);

% ------------------------------------------------------------- 1. summary
ch = Chapter('Title', 'Summary');
switch out.verdict
    case "ok",       vt = 'Ready for a test hover. The new tune is more stable on all axes and the data is good.'; vc = GREEN;
    case "ok-check", vt = 'Ready for a careful test hover. Some axes have lower confidence (see below).'; vc = AMBER;
    otherwise,       vt = 'Usable, but test very gently: roll or pitch has low confidence. More flight data would help.'; vc = AMBER;
end
p = Paragraph(vt); p.Style = {BackgroundColor(vc), Color(INK), Bold(true), FontSize('13pt'), OuterMargin('0pt', '0pt', '4pt', '10pt'), InnerMargin('8pt')};
append(ch, p);
append(ch, para(sprintf('Based on %d flight segment(s), %.0f s of 400 Hz flight data from: %s.', out.flights, out.airborne_s, ...
    strjoin(out.logInfo.File(out.logInfo.Used), ", "))));

% per-axis results
hdr = {'Axis', 'Confidence', 'Stability margin (now -> new)', 'Overshoot (now -> new)', 'Response time (now -> new)', 'What changes'};
body = cell(3, numel(hdr));
for a = 1:3
    A = out.axes(a);
    body(a, :) = {cap(axn(a)), char(A.Confidence), ...
        ifelse(A.Kept, 'not tuned', sprintf('%.0f%s (%s) -> %.0f%s (%s)', A.PM(1), char(176), pmWord(A.PM(1)), A.PM(2), char(176), pmWord(A.PM(2)))), ...
        ifelse(A.Kept, '-', sprintf('%.0f %% -> %.0f %%', A.Overshoot)), ...
        ifelse(A.Kept, '-', sprintf('%.0f -> %.0f ms', A.Rise_ms)), char(A.Text)};
end
t = mkTable(hdr, body, [0.6 1.1 1.5 1.1 1.1 2.3]);
for a = 1:3
    c = out.axes(a).Confidence;
    col = GREY; if c == "High", col = GREEN; elseif c == "Medium", col = AMBER; elseif c == "Low", col = RED; end
    e = entry(t.Body, a, 2); e.Style = [e.Style {BackgroundColor(col), Color(INK)}];
end
append(ch, heading('Result per axis'));
append(ch, t);

% parameters
append(ch, heading('Parameters to load'));
append(ch, para(sprintf('File: %s', out.paramFile)));
hdr = {'Parameter', 'Current', 'New (step 1)', 'Change'};
body = {};
for a = 1:3
    A = out.axes(a);
    vals = {'P', A.Cur.P, A.New.P, 3; 'I', A.Cur.I, A.New.I, 3; 'D', A.Cur.D, A.New.D, 4};
    for k = 1:3
        d = vals{k, 4};
        chg = '-';
        if abs(vals{k, 3} - vals{k, 2}) > 1e-9
            if vals{k, 2} > 0, chg = sprintf('%+.0f %%', 100 * (vals{k, 3} / vals{k, 2} - 1)); else, chg = 'new'; end
        end
        body(end+1, :) = {sprintf('ATC_RAT_%s_%s', nm(a), vals{k, 1}), sprintf(['%.' num2str(d) 'f'], vals{k, 2}), ...
            sprintf(['%.' num2str(d) 'f'], vals{k, 3}), chg}; %#ok<AGROW>
    end
    body(end+1, :) = {sprintf('ATC_ANG_%s_P', nm(a)), sprintf('%.2f', A.Cur.AngP), sprintf('%.2f', A.New.AngP), ...
        ifelse(abs(A.New.AngP - A.Cur.AngP) < 1e-9, '-', sprintf('%+.0f %%', 100 * (A.New.AngP / A.Cur.AngP - 1)))}; %#ok<AGROW>
end
append(ch, mkTable(hdr, body, [2 1 1 1]));
fin = "";
for a = 1:3
    A = out.axes(a);
    if ~A.Kept && ~A.IsFinal
        fin = fin + sprintf(" %s: P %.3f  I %.3f  D %.4f.", cap(axn(a)), A.Final.P, A.Final.I, A.Final.D);
    end
end
if fin ~= ""
    append(ch, para("No gain changes by more than 30 % per flight, so this is step 1. Final targets after a good test flight:" + fin));
end

if ~isempty(out.warnings)
    append(ch, heading('Check before flying'));
    append(ch, UnorderedList(cellstr(out.warnings)));
end

append(ch, heading('Next steps'));
append(ch, OrderedList({ ...
    'In Mission Planner: CONFIG > Full Parameter List > Save to file (backup of the current settings).', ...
    'Load from file > select the .param file above > Write Params.', ...
    'Hover low (1-2 m) in STABILIZE or ALT_HOLD for 1-2 minutes in an open area, with some gentle stick inputs. Land.', ...
    'Run the tuner again on the new log. Repeat until the recommended values stop changing.', ...
    'Fix mechanical issues first (loose motors or props, frame flex, vibration): the tuner cannot see them.'}));
append(rpt, ch);

% ------------------------------------------------------------- 2. data
ch = Chapter('Title', 'Flight data used');
append(ch, para('Each selected log and whether it could be used:'));
L = out.logInfo;
body = [cellstr(L.File), cellstr(ifelse2(L.Used, "yes", "no")), num2cell(L.Flights), cellstr(compose("%.0f s", L.Airborne_s)), cellstr(L.Note)];
t = mkTable({'Log', 'Used', 'Flights', 'Air time', 'Note'}, body, [1.6 0.5 0.6 0.7 3.2]);
for i = 1:height(L)
    e = entry(t.Body, i, 2); e.Style = [e.Style {BackgroundColor(ifelse(L.Used(i), GREEN, RED)), Color(INK)}];
end
append(ch, t);

append(ch, heading('How well the drone could be identified'));
append(ch, para(['The drone''s response on each axis is learned as  K e^(-tau s) / (s (tm s + 1)):  K = control authority ' ...
    '(angular acceleration per unit of controller output), tm = motor/ESC lag, tau = delay. Gain fit and coherence measure ' ...
    'how clean the identification is; replay fit is how much of the measured rotation rate the model reproduces when it ' ...
    'is driven by the commands you flew.']));
Q = out.quality;
body = cell(height(Q), 9);
for a = 1:height(Q)
    body(a, :) = {cap(Q.Axis(a)), char(Q.Confidence(a)), sprintf('%.0f', Q.K_radps2(a)), sprintf('%.1f ms', Q.MotorLag_ms(a)), ...
        sprintf('%.0f %%', Q.GainFit_pct(a)), sprintf('%.2f', Q.Coherence(a)), sprintf('%.0f %%', Q.ReplayFit_pct(a)), ...
        sprintf('%.1f-%.1f Hz', Q.BandLo_Hz(a), Q.BandHi_Hz(a)), char(Q.Grade(a))};
end
append(ch, mkTable({'Axis', 'Confidence', 'K (rad/s^2)', 'Motor lag', 'Gain fit', 'Coherence', 'Replay fit', 'Valid band', 'Data'}, body, ...
    [0.6 0.9 0.8 0.8 0.7 0.8 0.8 1.0 0.7]));
append(rpt, ch);

% ------------------------------------------------------------- 3. graphs
info = { ...
    "2_loops.png", "Response to a quick stick input", "Top: open-loop Nichols chart; a curve further from the X at (-180 deg, 0 dB) is further from oscillating. Bottom: rotation rate after a sudden rate command. Grey = current tune, orange = new tune (step 1), red = final target (often the same as step 1). Less overshoot and ringing is better."; ...
    "3_margins.png", "Safety margins: current vs new", "Bars: phase margin and 5 x gain margin (higher = safer; dashed line = required minimum). Dots: share of 'what-if' drones with +-30 % model error and +-50 % delay error that would be poorly damped (lower = more robust)."; ...
    "4_replay.png", "Your flight replayed through the model", "Dotted: measured rotation rate. Green: commanded. Grey/red: what the model predicts for the current/new tune. Shows how well the model matches the real drone."; ...
    "1_design_space.png", "How the tune was chosen", "Every candidate gain set (LQR sweep and direct search), placed by crossover frequency and phase margin. Coloured = checked by Monte Carlo. Star = picked: the best disturbance rejection that meets every requirement."; ...
    "5_delay_fit.png", "Delay estimate", "Match between model and flight for different assumed delays. The design uses the best match (never below 6 ms)."};
ch = Chapter('Title', 'Graphs');
for k = 1:size(info, 1)
    f = fullfile(out.outDir, info{k, 1});
    if ~isfile(f), continue; end
    append(ch, heading(info{k, 2}));
    im = Image(char(f)); im.Style = {ScaleToFit(true), Width('6.5in')};
    append(ch, im);
    append(ch, para(info{k, 3}, 9));
end
append(rpt, ch);

% ------------------------------------------------------------- 4. simulator
if ~isempty(out.sitl) && height(out.sitl) > 0
    ch = Chapter('Title', 'Simulator test');
    append(ch, para(['Both tunes were flown on the real ArduCopter firmware (SITL) with the physics set to the identified ' ...
        'drone, in the same wind, turbulence and vibration. Lower is better in every column.']));
    S = out.sitl;
    body = [cellstr(S.Tune), cellstr(compose("%.2f", S.AttTrackRMS_deg)), cellstr(compose("%.1f", S.StepOvershoot_pct)), ...
        cellstr(compose("%.1f", S.YawTrackRMS_degps)), cellstr(compose("%.2f", S.MotorNoiseRoll_pct)), cellstr(compose("%.2f", S.MotorNoisePitch_pct)), ...
        cellstr(compose("%.3f", S.AltHoldRMS_m)), cellstr(compose("%.2f", S.LoiterRMS_m))];
    append(ch, mkTable({'Tune', 'Attitude error (deg)', 'Overshoot (%)', 'Yaw error (deg/s)', 'Roll motor noise (%)', ...
        'Pitch motor noise (%)', 'Alt hold (m)', 'Loiter (m)'}, body, ones(1, 8)));
    append(rpt, ch);
end

% ------------------------------------------------------------- 5. method + technical
ch = Chapter('Title', 'Method and technical details');
append(ch, para(['1. System identification. From the 400 Hz RATE log, the frequency response from rate-controller output ' ...
    'to body rate is estimated with the commanded rate as excitation (closed-loop indirect method), and fitted with ' ...
    'K e^(-tau s) / (s (tm s + 1)). The delay is the value that best reproduces the measured rate when the flown tune ' ...
    'is driven by the logged rate commands (1-15 Hz). A joint refit of K, tm and tau to the flight is also tried and ' ...
    'used only if it explains the flight clearly better (at least +3 percentage points); its fit on a held-out flight ' ...
    'is reported.']));
append(ch, para(['2. Candidates. LQR (states: integral of rate error, body rate, motor state; the LQR state feedback is ' ...
    'exactly a PID: P = k_w, I = -k_z, D = k_m / K) with swept weights, plus a direct P/I/D grid around the tune that ' ...
    'flew (0.4-2x P and I, 0.5-1.5x D).']));
append(ch, para(['3. Requirements, on the delayed plant with ArduPilot''s D and target filters: phase margin >= 50 deg, ' ...
    'gain margin >= 6 dB, peak sensitivity Ms <= 1.8, crossover inside the identified band, I between 0.8x and 2x P ' ...
    '(roll/pitch), and at most 10 % of 200 Monte-Carlo plants (K +-30 %, motor lag +-30 %, delay +-50 %) poorly ' ...
    'damped (phase margin < 35 deg or gain margin < 4 dB).']));
append(ch, para(['4. Objective. Of the candidates that meet every requirement, the one with the best disturbance rejection ' ...
    'and command following is picked: lowest mean |S(jw)| over 0.5-5 Hz, where wind, CG offset and stick inputs act. ' ...
    'This is set mainly by P and I (crossover frequency is not used: on this airframe it is set mostly by D, so ' ...
    'ranking by it favours cutting P and I).']));
append(ch, para(['5. Step 1. The move from the current tune to the target is made along a geometric path, so no gain ' ...
    'changes by more than 30 % (up or down) per flight. Step 1 must be at least as stable as the current tune. ' ...
    'The angle-loop P is only lowered when the rate loop is too slow for it, never raised.']));
G = out.gains(ismember(out.gains.Case, ["current (from log)", "LQR step 1", "LQR target"]), :);
body = [cellstr(G.Axis), cellstr(G.Case), cellstr(compose("%.3f", G.P)), cellstr(compose("%.3f", G.I)), cellstr(compose("%.4f", G.D)), ...
    cellstr(compose("%.1f", G.Crossover_Hz)), cellstr(compose("%.1f", G.PM_deg)), cellstr(compose("%.1f", G.GM_dB)), ...
    cellstr(compose("%.0f", G.MonteCarloFailPct)), cellstr(compose("%.0f", 1000 * G.Rise_s)), cellstr(compose("%.0f", G.Overshoot_pct))];
append(ch, heading('Rate-loop analysis (design delay)'));
append(ch, mkTable({'Axis', 'Tune', 'P', 'I', 'D', 'Crossover (Hz)', 'PM (deg)', 'GM (dB)', 'MC fail (%)', 'Rise (ms)', 'Overshoot (%)'}, ...
    body, [0.6 1.3 0.6 0.6 0.7 0.8 0.7 0.7 0.7 0.7 0.8], 8));
append(ch, para(sprintf('Filters from the log: D filter %.1f Hz, target filter %.1f Hz, gyro filter %g Hz.', ...
    out.current.FLTD, out.current.FLTT, out.current.GyroFilter), 9));
append(rpt, ch);

close(rpt);
end

%% ----------------------------------------------------------------------
function p = para(txt, sz)
import mlreportgen.dom.*
if nargin < 2, sz = 10.5; end
p = Paragraph(char(txt)); p.Style = {FontSize(sprintf('%gpt', sz)), OuterMargin('0pt', '0pt', '2pt', '6pt')};
end

function h = heading(txt)
import mlreportgen.dom.*
h = Heading2(char(txt));
end

function t = mkTable(hdr, body, w, sz)
import mlreportgen.dom.*
if nargin < 4, sz = 9; end
t = FormalTable(hdr, body);
t.Style = {Border('solid', '#BBBBBB', '0.5pt'), ColSep('solid', '#DDDDDD', '0.5pt'), RowSep('solid', '#DDDDDD', '0.5pt'), ...
    Width('100%'), FontSize(sprintf('%gpt', sz)), ResizeToFitContents(false), OuterMargin('0pt', '0pt', '4pt', '10pt')};
t.TableEntriesStyle = {InnerMargin('3pt', '3pt', '2pt', '2pt')};
t.Header.Style = {BackgroundColor('#2E7D46'), Color('white'), Bold(true)};
grps = TableColSpecGroup; specs = TableColSpec.empty;
for k = 1:numel(w)
    s = TableColSpec; s.Style = {Width(sprintf('%.0f%%', 100 * w(k) / sum(w)))}; specs(k) = s;
end
grps.ColSpecs = specs; t.ColSpecGroups = grps;
end

function s = cap(x)
x = string(x); s = char(upper(extractBefore(x, 2)) + extractAfter(x, 1));
end

function w = pmWord(pm)
if pm < 30, w = 'low'; elseif pm < 45, w = 'marginal'; else, w = 'good'; end
end

function v = ifelse(c, a, b)
if c, v = a; else, v = b; end
end

function v = ifelse2(c, a, b)
v = repmat(string(b), size(c)); v(c) = a;
end
