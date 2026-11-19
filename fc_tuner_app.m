function app = fc_tuner_app(logFiles, opts)
%FC_TUNER_APP  FC PID tuner: pick flight log(s), press Tune, load the .param it gives you.
%
%   In MATLAB, from the fc_test folder:   fc_tuner_app
%
%   Step 1  Add log(s)   - each log is checked and marked usable / not usable (with why)
%   Step 2  Tune         - learns how the drone responds from the logs, designs the PID
%                          with LQR and checks it stays stable with model errors
%   Result               - per axis: current vs new gains, confidence, what changes;
%                          Save .param and load it in Mission Planner
%
%   Back end: bench/fc_tune_from_log.m (usable without the app).
%   app = fc_tuner_app(files, AutoRun=true, Visible="off")   % scripted / test use

arguments
    logFiles string = strings(0)
    opts.AutoRun (1,1) logical = false
    opts.Visible (1,1) string = "on"
end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, "bench"));

LOGS = table(strings(0, 1), strings(0, 1), false(0, 1), strings(0, 1), 'VariableNames', ["Path", "File", "OK", "Status"]);
out = []; lastDir = string(fullfile(getenv('USERPROFILE'), 'Downloads'));
if ~isfolder(lastDir), lastDir = string(pwd); end
C = struct('green', [0.80 0.93 0.80], 'amber', [1.00 0.92 0.70], 'red', [1.00 0.82 0.80], 'grey', [0.90 0.90 0.90], ...
    'ink', [0.10 0.10 0.10], 'accent', [0.18 0.50 0.28]);
plotInfo = struct( ...
    'file', {"2_loops.png", "3_margins.png", "4_replay.png", "1_design_space.png", "5_delay_fit.png"}, ...
    'name', {"Response to a quick stick input", "Safety margins: current vs new", "Your flight replayed through the model", ...
             "How the tune was chosen (technical)", "Delay estimate (technical)"}, ...
    'help', {"Top: loop stability - a line further from the X is more stable. Bottom: how the rotation rate follows a sudden stick input. Less overshoot and ringing is better. Grey = current tune, orange = new tune (step 1), red = final target (often the same).", ...
             "Bars: phase margin and gain margin (higher = safer; dashed line = minimum). Dots: share of 'what-if' drones with +-30 % model error that would be poorly damped (lower = more robust).", ...
             "Dotted = what your drone actually did, green = what was commanded, grey/red = what the model predicts for the current/new tune. Shows how well the model matches your drone.", ...
             "Every candidate tune from the LQR sweep. Star = the one picked: the fastest that still meets the safety margins.", ...
             "How well the model matches the flight for different assumed delays. The peak is the delay used for the design."});

% ================================================================== layout
fig = uifigure('Name', 'FC PID Tuner', 'Position', [60 40 1320 840], 'Visible', opts.Visible);
M = uigridlayout(fig, [1 2]); M.ColumnWidth = {420, '1x'}; M.Padding = [12 12 12 12]; M.ColumnSpacing = 12;

% ---------------- left column
Lc = uigridlayout(M, [6 1]); Lc.RowHeight = {34, 22, '1x', 'fit', 30, 0}; Lc.Padding = [0 0 0 0]; Lc.RowSpacing = 8;
uilabel(Lc, 'Text', 'FC PID Tuner', 'FontSize', 22, 'FontWeight', 'bold');
uilabel(Lc, 'Text', 'Add your flight log, press Tune, load the result in Mission Planner.', 'FontColor', [0.5 0.5 0.5]);

p1 = uipanel(Lc, 'Title', 'Step 1 - Choose flight log(s)', 'FontWeight', 'bold', 'FontSize', 13);
g1 = uigridlayout(p1, [3 3]); g1.RowHeight = {'1x', 32, 'fit'}; g1.ColumnWidth = {'1x', '1x', '1x'};
tblLogs = uitable(g1, 'ColumnName', {'Log', 'Status'}, 'ColumnWidth', {150, 'auto'}, 'RowName', {}, 'FontSize', 12, ...
    'SelectionType', 'row', 'SelectionChangedFcn', @(~, ~) showLogStatus());
tblLogs.Layout.Column = [1 3];
uibutton(g1, 'Text', 'Add log...', 'FontWeight', 'bold', 'ButtonPushedFcn', @(~, ~) addLogs());
uibutton(g1, 'Text', 'Remove selected', 'ButtonPushedFcn', @(~, ~) removeSel());
uibutton(g1, 'Text', 'Clear', 'ButtonPushedFcn', @(~, ~) setLogs(LOGS([], :)));
HINT = 'Logs need 400 Hz rate logging (LOG_BITMASK = 180223) and at least 15 s of flight. More stick movement gives a better result. Click a log to see its full status.';
hint = uilabel(g1, 'WordWrap', 'on', 'FontColor', [0.5 0.5 0.5], 'FontSize', 11, 'Text', HINT);
hint.Layout.Column = [1 3];

p2 = uipanel(Lc, 'Title', 'Step 2 - Tune', 'FontWeight', 'bold', 'FontSize', 13);
g2 = uigridlayout(p2, [4 1]); g2.RowHeight = {46, 24, 26, 0}; g2.RowSpacing = 6;
bRun = uibutton(g2, 'Text', 'Tune', 'FontWeight', 'bold', 'FontSize', 17, 'BackgroundColor', C.accent, ...
    'FontColor', 'w', 'Enable', 'off', 'ButtonPushedFcn', @(~, ~) runTune(true));
cbSITL = uicheckbox(g2, 'Text', 'Also test the new tune in the simulator (adds ~6 min)', 'Value', false);
bAdv = uibutton(g2, 'Text', 'Show advanced settings', 'ButtonPushedFcn', @(~, ~) toggleAdv());
gA = uigridlayout(g2, [5 2]); gA.ColumnWidth = {170, '1x'}; gA.RowHeight = repmat({26}, 1, 5); gA.Padding = [0 0 0 0];
uilabel(gA, 'Text', 'Design delay', 'Tooltip', 'Delay between controller output and motor response');
ddDelay = uidropdown(gA, 'Items', ["From the flight (recommended)", "Worst case (phase-based)", "Fixed 6 ms"], ...
    'ItemsData', ["replay", "identified", "component"], 'Value', "replay");
uilabel(gA, 'Text', 'Min phase margin (deg)');
spPM = uispinner(gA, 'Value', 50, 'Limits', [30 75], 'Step', 5);
uilabel(gA, 'Text', 'Min gain margin (dB)');
spGM = uispinner(gA, 'Value', 6, 'Limits', [3 15], 'Step', 1);
uilabel(gA, 'Text', 'Max poorly-damped what-ifs (%)', 'Tooltip', 'Monte Carlo: share of drones with +-30 % model error allowed to be poorly damped');
spMC = uispinner(gA, 'Value', 10, 'Limits', [0 60], 'Step', 5);
uilabel(gA, 'Text', 'Max change per flight (%)', 'Tooltip', 'Step 1 may change a gain by at most this much, up or down');
spInc = uispinner(gA, 'Value', 30, 'Limits', [10 60], 'Step', 5);

status = uilabel(Lc, 'Text', 'Add a log to start.', 'FontAngle', 'italic', 'WordWrap', 'on');

% ---------------- right column
TG = uitabgroup(M);
tRes = uitab(TG, 'Title', 'Result');
R = uigridlayout(tRes, [5 1]); R.RowHeight = {'fit', 'fit', '1x', 'fit', 40}; R.RowSpacing = 10; R.Padding = [12 12 12 12];
banner = uilabel(R, 'Text', '  Add a flight log on the left and press Tune.', 'FontSize', 15, 'FontWeight', 'bold', ...
    'BackgroundColor', C.grey, 'FontColor', C.ink, 'WordWrap', 'on');
warnLbl = uilabel(R, 'Text', '', 'WordWrap', 'on', 'FontSize', 12);
gc = uigridlayout(R, [1 3]); gc.ColumnSpacing = 10; gc.Padding = [0 0 0 0];
cards = [makeCard(gc, "Roll"), makeCard(gc, "Pitch"), makeCard(gc, "Yaw")];
nextLbl = uilabel(R, 'WordWrap', 'on', 'FontSize', 12, 'Text', welcomeText());
gb = uigridlayout(R, [1 5]); gb.ColumnWidth = {'1x', 190, 180, 170, 170}; gb.Padding = [0 0 0 0];
uilabel(gb, 'Text', '');
bRep = uibutton(gb, 'Text', 'Open PDF report', 'FontWeight', 'bold', 'Enable', 'off', 'BackgroundColor', C.accent, ...
    'FontColor', 'w', 'ButtonPushedFcn', @(~, ~) openReport());
bSave = uibutton(gb, 'Text', 'Save .param file...', 'FontWeight', 'bold', 'Enable', 'off', 'ButtonPushedFcn', @(~, ~) saveParam());
bCopy = uibutton(gb, 'Text', 'Copy parameters', 'Enable', 'off', 'ButtonPushedFcn', @(~, ~) copyParam());
bOpen = uibutton(gb, 'Text', 'Open results folder', 'Enable', 'off', 'ButtonPushedFcn', @(~, ~) openFolder());

tPlot = uitab(TG, 'Title', 'Graphs');
gP = uigridlayout(tPlot, [3 1]); gP.RowHeight = {30, 'fit', '1x'};
ddPlot = uidropdown(gP, 'Items', "(press Tune first)", 'ValueChangedFcn', @(~, ~) showPlot());
plotHelp = uilabel(gP, 'Text', '', 'WordWrap', 'on', 'FontColor', [0.45 0.45 0.45]);
img = uiimage(gP, 'ScaleMethod', 'fit');

tSitl = uitab(TG, 'Title', 'Simulator test');
gT = uigridlayout(tSitl, [2 1]); gT.RowHeight = {'fit', '1x'};
sitlLbl = uilabel(gT, 'WordWrap', 'on', 'Text', ['Tick "Also test the new tune in the simulator" before pressing Tune. The current and the new tune ' ...
    'are then flown on the real ArduCopter firmware (SITL) with physics matched to your drone, in the same wind and vibration.']);
tblSitl = uitable(gT, 'FontSize', 12, 'RowName', {});

tDet = uitab(TG, 'Title', 'Technical details');
gD = uigridlayout(tDet, [2 1]); gD.RowHeight = {'1x', '1x'};
txtSum = uitextarea(gD, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 11);
txtLog = uitextarea(gD, 'Editable', 'off', 'FontName', 'Consolas', 'FontSize', 10);

tHelp = uitab(TG, 'Title', 'Help');
gH = uigridlayout(tHelp, [1 1]);
uitextarea(gH, 'Editable', 'off', 'FontSize', 13, 'Value', helpText());

LOGS = addFiles(LOGS, logFiles(:)', false);
setLogs(LOGS);
app = struct('Figure', fig, 'run', @() runTune(false), 'result', @getResult);
if opts.AutoRun && any(LOGS.OK), runTune(false); end

% ================================================================== callbacks
    function r = getResult()
        r = out;
    end

    function addLogs()
        [f, p] = uigetfile({'*.bin;*.BIN', 'ArduPilot flight logs (*.bin)'}, 'Select flight log(s)', char(lastDir), 'MultiSelect', 'on');
        figure(fig);
        if isequal(f, 0), return; end
        lastDir = string(p);
        setLogs(addFiles(LOGS, string(fullfile(p, f)), true));
    end

    function T = addFiles(T, files, interactive)
        files = files(~ismember(files, T.Path));
        if isempty(files), return; end
        dlg = [];
        if interactive, dlg = uiprogressdlg(fig, 'Title', 'Checking logs', 'Indeterminate', 'on', 'Message', 'Reading log headers...'); end
        for f = files
            [~, n, e] = fileparts(f);
            if ~isempty(dlg), dlg.Message = "Checking " + n + e; drawnow; end
            [ok, note] = fc_check_log(f);
            T = [T; {f, n + e, ok, note}]; %#ok<AGROW>
        end
        if ~isempty(dlg), close(dlg); end
    end

    function setLogs(T)
        LOGS = T;
        tblLogs.Data = table(LOGS.File, LOGS.Status, 'VariableNames', ["Log", "Status"]);
        removeStyle(tblLogs);
        if ~isempty(LOGS)
            addStyle(tblLogs, uistyle('BackgroundColor', C.green, 'FontColor', C.ink), 'row', find(LOGS.OK));
            addStyle(tblLogs, uistyle('BackgroundColor', C.red, 'FontColor', C.ink), 'row', find(~LOGS.OK));
        end
        n = nnz(LOGS.OK);
        if isempty(LOGS)
            status.Text = 'Add a log to start.'; bRun.Enable = 'off';
        elseif n == 0
            status.Text = 'None of these logs can be used - see the Status column.'; bRun.Enable = 'off';
        else
            status.Text = sprintf('%d usable log(s). Press Tune.', n); bRun.Enable = 'on';
        end
    end

    function showLogStatus()
        s = tblLogs.Selection;
        if isempty(s) || isempty(LOGS), hint.Text = HINT; return; end
        k = s(1, 1);
        if k <= height(LOGS), hint.Text = LOGS.File(k) + ":  " + LOGS.Status(k); end
    end

    function removeSel()
        s = tblLogs.Selection;
        if isempty(s), return; end
        if size(s, 2) == 2, s = unique(s(:, 1)); end
        T = LOGS; T(s, :) = []; setLogs(T);
    end

    function toggleAdv()
        if g2.RowHeight{4} == 0
            g2.RowHeight{4} = 5 * 26 + 4 * 10; bAdv.Text = 'Hide advanced settings';
        else
            g2.RowHeight{4} = 0; bAdv.Text = 'Show advanced settings';
        end
    end

    function runTune(interactive)
        files = LOGS.Path(LOGS.OK)';
        if isempty(files), return; end
        bRun.Enable = 'off'; status.Text = 'Working...';
        dlg = [];
        if interactive, dlg = uiprogressdlg(fig, 'Title', 'Tuning', 'Message', 'Reading the logs...', 'Value', 0); end
        prog = @(f, m) progress(dlg, f, m);
        args = {'Delay', ddDelay.Value, 'PMmin', spPM.Value, 'GMmin', spGM.Value, 'MCmaxFail', spMC.Value, ...
            'MaxIncrease', 1 + spInc.Value / 100, 'MaxDecrease', 1 - spInc.Value / 100, 'SITL', cbSITL.Value, 'Progress', prog};
        res = []; %#ok<NASGU>   (nested workspace is static: declare before evalc assigns it)
        try
            txt = evalc('res = fc_tune_from_log(files, args{:});');
            out = res;
            txtLog.Value = splitlines(string(txt));
            % per-log result back into the list
            T = LOGS;
            for i = 1:height(out.logInfo)
                k = find(T.File == out.logInfo.File(i), 1);
                if ~isempty(k), T.OK(k) = out.logInfo.Used(i); T.Status(k) = out.logInfo.Note(i); end
            end
            setLogs(T);
            showResults();
            status.Text = 'Done. See the Result tab.';
        catch e
            txtLog.Value = splitlines(string(getReport(e, 'basic')));
            status.Text = 'Could not tune - see the message.';
            showError(e.message);
            if interactive
                uialert(fig, e.message, 'Could not tune from these logs');
            else
                fprintf(2, 'fc_tuner_app: %s\n', e.message);
            end
        end
        if ~isempty(dlg) && isvalid(dlg), close(dlg); end
        bRun.Enable = 'on';
    end

    function progress(dlg, f, m)
        if ~isempty(dlg) && isvalid(dlg), dlg.Value = min(max(f, 0), 1); dlg.Message = char(m); end
        status.Text = char(m); drawnow limitrate;
    end

    function showError(msg)
        banner.Text = "  Could not tune: " + string(msg);
        banner.BackgroundColor = C.red;
        warnLbl.Text = '';
        for k = 1:3, clearCard(cards(k)); end
        bSave.Enable = 'off'; bCopy.Enable = 'off'; bOpen.Enable = 'off'; bRep.Enable = 'off';
        TG.SelectedTab = tRes;
    end

    function showResults()
        switch out.verdict
            case "ok",       banner.Text = "  Ready for a test hover. The new tune is more stable on all axes and the data is good."; banner.BackgroundColor = C.green;
            case "ok-check", banner.Text = "  Ready for a careful test hover. Some axes have lower confidence - see below."; banner.BackgroundColor = C.amber;
            otherwise,       banner.Text = "  Usable, but test very gently: roll or pitch has LOW confidence. More flight data would help."; banner.BackgroundColor = C.amber;
        end
        if isempty(out.warnings), warnLbl.Text = '';
        else, warnLbl.Text = strjoin("•  " + out.warnings, newline); end
        for k = 1:3, fillCard(cards(k), out.axes(k)); end
        nextLbl.Text = nextSteps();
        txtSum.Value = splitlines(out.summary);
        % graphs
        have = isfile(fullfile(out.outDir, [plotInfo.file]));
        if any(have)
            P = plotInfo(have);
            ddPlot.Items = [P.name]; ddPlot.ItemsData = string(fullfile(out.outDir, [P.file]));
            ddPlot.Value = ddPlot.ItemsData(1); showPlot();
        end
        % simulator
        if ~isempty(out.sitl) && height(out.sitl) > 0
            S = out.sitl;
            tblSitl.Data = table(S.Tune, round(S.AttTrackRMS_deg, 2), round(S.StepOvershoot_pct, 1), round(S.YawTrackRMS_degps, 1), ...
                round(S.MotorNoiseRoll_pct, 2), round(S.MotorNoisePitch_pct, 2), 'VariableNames', ...
                ["Tune", "Attitude error (deg)", "Overshoot (%)", "Yaw error (deg/s)", "Roll motor noise (%)", "Pitch motor noise (%)"]);
            sitlLbl.Text = ['Both tunes flown on the real ArduCopter firmware (SITL) with physics matched to your drone. ' ...
                'Lower is better in every column. A softer tune shows a little more attitude error but less motor noise.'];
        else
            tblSitl.Data = table();
        end
        bSave.Enable = 'on'; bCopy.Enable = 'on'; bOpen.Enable = 'on';
        if isfield(out, 'reportFile') && isfile(out.reportFile), bRep.Enable = 'on'; else, bRep.Enable = 'off'; end
        TG.SelectedTab = tRes;
    end

    function openReport()
        if ispc, winopen(out.reportFile); else, system("open """ + out.reportFile + """"); end
    end

    function s = nextSteps()
        s = sprintf(['Next steps\n' ...
            '1.  In Mission Planner: CONFIG > Full Parameter List > Save to file (backup of your current settings).\n' ...
            '2.  Click "Save .param file..." here, then in Mission Planner: Load from file > Write Params.\n' ...
            '     (The full write-up with graphs is in "Open PDF report".)\n' ...
            '3.  Hover low (1-2 m) in STABILIZE or ALT_HOLD for 1-2 minutes, with some gentle stick inputs. Land.\n' ...
            '4.  Add the new log here and press Tune again. Repeat until the values stop changing.']);
    end

    function showPlot()
        v = string(ddPlot.Value);
        if ~isfile(v), return; end
        img.ImageSource = char(v);
        [~, n, e] = fileparts(v);
        k = find([plotInfo.file] == n + e, 1);
        if ~isempty(k), plotHelp.Text = plotInfo(k).help; end
    end

    function saveParam()
        [f, p] = uiputfile('*.param', 'Save parameter file', char(fullfile(lastDir, "fc_lqr_tune.param")));
        figure(fig);
        if isequal(f, 0), return; end
        copyfile(out.paramFile, fullfile(p, f));
        status.Text = "Saved " + string(fullfile(p, f));
    end

    function copyParam()
        clipboard('copy', fileread(out.paramFile));
        status.Text = 'Parameters copied to the clipboard.';
    end

    function openFolder()
        if ispc, winopen(out.outDir); else, system("open """ + out.outDir + """"); end
    end

    % ---------------- axis cards
    function c = makeCard(parent, name)
        c.panel = uipanel(parent, 'BorderType', 'line');
        g = uigridlayout(c.panel, [6 1]); g.RowHeight = {30, 'fit', 'fit', 'fit', 'fit', '1x'}; g.RowSpacing = 6;
        h = uigridlayout(g, [1 2]); h.ColumnWidth = {'1x', 150}; h.Padding = [0 0 0 0];
        uilabel(h, 'Text', name, 'FontSize', 18, 'FontWeight', 'bold');
        c.badge = uilabel(h, 'Text', '', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontColor', C.ink);
        t = uigridlayout(g, [3 4]); t.ColumnWidth = {55, '1x', '1x', '1x'}; t.RowHeight = {20, 22, 22}; t.Padding = [0 0 0 0]; t.RowSpacing = 2;
        hdr = ["", "P", "I", "D"];
        for j = 1:4, uilabel(t, 'Text', hdr(j), 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'FontColor', [0.5 0.5 0.5]); end
        uilabel(t, 'Text', 'Now', 'FontColor', [0.5 0.5 0.5]);
        for j = 1:3, c.cur(j) = uilabel(t, 'Text', '-', 'HorizontalAlignment', 'center'); end
        uilabel(t, 'Text', 'New', 'FontWeight', 'bold');
        for j = 1:3, c.new(j) = uilabel(t, 'Text', '-', 'HorizontalAlignment', 'center', 'FontWeight', 'bold', 'FontSize', 14); end
        c.margin = uilabel(g, 'Text', '', 'WordWrap', 'on', 'FontWeight', 'bold');
        c.text = uilabel(g, 'Text', '', 'WordWrap', 'on');
        c.final = uilabel(g, 'Text', '', 'WordWrap', 'on', 'FontColor', [0.5 0.5 0.5], 'FontSize', 11);
        uilabel(g, 'Text', '');
    end

    function clearCard(c)
        c.badge.Text = ''; c.badge.BackgroundColor = 'none';
        for j = 1:3, c.cur(j).Text = '-'; c.new(j).Text = '-'; end
        c.margin.Text = ''; c.text.Text = ''; c.final.Text = '';
    end

    function fillCard(c, A)
        fmt = @(v, d) sprintf(['%.' num2str(d) 'f'], v);
        cv = [A.Cur.P A.Cur.I A.Cur.D]; nv = [A.New.P A.New.I A.New.D]; dg = [3 3 4];
        for j = 1:3
            c.cur(j).Text = fmt(cv(j), dg(j));
            c.new(j).Text = fmt(nv(j), dg(j));
            if abs(nv(j) - cv(j)) < 1e-6, c.new(j).FontColor = [0.5 0.5 0.5]; elseif nv(j) > cv(j), c.new(j).FontColor = [0.15 0.45 0.80]; else, c.new(j).FontColor = [0.85 0.40 0.05]; end
        end
        switch A.Confidence
            case "High",   c.badge.Text = 'High confidence';   c.badge.BackgroundColor = C.green;
            case "Medium", c.badge.Text = 'Medium confidence'; c.badge.BackgroundColor = C.amber;
            case "Low",    c.badge.Text = 'Low confidence';    c.badge.BackgroundColor = C.red;
            otherwise,     c.badge.Text = 'Unchanged';         c.badge.BackgroundColor = C.grey;
        end
        if A.Kept
            c.margin.Text = 'Not tuned';
        else
            c.margin.Text = sprintf('Stability margin: %.0f%s (%s)  ->  %.0f%s (%s)', A.PM(1), char(176), pmWord(A.PM(1)), A.PM(2), char(176), pmWord(A.PM(2)));
        end
        c.text.Text = A.Text;
        if ~A.Kept && ~A.IsFinal
            c.final.Text = sprintf('This is step 1 (max 30 %% change per flight). Final target: P %.3f  I %.3f  D %.4f', A.Final.P, A.Final.I, A.Final.D);
        elseif ~A.Kept
            c.final.Text = sprintf('Angle P: %.2f', A.New.AngP);
        else
            c.final.Text = '';
        end
    end
end

%% ----------------------------------------------------------------------
function w = pmWord(pm)
if pm < 30, w = 'low'; elseif pm < 45, w = 'marginal'; else, w = 'good'; end
end

function s = welcomeText()
s = sprintf(['How it works\n' ...
    '1.  Add one or more flight logs (.bin) on the left. Each one is checked: green = usable, red = not usable (and why).\n' ...
    '2.  Press Tune. The tool learns how your drone responds from the logs, designs new PID gains with LQR and checks that\n' ...
    '     they stay stable even if the model is off by 30 %%. It takes 2-3 minutes.\n' ...
    '3.  The result shows the new gains for roll, pitch and yaw, how confident it is, and what will change.']);
end

function s = helpText()
s = [
"FC PID TUNER"
""
"WHAT IT DOES"
"  1. Reads your flight logs and the PID values that were flying (from the log itself)."
"  2. Learns how the drone responds on roll, pitch and yaw (how hard it accelerates, how fast the motors react, how much delay)."
"  3. Generates candidate gains with LQR (an optimal-control method) plus a direct search around your current"
"     tune. Keeps only gains that are well damped on the model (phase margin >= 50 deg, gain margin >= 6 dB,"
"     peak sensitivity <= 1.8) and stay safe if the model is off by 30 %. Of those, it picks the one that"
"     holds attitude best against wind and follows your stick best (set mainly by P and I)."
"  4. Gives you a .param file to load in Mission Planner. Gains change by at most 30 % per flight."
""
"WHICH LOGS WORK"
"  - Logs recorded with 400 Hz rate logging: set LOG_BITMASK = 180223 before flying."
"  - At least 15 seconds in the air. Gentle stick movements on all axes give the best result."
"  - You can add several logs from the same drone and tune; they are combined."
"  - Red rows in the log list cannot be used; the Status column says why."
""
"READING THE RESULT"
"  - Confidence: how well the learned model matches your flight. High = trust it. Medium = safe on paper,"
"    check how it feels. Low = test very gently and add more flight data."
"  - Stability margin (phase margin): how far the drone is from oscillating. Below 30 deg = low,"
"    30-45 deg = marginal, 45 deg or more = good."
"  - New values in blue went up, orange went down."
"  - 'Step 1': no gain changes by more than 30 % (up or down) per flight. Fly, then tune again."
""
"SAFETY"
"  - Always back up your parameters before loading new ones."
"  - First flight with new gains: low hover, open area, ready to land or switch back."
"  - The tool cannot see mechanical problems (loose motors or props, frame flex, vibration). Fix those first."
""
"ADVANCED SETTINGS"
"  Design delay, minimum margins, allowed what-if failures and maximum gain increase per flight."
"  The defaults are a good choice; stricter values give a softer, safer tune."
];
end
