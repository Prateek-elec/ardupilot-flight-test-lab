function out = run_sitl_bench(opts)
%RUN_SITL_BENCH Real ArduCopter firmware (SITL) flying the MATLAB 6-DOF model.
%
%   1. Builds a SITL defaults file from your flight log (your tune).
%   2. Starts ArduCopter SITL with --model JSON (physics = this MATLAB session).
%   3. Starts the scripted test pilot (sitl_tests.py, MAVLink on tcp:5760).
%   4. Runs the physics bridge in lockstep, printing live status.
%   5. Stops SITL and returns the SITL DataFlash log + the physics log.
%
%   Watch live in Mission Planner: connect TCP, 127.0.0.1, port 5762.
%
%   out = run_sitl_bench();
%   out = run_sitl_bench(Wind=[3 0 0], Gusts=1.5);

arguments
    opts.LogFile (1,1) string = "logs\2026-11-09 16-30-17.bin"
    opts.Wind (1,3) double = [2 1 0]       % steady wind NED, m/s (blowing toward N/E)
    opts.Gusts (1,1) double = 1.0          % turbulence RMS, m/s
    opts.Home (1,1) string = "-35.363262,149.165237,300,0"
    opts.Params = quad_params()
    opts.Python (1,1) string = fullfile(getenv('LOCALAPPDATA'), "Programs", "Python", "Python311", "python.exe")
    opts.ParamOverrides cell = {}          % {"ATC_RAT_YAW_P", 0.34; ...} applied on top of the log tune
    opts.Label (1,1) string = "baseline"
    opts.VibAccel (1,3) double = [0 0 0]   % RMS m/s^2 at hover
    opts.VibGyro (1,3) double = [0 0 0]    % RMS rad/s at hover
    opts.GyroNoise (1,1) double = deg2rad(0.3)
    opts.Seed (1,1) double = 3             % same turbulence for every run
    opts.Analyze (1,1) logical = true
    opts.MotorDelay (1,1) double = 0       % s of extra motor-command delay in the physics (robustness)
end
if ~isfile(opts.Python), error('Python not found at %s - pass Python="C:\\path\\python.exe"', opts.Python); end

here = fileparts(mfilename('fullpath'));
sitlDir = fullfile(fileparts(here), "sitl");
exe = fullfile(sitlDir, "bin", "ArduCopter.elf");
runDir = fullfile(sitlDir, "run_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')) + "_" + opts.Label);
mkdir(runDir);
parmFile = fullfile(runDir, "fc_from_log.parm");
sitl_params_from_log(opts.LogFile, parmFile);
if ~isempty(opts.ParamOverrides)
    fid = fopen(parmFile, 'a');
    fprintf(fid, '# --- overrides for run "%s"\n', opts.Label);
    for i = 1:size(opts.ParamOverrides, 1)
        fprintf(fid, '%s %.7g\n', opts.ParamOverrides{i,1}, opts.ParamOverrides{i,2});
        fprintf('  override %s = %.7g\n', opts.ParamOverrides{i,1}, opts.ParamOverrides{i,2});
    end
    fclose(fid);
end
fprintf('=== SITL run "%s"\n', opts.Label);

% physics model
m = quad_physics(opts.Params);
m.Vref = 8.2;                          % thrust curve defined at MOT_BAT_VOLT_MAX
m.wind = opts.Wind(:);
tg = 0:0.05:3600; rng(opts.Seed);
gw = opts.Gusts * filter(0.05, [1 -0.95], randn(numel(tg), 3)) / sqrt(0.05^2/(1-0.95^2));
m.gust = @(t) gw(min(floor(t/0.05) + 1, numel(tg)), :)';   % 20 Hz turbulence samples

% start SITL (stdout to file) and the test pilot
% physics at SITL's default 1200 Hz: ArduPilot needs gyro rate >= 1.8 x 400 Hz loop
sitlCmd = sprintf('"%s" --model JSON:127.0.0.1 --speedup 1 --home %s --defaults "%s" --wipe -I0 > sitl_stdout.txt 2>&1', ...
    exe, opts.Home, parmFile);
pS = startProc("cmd.exe", "/c """ + sitlCmd + """", runDir);
fprintf('Started ArduCopter SITL (pid %d) in %s\n', pS.Id, runDir);
drvLog = fullfile(runDir, "test_pilot.txt");
pyCmd = sprintf('"%s" -u "%s" tcp:127.0.0.1:5760 > "%s" 2>&1', opts.Python, fullfile(here, "sitl_tests.py"), drvLog);
pause(2);
pD = startProc("cmd.exe", "/c """ + pyCmd + """", runDir);
fprintf('Started test pilot (pid %d). Mission Planner: TCP 127.0.0.1:5762 to watch.\n\n', pD.Id);

state.pos = 0;
stopFcn = @() pollPilot();
c1 = onCleanup(@() killTree(pS.Id));
c2 = onCleanup(@() killTree(pD.Id));
L = sitl_bridge(m, StopFcn=stopFcn, BattVolt=7.6, MaxSimTime=400, ...
    VibAccel=opts.VibAccel, VibGyro=opts.VibGyro, GyroNoise=opts.GyroNoise, MotorDelay=opts.MotorDelay);
pollPilot();
pause(1); killTree(pS.Id);

logs = dir(fullfile(runDir, "logs", "*.BIN"));
[~, i] = sort([logs.datenum]);
out.runDir = runDir; out.physics = L; out.sitlLogs = string(fullfile({logs(i).folder}, {logs(i).name}));
out.pilotLog = string(fileread(drvLog));
fprintf('\nSITL run finished. Logs:\n'); fprintf('  %s\n', out.sitlLogs);
save(fullfile(runDir, "physics_log.mat"), 'L');
out.label = opts.Label;
if opts.Analyze && ~isempty(out.sitlLogs)
    out.analysis = sitl_analysis(out.sitlLogs(end), L);
end

    function done = pollPilot()
        % echo new lines from the test pilot; stop when it exits
        if isfile(drvLog)
            fid = fopen(drvLog, 'r'); fseek(fid, state.pos, 'bof');
            txt = fread(fid, inf, '*char')'; state.pos = ftell(fid); fclose(fid);
            if ~isempty(txt), fprintf('%s', txt); end
        end
        done = pD.HasExited;
    end
end

function p = startProc(file, args, wd)
psi = System.Diagnostics.ProcessStartInfo(char(file), char(args));
psi.WorkingDirectory = char(wd); psi.UseShellExecute = false; psi.CreateNoWindow = true;
p = System.Diagnostics.Process.Start(psi);
end

function killTree(pid)
[~, ~] = system(sprintf('taskkill /T /F /PID %d', pid));
end
