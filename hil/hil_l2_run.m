function out = hil_l2_run(target, opts)
%HIL_L2_RUN Fly the scripted test mission for HIL Level 2.
%
%   PC reference (ArduCopter SITL with ArduPilot's built-in X-quad physics + vehicle.json, same tune):
%       ref = hil_l2_run("sitl");
%   Real board (Your FC flashed with Simulation-on-Hardware firmware, on USB):
%       hil = hil_l2_run("COM8");
%   Then compare:
%       hil_l2_compare(hil.log, ref.log)     % hil.log: the .BIN copied from the board
%
%   The mission is bench\sitl_tests.py: arm in ALT_HOLD, climb, roll/pitch/yaw steps,
%   climb step, LOITER, RTL, land. Mission Planner can watch: SITL on TCP 127.0.0.1:5762;
%   for the board, use a second USB/telemetry port or watch after the run.

arguments
    target (1,1) string
    opts.ParamFile (1,1) string = ""
    opts.Python (1,1) string = fullfile(getenv('LOCALAPPDATA'), "Programs", "Python", "Python311", "python.exe")
    opts.Home (1,1) string = "29.874330,77.901740,300,0"
    opts.Timeout (1,1) double = 600
end
here = fileparts(mfilename('fullpath')); root = fileparts(here);
if ~isfile(opts.Python)
    [st, p] = system('where python'); if st == 0, opts.Python = strtrim(extractBefore(string(p) + newline, newline)); end
end
pilot = fullfile(root, "bench", "sitl_tests.py");
runDir = fullfile(here, "results", "L2_" + lower(regexprep(target, '\W', '')) + "_" + string(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
mkdir(runDir);
out.runDir = runDir; out.log = "";

if lower(target) == "sitl"
    if opts.ParamFile == "", opts.ParamFile = fullfile(here, "l2", "ref_sitl.parm"); end
    if ~isfile(opts.ParamFile), error("Run hil_l2_make_params first (missing %s).", opts.ParamFile); end
    exe = fullfile(root, "sitl", "bin", "ArduCopter.elf");
    copyfile(fullfile(here, "l2", "vehicle.json"), fullfile(runDir, "vehicle.json"));   % SITL reads it from its working folder
    sitlCmd = sprintf('"%s" --model x:vehicle.json --speedup 1 --home %s --defaults "%s" --wipe -I0 > sitl_stdout.txt 2>&1', exe, opts.Home, opts.ParamFile);
    pS = startProc("cmd.exe", "/c """ + sitlCmd + """", runDir);
    c1 = onCleanup(@() killTree(pS.Id));
    fprintf('Started PC reference SITL (built-in X-quad physics + vehicle.json), pid %d. Mission Planner: TCP 127.0.0.1:5762\n', pS.Id);
    pause(2); addr = "tcp:127.0.0.1:5760";
else
    addr = target;
    fprintf(['Flying the mission on the board at %s.\n' ...
        '  Board must run the Simulation-on-Hardware firmware with hil.parm loaded.\n' ...
        '  PROPS OFF. Mission Planner must be DISCONNECTED from this port.\n'], addr);
end

drvLog = fullfile(runDir, "test_pilot.txt");
pyCmd = sprintf('"%s" -u "%s" %s > "%s" 2>&1', opts.Python, pilot, addr, drvLog);
pD = startProc("cmd.exe", "/c """ + pyCmd + """", runDir);
c2 = onCleanup(@() killTree(pD.Id));
pos = 0; t0 = tic;
while ~pD.HasExited && toc(t0) < opts.Timeout
    pause(0.5); pos = echoNew(drvLog, pos);
end
pause(0.5); echoNew(drvLog, pos);
if ~pD.HasExited, warning("Mission timed out after %.0f s.", opts.Timeout); end
txt = string(fileread(drvLog)); out.pilotLog = txt;
out.passed = contains(txt, "TESTS COMPLETE") && contains(txt, "LANDED and disarmed");
fprintf('\nMission %s.\n', ternary(out.passed, "COMPLETE (landed and disarmed)", "DID NOT COMPLETE - see test_pilot.txt"));

if lower(target) == "sitl"
    pause(2); killTree(pS.Id);
    logs = dir(fullfile(runDir, "logs", "*.BIN"));
    if ~isempty(logs), [~, i] = max([logs.datenum]); out.log = string(fullfile(logs(i).folder, logs(i).name)); end
    fprintf('Reference log: %s\n', out.log);
else
    fprintf(['\nNext: copy the newest .BIN from the board (Mission Planner > DataFlash Logs >\n' ...
        'Download, or the SD card APM\\LOGS folder) into\n  %s\nthen run  hil_l2_compare("<that .BIN>", ref.log)\n'], runDir);
end
save(fullfile(runDir, "run_info.mat"), 'out');
end

function pos = echoNew(f, pos)
if ~isfile(f), return; end
fid = fopen(f, 'r'); fseek(fid, pos, 'bof'); t = fread(fid, inf, '*char')'; pos = ftell(fid); fclose(fid);
if ~isempty(t), fprintf('%s', t); end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end

function p = startProc(file, args, wd)
psi = System.Diagnostics.ProcessStartInfo(char(file), char(args));
psi.WorkingDirectory = char(wd); psi.UseShellExecute = false; psi.CreateNoWindow = true;
p = System.Diagnostics.Process.Start(psi);
end

function killTree(pid)
[~, ~] = system(sprintf('taskkill /T /F /PID %d', pid));
end
