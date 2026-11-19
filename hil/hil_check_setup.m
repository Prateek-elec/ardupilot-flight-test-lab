function ok = hil_check_setup(opts)
%HIL_CHECK_SETUP Check a PC is ready for the flight controller HIL tests. Run this first on a new PC.
%   ok = hil_check_setup();
%   ok = hil_check_setup(LogFile="D:\logs\2026-11-09 16-30-17.bin");

arguments
    opts.LogFile (1,1) string = fullfile(fileparts(mfilename('fullpath')), "data", "flight_2026-11-09.bin")
end
here = fileparts(mfilename('fullpath')); root = fileparts(here);
rows = strings(0, 3);
chk = @(name, pass, detail) [name, ternary(pass, "OK", "MISSING"), detail];

v = ver('MATLAB'); rows(end+1, :) = chk("MATLAB", true, v.Release);
tb = ver; names = string({tb.Name});
for t = ["UAV Toolbox", "Aerospace Blockset", "Control System Toolbox"]
    rows(end+1, :) = chk(t, any(names == t), ternary(any(names == t), "installed", "needed for log reading / validation")); %#ok<AGROW>
end
py = fullfile(getenv('LOCALAPPDATA'), "Programs", "Python", "Python311", "python.exe");
if ~isfile(py)
    [st, p] = system('where python'); py = ""; if st == 0, py = strtrim(extractBefore(string(p) + newline, newline)); end
end
rows(end+1, :) = chk("Python 3", py ~= "" && isfile(py), py);
if py ~= "" && isfile(py)
    [st, o] = system(sprintf('"%s" -c "import pymavlink, serial; print(pymavlink.__version__ if hasattr(pymavlink, ''__version__'') else ''ok'')"', py));
    rows(end+1, :) = chk("pymavlink + pyserial", st == 0, strtrim(string(o)));
end
exe = fullfile(root, "sitl", "bin", "ArduCopter.elf");
rows(end+1, :) = chk("ArduCopter SITL (Windows build)", isfile(exe), exe);
rows(end+1, :) = chk("Flight log for the tune", isfile(opts.LogFile), opts.LogFile);
for f = ["hil_l1_io_test.py", "hil_params.py", "l2\build_sim_on_hw.sh", "l2\vehicle.json", fullfile("..", "bench", "sitl_tests.py"), fullfile("..", "bench", "sitl_analysis.m")]
    rows(end+1, :) = chk(f, isfile(fullfile(here, f)), "");  %#ok<AGROW>
end
[st, o] = system('wsl.exe -l -q');
o = strrep(string(o), char(0), "");
hasUbuntu = st == 0 && contains(o, "Ubuntu");
rows(end+1, :) = chk("WSL Ubuntu (Level 2 build only)", hasUbuntu, ternary(hasUbuntu, strtrim(o), "install: wsl --install -d Ubuntu-24.04 (admin, reboot)"));
try
    ports = serialportlist("all");
    rows(end+1, :) = chk("Serial ports", true, strjoin(ports, ", "));
catch
    [~, o] = system('powershell -NoProfile -Command "[System.IO.Ports.SerialPort]::GetPortNames() -join '', ''"');
    rows(end+1, :) = chk("Serial ports", true, strtrim(string(o)));
end
T = array2table(rows, 'VariableNames', ["Item", "Status", "Detail"]);
disp(T);
need = ~contains(T.Item, "WSL");
ok = all(T.Status(need) == "OK");
fprintf('%s\n', ternary(ok, "Ready for HIL Level 1 and the Level 2 PC reference.", "Fix the MISSING items above first."));
if ~hasUbuntu, fprintf('Level 2 firmware build needs WSL Ubuntu (see the runbook, section 4).\n'); end
end

function s = ternary(c, a, b)
if c, s = string(a); else, s = string(b); end
end
