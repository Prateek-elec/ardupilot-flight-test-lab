function f = lab_default_log()
%LAB_DEFAULT_LOG  The newest ArduPilot DataFlash log in this lab's logs/ folder.
%
%   Used as the default by run_sim, run_analysis, run_sitl_bench and fc_benchmark,
%   so they work with your own flight without editing any file names.
%   Put .BIN logs in <lab>/logs/ (copy them from the SD card, or download them in
%   Mission Planner: DATA -> DataFlash Logs -> Download DataFlash Log Via Mavlink).

logDir = fullfile(fileparts(mfilename('fullpath')), "logs");
d = [dir(fullfile(logDir, "*.bin")); dir(fullfile(logDir, "*.BIN"))];
[~, keep] = unique(lower(string({d.name})));   % Windows lists *.bin and *.BIN twice
d = d(keep);
if isempty(d)
    error('lab:noLog', ['No flight logs found in %s\n' ...
        'Copy one or more ArduPilot .BIN logs there, or pass a file name explicitly.'], logDir);
end
[~, i] = max([d.datenum]);
f = string(fullfile(d(i).folder, d(i).name));
end
