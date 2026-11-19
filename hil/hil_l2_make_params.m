function files = hil_l2_make_params(opts)
%HIL_L2_MAKE_PARAMS Parameter files for HIL Level 2 (Simulation on Hardware).
%   Writes, into hil\l2\:
%     hil.parm      - load onto the flight controller running the Sim-on-Hardware firmware
%     ref_sitl.parm - the same flight tune for the PC reference run (built-in physics)
%   Both carry the tune from the real flight log plus the recommended Tune C.
%
%   files = hil_l2_make_params();
%   files = hil_l2_make_params(TuneC=false, EKF="sim");   % log tune, ArduPilot's simulated AHRS

arguments
    opts.LogFile (1,1) string = fullfile(fileparts(mfilename('fullpath')), "data", "flight_2026-11-09.bin")
    opts.TuneC (1,1) logical = true
    opts.EKF (1,1) string {mustBeMember(opts.EKF, ["ekf3", "sim"])} = "ekf3"   % ekf3 = real EKF3 runs on the board
    opts.Home (1,4) double = [-35.363262 149.165237 300 0]
end
here = fileparts(mfilename('fullpath'));
outDir = fullfile(here, "l2"); if ~isfolder(outDir), mkdir(outDir); end
if ~isfile(opts.LogFile)
    error("Flight log not found: %s\nPass LogFile=""<your .bin>"" (any real flight log of this drone).", opts.LogFile);
end
addpath(fullfile(fileparts(here), "bench"));

base = fullfile(outDir, "tune_from_log.parm");
sitl_params_from_log(opts.LogFile, base);
tune = fileread(base);
if opts.TuneC
    tune = sprintf('%s# Recommended Tune C\nATC_RAT_YAW_P 0.34\nATC_RAT_YAW_I 0.034\nATC_RAT_PIT_D 0.0034\n', tune);
end

ins = sprintf(['INS_ACCOFFS_X 0.001\nINS_ACCOFFS_Y 0.001\nINS_ACCOFFS_Z 0.001\n' ...
    'INS_ACCSCAL_X 1.001\nINS_ACCSCAL_Y 1.001\nINS_ACCSCAL_Z 1.001\n' ...
    'INS_ACC2OFFS_X 0.001\nINS_ACC2OFFS_Y 0.001\nINS_ACC2OFFS_Z 0.001\n' ...
    'INS_ACC2SCAL_X 1.001\nINS_ACC2SCAL_Y 1.001\nINS_ACC2SCAL_Z 1.001\n']);
ekfType = 3; if opts.EKF == "sim", ekfType = 10; end
% Level 2 flies ArduPilot's built-in multicopter physics loaded with hil\l2\vehicle.json
% (280 g, 220 mm, 2S, hover 0.50 - this drone). The drone's own motor curve stays.
tune = regexprep(tune, '(?m)^SIM_BATT_VOLTAGE[ \t][^\n]*\n', '');
tune = sprintf('%sSIM_BATT_VOLTAGE 8.2\n', tune);

hw = sprintf(['# ---- Simulation on Hardware (ArduPilot Tools/scripts/sitl-on-hardware/default.param) ----\n' ...
    'AHRS_EKF_TYPE %d\nGPS1_TYPE 100\nSIM_MAG1_DEVID 97539\nSIM_RATE_HZ 400\nSCHED_LOOP_RATE 400\n' ...
    'BRD_RTC_TYPES 2\nBRD_SAFETY_DEFLT 0\n%s' ...
    '# Motor outputs stay OFF in simulation. Set SIM_OH_MASK 15 only with ALL PROPS REMOVED.\nSIM_OH_MASK 0\n' ...
    'SIM_OPOS_LAT %.6f\nSIM_OPOS_LNG %.6f\nSIM_OPOS_ALT %.1f\nSIM_OPOS_HDG %.0f\n' ...
    'FRAME_CLASS 1\nFRAME_TYPE 1\n'], ekfType, ins, opts.Home);

files.hil = fullfile(outDir, "hil.parm");
files.ref = fullfile(outDir, "ref_sitl.parm");
writeFile(files.hil, sprintf('# Your FC HIL Level 2 parameters (generated %s)\n%s%s', string(datetime('now')), tune, hw));
writeFile(files.ref, sprintf('# Your FC PC reference SITL parameters (generated %s)\n%s%sFRAME_CLASS 1\nFRAME_TYPE 1\nAHRS_EKF_TYPE %d\n', ...
    string(datetime('now')), tune, ins, ekfType));
fprintf('Wrote\n  %s  (load onto the board)\n  %s  (PC reference)\n', files.hil, files.ref);
end

function writeFile(p, s)
fid = fopen(p, 'w'); fprintf(fid, '%s', s); fclose(fid);
end
