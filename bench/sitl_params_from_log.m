function T = sitl_params_from_log(binFile, outFile)
%SITL_PARAMS_FROM_LOG Write a SITL defaults file with the tuning from a flight log.
%   Copies the parameters that define how the drone flies (attitude/rate/
%   position controllers, motor curve, filters, limits, failsafes) and adds
%   the settings SITL needs (no DShot, no fence, fast logging, battery).

lo = ardupilotreader(binFile);
Tm = readMessages(lo, MessageNames={'PARM'});
P = Tm.MsgData{1};
names = string(P.Name); vals = double(P.Value);
[u, ~, j] = unique(names, 'stable');
last = accumarray(j, (1:numel(names))', [], @max);
names = u; vals = vals(last);

keep = startsWith(names, ["ATC_","PSC_","MOT_","ANGLE_MAX","PILOT_","LOIT_","WPNAV_","RTL_","LAND_", ...
    "FRAME_","ACRO_","INS_GYRO_FILTER","INS_ACCEL_FILTER","INS_HNTCH","INS_HNTC2","FS_EKF","FS_CRASH", ...
    "PHLD_","THR_DZ","GUID_"]);
drop = ismember(names, ["MOT_PWM_TYPE","MOT_PWM_MIN","MOT_PWM_MAX","MOT_SAFE_DISARM","MOT_SAFE_TIME"]);
names = names(keep & ~drop); vals = vals(keep & ~drop);

over = { ...
    "MOT_PWM_TYPE", 0; "MOT_PWM_MIN", 1000; "MOT_PWM_MAX", 2000; ...
    "MOT_HOVER_LEARN", 0; ...                 % keep the learned hover throttle fixed
    "SERVO1_FUNCTION", 33; "SERVO2_FUNCTION", 34; "SERVO3_FUNCTION", 35; "SERVO4_FUNCTION", 36; ...
    "FENCE_ENABLE", 0; ...
    "LOG_BITMASK", 180223; ...                % fast attitude (400 Hz) + PID logging
    "LOG_DISARMED", 0; "LOG_FILE_DSRMROT", 1; ...
    "BATT_MONITOR", 4; "SIM_BATT_VOLTAGE", 7.6; "BATT_CAPACITY", 5000; ...
    "BATT_LOW_VOLT", 0; "BATT_CRT_VOLT", 0; "BATT_ARM_VOLT", 0; ... % 2S: SITL defaults assume more cells
    "BATT_FS_LOW_ACT", 0; "BATT_FS_CRT_ACT", 0; ...
    "FLTMODE_CH", 5; "FLTMODE1", 2; "FLTMODE2", 2; "FLTMODE3", 2; ... % RC switch = ALT_HOLD only;
    "FLTMODE4", 2; "FLTMODE5", 2; "FLTMODE6", 2; ...                  % the test pilot holds CH5 fixed
    "ARMING_CHECK", 1; "RC_OVERRIDE_TIME", 3; ...
    "FS_THR_ENABLE", 0; "FS_GCS_ENABLE", 0};  % scripted test: no RC/GCS failsafes
for i = 1:size(over, 1)
    k = find(names == over{i,1}, 1);
    if isempty(k), names(end+1) = over{i,1}; vals(end+1) = over{i,2}; %#ok<AGROW>
    else, vals(k) = over{i,2}; end
end
fid = fopen(outFile, 'w');
fprintf(fid, '# SITL defaults generated from %s\n', binFile);
for i = 1:numel(names)
    fprintf(fid, '%s %.7g\n', names(i), vals(i));
end
fclose(fid);
T = table(names(:), vals(:), 'VariableNames', ["Name","Value"]);
fprintf('Wrote %d parameters to %s\n', numel(names), outFile);
end
