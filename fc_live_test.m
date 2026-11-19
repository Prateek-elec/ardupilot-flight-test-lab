function results = fc_live_test(opts)
%FC_LIVE_TEST Live bench test of an ArduPilot flight controller over MAVLink.
%
%   Checks sensors, power, GPS, RC link, EKF health and pre-arm messages,
%   then (optionally) walks you through tilt / rotate / stick tests.
%
%   BEFORE RUNNING
%     1. Connect the FC to Mission Planner as usual (USB or telemetry).
%     2. In Mission Planner press Ctrl+F -> "Mavlink Mirror" ->
%        "UDP Client", host 127.0.0.1, port 14560 (answer "No" to write access).
%     3. Put the drone on a flat surface, PROPS OFF, radio ON and bound.
%
%   USAGE
%     results = fc_live_test();                     % full test with prompts
%     results = fc_live_test(Interactive=false);    % still test only
%     results = fc_live_test(Port=14560, StillSeconds=15);
%
%   Try it without hardware:
%     stopFake = fc_fake_drone();  results = fc_live_test(Interactive=false);  stopFake();

arguments
    opts.Port (1,1) double = 14560
    opts.StillSeconds (1,1) double {mustBePositive} = 10
    opts.Interactive (1,1) logical = true
    opts.Plot (1,1) logical = true
    opts.HeartbeatTimeout (1,1) double {mustBePositive} = 15
end

dialect = mavlinkdialect("ardupilotmega.xml");
mav = mavlinkio(dialect);
connect(mav, "UDP", LocalPort=opts.Port);
cleanupObj = onCleanup(@() disconnect(mav)); %#ok<NASGU>

R = table('Size', [0 4], 'VariableTypes', ["string","string","string","string"], ...
          'VariableNames', ["Check","Measured","Limit","Result"]);

%% 1) Wait for the autopilot heartbeat
fprintf('\nWaiting for the flight controller on UDP port %d ...\n', opts.Port);
hbSub = mavlinksub(mav, "HEARTBEAT", BufferSize=50);
hb = [];
t0 = tic;
while toc(t0) < opts.HeartbeatTimeout
    m = latestmsgs(hbSub, 50);
    m = m(arrayfun(@(x) double(x.Payload.autopilot) == 3, m));   % 3 = ArduPilot
    if ~isempty(m), hb = m(1); break; end
    pause(0.25);
end
delete(hbSub);
if isempty(hb)
    error('fc_live_test:noHeartbeat', ['No ArduPilot heartbeat on UDP port %d.\n' ...
        'In Mission Planner: Ctrl+F -> Mavlink Mirror -> UDP Client, 127.0.0.1 : %d.'], opts.Port, opts.Port);
end
fprintf('Connected: system ID %d, vehicle type %d.\n', hb.SystemID, double(hb.Payload.type));
R = addRow(R, "Heartbeat", "received", "ArduPilot", "PASS");

%% 2) Still test
fprintf('\nSTILL TEST: keep the drone completely still for %g s ...\n', opts.StillSeconds);
topics = ["ATTITUDE","RAW_IMU","SCALED_IMU2","SCALED_PRESSURE","GPS_RAW_INT","SYS_STATUS", ...
          "POWER_STATUS","RC_CHANNELS","VIBRATION","EKF_STATUS_REPORT","STATUSTEXT"];
D = collect(mav, topics, opts.StillSeconds);

% Attitude
if isempty(D.ATTITUDE)
    R = addRow(R, "ATTITUDE stream", "none", "> 0 msgs", "FAIL");
else
    roll  = rad2deg(fv(D.ATTITUDE, "roll"));
    pitch = rad2deg(fv(D.ATTITUDE, "pitch"));
    R = addRow(R, "Level (|roll|,|pitch|)", sprintf('%.1f, %.1f deg', abs(mean(roll)), abs(mean(pitch))), ...
               "< 3 deg (warn < 6)", grade(max(abs([mean(roll) mean(pitch)])), 3, 6));
    R = addRow(R, "Attitude jitter (std)", sprintf('%.3f deg', max(std(roll), std(pitch))), ...
               "< 0.2 deg (warn < 0.5)", grade(max(std(roll), std(pitch)), 0.2, 0.5));
end

% IMU (ArduPilot RAW_IMU: accel mg, gyro mrad/s, mag mGauss)
if isempty(D.RAW_IMU)
    R = addRow(R, "RAW_IMU stream", "none", "> 0 msgs", "SKIP");
else
    acc = [fv(D.RAW_IMU,"xacc") fv(D.RAW_IMU,"yacc") fv(D.RAW_IMU,"zacc")];
    gyr = [fv(D.RAW_IMU,"xgyro") fv(D.RAW_IMU,"ygyro") fv(D.RAW_IMU,"zgyro")];
    mag = [fv(D.RAW_IMU,"xmag") fv(D.RAW_IMU,"ymag") fv(D.RAW_IMU,"zmag")];
    g = mean(vecnorm(acc, 2, 2));
    R = addRow(R, "Accel |g|", sprintf('%.0f mg', g), "1000 +/- 50 mg", grade(abs(g - 1000), 50, 100));
    R = addRow(R, "Accel noise (max std)", sprintf('%.1f mg', max(std(acc))), "< 20 mg (warn < 50)", grade(max(std(acc)), 20, 50));
    R = addRow(R, "Gyro bias (max |mean|)", sprintf('%.1f mrad/s', max(abs(mean(gyr)))), "< 10 (warn < 30)", grade(max(abs(mean(gyr))), 10, 30));
    R = addRow(R, "Gyro noise (max std)", sprintf('%.2f mrad/s', max(std(gyr))), "< 5 (warn < 15)", grade(max(std(gyr)), 5, 15));
    B = mean(vecnorm(mag, 2, 2));
    R = addRow(R, "Compass |field|", sprintf('%.0f mGauss', B), "150 - 850 mGauss", ...
               ternary(B >= 150 && B <= 850, "PASS", "FAIL"));
    R = addRow(R, "Compass noise (max std)", sprintf('%.1f mGauss', max(std(mag))), "< 10 (warn < 25)", grade(max(std(mag)), 10, 25));
end

% Second IMU (test board: IMU1 BMI088, IMU2 BMI270)
if isempty(D.SCALED_IMU2)
    R = addRow(R, "IMU 2 stream", "none", "2 IMUs expected", "WARN");
else
    acc2 = [fv(D.SCALED_IMU2,"xacc") fv(D.SCALED_IMU2,"yacc") fv(D.SCALED_IMU2,"zacc")];
    gyr2 = [fv(D.SCALED_IMU2,"xgyro") fv(D.SCALED_IMU2,"ygyro") fv(D.SCALED_IMU2,"zgyro")];
    g2 = mean(vecnorm(acc2, 2, 2));
    R = addRow(R, "IMU 2 accel |g|", sprintf('%.0f mg', g2), "1000 +/- 50 mg", grade(abs(g2 - 1000), 50, 100));
    R = addRow(R, "IMU 2 gyro noise (max std)", sprintf('%.2f mrad/s', max(std(gyr2))), "< 5 (warn < 15)", grade(max(std(gyr2)), 5, 15));
    if ~isempty(D.RAW_IMU)
        d = norm(mean(acc2) - mean(acc));
        R = addRow(R, "IMU 1 vs IMU 2 accel", sprintf('%.0f mg apart', d), "< 50 mg (warn < 100)", grade(d, 50, 100));
    end
end

% 5 V rail (board 5 V BEC)
if ~isempty(D.POWER_STATUS)
    vcc = mean(fv(D.POWER_STATUS, "Vcc")) / 1000;
    if vcc < 0.5    % board has no 5 V sensing (reports 0)
        R = addRow(R, "Board 5V rail (Vcc)", "not measured by this board", "-", "SKIP");
    else
        R = addRow(R, "Board 5V rail (Vcc)", sprintf('%.2f V', vcc), "4.8 - 5.4 V", ...
                   ternary(vcc >= 4.8 && vcc <= 5.4, "PASS", ternary(vcc >= 4.6 && vcc <= 5.6, "WARN", "FAIL")));
    end
end

% Barometer
if isempty(D.SCALED_PRESSURE)
    R = addRow(R, "Baro stream", "none", "> 0 msgs", "SKIP");
else
    altNoise = std(fv(D.SCALED_PRESSURE, "press_abs")) * 8.3;   % ~8.3 m per hPa
    R = addRow(R, "Baro altitude noise", sprintf('%.2f m', altNoise), "< 0.3 m (warn < 1)", grade(altNoise, 0.3, 1));
end

% Vibration (motors off -> should be tiny)
if ~isempty(D.VIBRATION)
    vib = max([fv(D.VIBRATION,"vibration_x"); fv(D.VIBRATION,"vibration_y"); fv(D.VIBRATION,"vibration_z")]);
    clip = [fv(D.VIBRATION,"clipping_0"); fv(D.VIBRATION,"clipping_1"); fv(D.VIBRATION,"clipping_2")];
    R = addRow(R, "Vibration (still)", sprintf('%.1f m/s^2', vib), "< 15 (warn < 30)", grade(vib, 15, 30));
    R = addRow(R, "Accel clipping", sprintf('%d', max(clip) - min(clip)), "0 new clips", ternary(max(clip) == min(clip), "PASS", "FAIL"));
end

% Power, CPU load, sensor health
if ~isempty(D.SYS_STATUS)
    s = D.SYS_STATUS(1);
    V = double(s.Payload.voltage_battery) / 1000;
    I = double(s.Payload.current_battery) / 100;
    load = double(s.Payload.load) / 10;
    R = addRow(R, "Battery voltage", sprintf('%.2f V', V), "> 0 (check vs multimeter)", ternary(V > 1, "PASS", "FAIL"));
    if I >= 0
        R = addRow(R, "Current while disarmed", sprintf('%.2f A', I), "< 1 A (warn < 3)", grade(I, 1, 3));
    else
        R = addRow(R, "Current while disarmed", "not measured", "-", "SKIP");
    end
    R = addRow(R, "CPU load", sprintf('%.0f %%', load), "< 70 % (warn < 85)", grade(load, 70, 85));

    present = uint32(s.Payload.onboard_control_sensors_present);
    enabled = uint32(s.Payload.onboard_control_sensors_enabled);
    health  = uint32(s.Payload.onboard_control_sensors_health);
    bad = strings(0);
    for b = sensorBits()'
        bit = uint32(b{2});
        if bitand(present, bit) && bitand(enabled, bit) && ~bitand(health, bit)
            bad(end+1) = b{1}; %#ok<AGROW>
        end
    end
    R = addRow(R, "Sensor health", ternary(isempty(bad), "all healthy", strjoin(bad, ", ")), ...
               "no unhealthy sensors", ternary(isempty(bad), "PASS", "FAIL"));
end

% GPS
if ~isempty(D.GPS_RAW_INT)
    gp = D.GPS_RAW_INT(1).Payload;
    fix = double(gp.fix_type); sats = double(gp.satellites_visible); hdop = double(gp.eph) / 100;
    R = addRow(R, "GPS fix", sprintf('type %d', fix), ">= 3 (3D)", ternary(fix >= 3, "PASS", "FAIL"));
    R = addRow(R, "GPS satellites", sprintf('%d', sats), ">= 10 (warn >= 6)", gradeHigh(sats, 10, 6));
    if fix >= 3
        R = addRow(R, "GPS HDOP", sprintf('%.2f', hdop), "< 1.5 (warn < 2.5)", grade(hdop, 1.5, 2.5));
    end
end

% RC link
if isempty(D.RC_CHANNELS)
    R = addRow(R, "RC channels", "none", "receiver connected", "FAIL");
else
    rc = D.RC_CHANNELS(1).Payload;
    n = double(rc.chancount); rssi = double(rc.rssi);
    R = addRow(R, "RC channel count", sprintf('%d', n), ">= 8", gradeHigh(n, 8, 4));
    if rssi ~= 255
        R = addRow(R, "RC RSSI / link", sprintf('%.0f %%', rssi / 2.54), "> 70 % (warn > 40)", gradeHigh(rssi / 2.54, 70, 40));
    end
end

% EKF
if ~isempty(D.EKF_STATUS_REPORT)
    e = D.EKF_STATUS_REPORT(1).Payload;
    ev = [double(e.velocity_variance) double(e.pos_horiz_variance) double(e.pos_vert_variance) double(e.compass_variance)];
    R = addRow(R, "EKF variances (vel,posH,posV,mag)", sprintf('%.2f %.2f %.2f %.2f', ev), "< 0.5 (warn < 0.8)", grade(max(ev), 0.5, 0.8));
end

% Pre-arm / status messages seen
texts = statusTexts(D.STATUSTEXT);
if ~isempty(texts)
    fprintf('\nMessages from the FC during the test:\n');
    fprintf('  %s\n', texts);
    pre = texts(contains(texts, "PreArm", IgnoreCase=true));
    R = addRow(R, "PreArm messages", sprintf('%d', numel(pre)), "0", ternary(isempty(pre), "PASS", "FAIL"));
end

if opts.Plot && ~isempty(D.ATTITUDE) && ~isempty(D.RAW_IMU)
    plotStill(D);
end

%% 3) Interactive movement tests
if opts.Interactive && ~isempty(D.ATTITUDE)
    fprintf('\nMOVEMENT TESTS (props OFF). Follow each prompt, hold the position, then press Enter.\n');

    a = attNow(mav, "Hold the drone LEVEL and still");
    yaw0 = a(3);
    a = attNow(mav, "Tilt the NOSE UP about 30-45 deg and hold");
    R = addRow(R, "Pitch direction (nose up)", sprintf('%+.0f deg', a(2)), "> +15 deg", ternary(a(2) > 15, "PASS", "FAIL"));
    a = attNow(mav, "Level again, then roll the RIGHT side DOWN about 30-45 deg");
    R = addRow(R, "Roll direction (right down)", sprintf('%+.0f deg', a(1)), "> +15 deg", ternary(a(1) > 15, "PASS", "FAIL"));
    a = attNow(mav, "Put it level, then turn it 90 deg CLOCKWISE (seen from above)");
    dyaw = wrap180(a(3) - yaw0);
    R = addRow(R, "Yaw / compass (90 deg CW)", sprintf('%+.0f deg', dyaw), "+90 +/- 25 deg", ternary(abs(dyaw - 90) <= 25, "PASS", "FAIL"));

    c = rcNow(mav, "Push the ROLL stick (right stick) fully RIGHT");
    R = addRow(R, "RC roll -> CH1", sprintf('%d us', c(1)), "> 1800 us", ternary(c(1) > 1800, "PASS", "FAIL"));
    c = rcNow(mav, "Push the PITCH stick (right stick) fully UP / forward");
    % ArduPilot convention: pitch stick FORWARD (nose down) = LOW PWM on CH2.
    % If it reads high, set RC2_REVERSED = 1 (or reverse CH2 in the radio).
    R = addRow(R, "RC pitch fwd -> CH2 low", sprintf('%d us', c(2)), "< 1200 us (else RC2_REVERSED=1)", ternary(c(2) < 1200, "PASS", "FAIL"));
    c = rcNow(mav, "Push the THROTTLE stick (left stick) fully UP");
    R = addRow(R, "RC throttle -> CH3", sprintf('%d us', c(3)), "> 1800 us", ternary(c(3) > 1800, "PASS", "FAIL"));
    c = rcNow(mav, "Throttle DOWN, push the YAW stick (left stick) fully RIGHT");
    R = addRow(R, "RC yaw -> CH4", sprintf('%d us', c(4)), "> 1800 us", ternary(c(4) > 1800, "PASS", "FAIL"));
end

%% 4) Report
fprintf('\n================ FC LIVE TEST REPORT ================\n');
disp(R);
nFail = sum(R.Result == "FAIL"); nWarn = sum(R.Result == "WARN");
fprintf('PASS %d   WARN %d   FAIL %d   SKIP %d\n', sum(R.Result == "PASS"), nWarn, nFail, sum(R.Result == "SKIP"));
if nFail == 0 && nWarn == 0
    fprintf('Result: ALL CHECKS PASSED\n');
elseif nFail == 0
    fprintf('Result: OK with warnings - review the WARN rows\n');
else
    fprintf('Result: FIX THE FAIL ROWS BEFORE FLYING\n');
end
results = R;
end

%% ---------------- helpers ----------------
function D = collect(mav, topics, seconds)
subs = cell(1, numel(topics));
for k = 1:numel(topics)
    subs{k} = mavlinksub(mav, topics(k), BufferSize=5000);
end
pause(seconds);
D = struct();
for k = 1:numel(topics)
    D.(topics(k)) = latestmsgs(subs{k}, 5000);
    delete(subs{k});
end
end

function v = fv(msgs, name)
v = arrayfun(@(m) double(m.Payload.(name)), msgs);
v = v(:);
end

function a = attNow(mav, prompt)
input(sprintf('  -> %s, then press Enter and KEEP HOLDING for 2 s ', prompt), 's');
D = collect(mav, "ATTITUDE", 2);
if isempty(D.ATTITUDE), a = [NaN NaN NaN]; return; end
a = [rad2deg(mean(fv(D.ATTITUDE,"roll"))), rad2deg(mean(fv(D.ATTITUDE,"pitch"))), ...
     circMeanDeg(rad2deg(fv(D.ATTITUDE,"yaw")))];
end

function c = rcNow(mav, prompt)
input(sprintf('  -> %s, then press Enter and KEEP HOLDING for 2 s ', prompt), 's');
D = collect(mav, "RC_CHANNELS", 1.5);
if isempty(D.RC_CHANNELS), c = [0 0 0 0]; return; end
c = round([mean(fv(D.RC_CHANNELS,"chan1_raw")) mean(fv(D.RC_CHANNELS,"chan2_raw")) ...
           mean(fv(D.RC_CHANNELS,"chan3_raw")) mean(fv(D.RC_CHANNELS,"chan4_raw"))]);
end

function m = circMeanDeg(d)
m = atan2d(mean(sind(d)), mean(cosd(d)));
end

function d = wrap180(d)
d = mod(d + 180, 360) - 180;
end

function t = statusTexts(msgs)
t = strings(0);
for k = numel(msgs):-1:1
    raw = msgs(k).Payload.text;
    raw = raw(raw ~= 0);
    t(end+1) = string(char(raw(:)')); %#ok<AGROW>
end
t = unique(t, 'stable');
end

function bits = sensorBits()
bits = {"Gyro", 1; "Accel", 2; "Compass", 4; "Barometer", 8; "GPS", 32; ...
        "RC receiver", 65536; "Gyro 2", 131072; "Accel 2", 262144; "Compass 2", 524288; ...
        "Geofence", 1048576; "AHRS/EKF", 2097152; "Logging", 16777216; "Battery", 33554432; ...
        "Pre-arm checks", 268435456};
end

function r = grade(val, passBelow, warnBelow)
if val < passBelow, r = "PASS"; elseif val < warnBelow, r = "WARN"; else, r = "FAIL"; end
end

function r = gradeHigh(val, passAtLeast, warnAtLeast)
if val >= passAtLeast, r = "PASS"; elseif val >= warnAtLeast, r = "WARN"; else, r = "FAIL"; end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end

function R = addRow(R, check, measured, limit, result)
R(end+1, :) = {string(check), string(measured), string(limit), string(result)};
end

function plotStill(D)
figure('Name', 'FC still test', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact');
nexttile; plot([rad2deg(fv(D.ATTITUDE,"roll")) rad2deg(fv(D.ATTITUDE,"pitch"))]);
title('Attitude while still'); ylabel('deg'); legend('roll','pitch'); grid on
nexttile; plot([fv(D.RAW_IMU,"xacc") fv(D.RAW_IMU,"yacc") fv(D.RAW_IMU,"zacc")]);
title('Accelerometer'); ylabel('mg'); legend('x','y','z'); grid on
nexttile; plot([fv(D.RAW_IMU,"xgyro") fv(D.RAW_IMU,"ygyro") fv(D.RAW_IMU,"zgyro")]);
title('Gyro'); ylabel('mrad/s'); xlabel('sample'); legend('x','y','z'); grid on
end
