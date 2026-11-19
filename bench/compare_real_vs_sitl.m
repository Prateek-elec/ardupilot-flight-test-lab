function R = compare_real_vs_sitl(realLog, sitlLog)
%COMPARE_REAL_VS_SITL Same flight-quality metrics on the real flight and the
%SITL flight (real ArduPilot code on the MATLAB physics), for model validation.

fprintf('\n--- Real flight vs SITL (same metrics) ---\n');
M = struct();
srcs = ["real", "sitl"]; files = [realLog, sitlLog];
for s = 1:2
    lo = ardupilotreader(files(s));
    want = intersect({'ATT','CTUN','RATE','RCOU'}, cellstr(lo.AvailableMessages.MsgName)); want = want(:)';
    T = readMessages(lo, MessageNames=want);
    g = @(n) T.MsgData{find(T.MsgName == n, 1)};
    att = g("ATT"); ct = g("CTUN"); rt = g("RATE");
    ta = seconds(att.Properties.RowTimes); tc = seconds(ct.Properties.RowTimes);
    fly = interp1(tc, double(double(ct.ThO) > 0.25 & double(ct.Alt) > 1.0), ta, 'nearest', 0) > 0.5;
    % downsample SITL to 10 Hz so both use the same sample rate
    if s == 2
        k = false(size(ta)); k(1:40:end) = true; fly = fly & k;
    end
    m.rollRms  = rms(double(att.DesRoll(fly)) - double(att.Roll(fly)));
    m.pitchRms = rms(double(att.DesPitch(fly)) - double(att.Pitch(fly)));
    flyc = double(ct.ThO) > 0.25 & double(ct.Alt) > 1.0;
    m.hoverThr = median(double(ct.ThO(flyc)));
    tr = seconds(rt.Properties.RowTimes);
    flyr = interp1(tc, double(flyc), tr, 'nearest', 0) > 0.5;
    m.rollOutRms = rms(double(rt.ROut(flyr)));
    m.pitchOutRms = rms(double(rt.POut(flyr)));
    m.yawOutMean = mean(abs(double(rt.YOut(flyr))));
    M.(srcs(s)) = m;
end
R = M;
f = @(x) sprintf('%.3f', x);
rows = {"Hover throttle (CTUN.ThO)", f(M.real.hoverThr), f(M.sitl.hoverThr);
        "Roll tracking RMS (deg)",   f(M.real.rollRms),  f(M.sitl.rollRms);
        "Pitch tracking RMS (deg)",  f(M.real.pitchRms), f(M.sitl.pitchRms);
        "Roll controller output RMS", f(M.real.rollOutRms), f(M.sitl.rollOutRms);
        "Pitch controller output RMS", f(M.real.pitchOutRms), f(M.sitl.pitchOutRms);
        "Mean |yaw output|",         f(M.real.yawOutMean), f(M.sitl.yawOutMean)};
fprintf('  %-30s %10s %10s\n', 'Metric', 'REAL', 'SITL');
for i = 1:size(rows,1), fprintf('  %-30s %10s %10s\n', rows{i,:}); end
fprintf(['  Note: the real flight had pilot manoeuvres, wind and 32 m/s^2 vibration on IMU0;\n' ...
         '  SITL flew a scripted sequence with 2.2 m/s wind and clean sensors.\n']);
end
