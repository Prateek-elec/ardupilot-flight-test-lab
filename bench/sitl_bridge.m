function L = sitl_bridge(m, opts)
%SITL_BRIDGE MATLAB physics server for ArduPilot SITL (--model JSON).
%
%   Protocol (ArduPilot libraries/SITL/SIM_JSON.cpp):
%     SITL -> MATLAB  UDP :9002, binary little-endian
%                     uint16 magic (18458 = 16 ch, 29569 = 32 ch), uint16 frame_rate,
%                     uint32 frame_count, uint16 pwm[16|32]
%     MATLAB -> SITL  one JSON line per frame, sent back to the sender:
%                     {"timestamp":s,"imu":{"gyro":[..],"accel_body":[..]},
%                      "position":[N,E,D],"quaternion":[w,x,y,z],"velocity":[N,E,D]}
%   SITL runs in lockstep with this loop, so real ArduPilot code flies the
%   quad_physics model.
%
%   L = sitl_bridge(quad_physics(quad_params()), StopFcn=@() false, MaxSimTime=300);

arguments
    m quad_physics
    opts.Port (1,1) double = 9002
    opts.MaxSimTime (1,1) double = inf          % s of simulated time
    opts.StopFcn = @() false                    % polled every 400 frames
    opts.GyroNoise (1,1) double = deg2rad(0.3)  % rad/s RMS added to gyro
    opts.AccelNoise (1,1) double = 0.15         % m/s^2 RMS added to accel
    opts.LogDecim (1,1) double = 4              % log every Nth frame
    opts.Verbose (1,1) logical = true
    opts.BattVolt (1,1) double = NaN            % hold battery at this voltage (NaN = model sag)
    opts.VibAccel (1,3) double = [0 0 0]        % motor vibration on accel, RMS m/s^2 (x y z) at hover
    opts.VibGyro (1,3) double = [0 0 0]         % motor vibration on gyro, RMS rad/s at hover
    opts.VibHoverHz (1,1) double = 300          % rotor frequency at hover (1404 3700KV, 2S, 4")
    opts.MotorDelay (1,1) double = 0            % s of EXTRA transport delay on the motor commands (robustness tests)
end
pwmBuf = []; bufIdx = 1;
holdV = ~isnan(opts.BattVolt);
% motor vibration: each rotor adds a sinusoid at its own rotation frequency,
% f_i = f_hover*sqrt(T_i/T_hover); amplitude scales with T_i/T_hover
Thov = m.P.mass * m.P.g / 4;
vibPh = 2*pi*rand(4,1); vibTheta = 2*pi*rand(6,4);
useVib = any(opts.VibAccel) || any(opts.VibGyro);

% UDP via MATLAB's built-in Java runtime (no Instrument Control Toolbox needed)
sock = java.net.DatagramSocket(opts.Port);
sock.setSoTimeout(1000);
cleanupObj = onCleanup(@() sock.close()); %#ok<NASGU>
rxBuf = zeros(1, 512, 'int8'); rxPkt = java.net.DatagramPacket(rxBuf, numel(rxBuf));
m.reset(); if holdV, m.volt = opts.BattVolt; end
lastFrame = -1; frames = 0; reply = int8([]); addr = []; port = 0;
nLog = 0; cap = 200000; Lg = zeros(cap, 22);
tWall = tic; tPrint = tic; connected = false; idle = 0;
if opts.Verbose, fprintf('SITL bridge listening on UDP %d ...\n', opts.Port); end

while m.t < opts.MaxSimTime
    try
        sock.receive(rxPkt);                % blocks up to 1 s
    catch
        idle = idle + 1;
        if connected && idle > 5
            if opts.Verbose, fprintf('No data from SITL for 5 s - stopping bridge.\n'); end
            break
        end
        if opts.StopFcn(), break; end
        continue
    end
    idle = 0;
    raw = rxPkt.getData(); n = rxPkt.getLength();
    b = typecast(int8(raw(1:n)), 'uint8'); b = b(:)';
    if numel(b) < 8, continue; end
    magic = typecast(b(1:2), 'uint16');
    if magic == 18458, nch = 16; elseif magic == 29569, nch = 32; else, continue; end
    if numel(b) < 8 + 2*nch, continue; end
    rate = double(typecast(b(3:4), 'uint16'));
    frame = double(typecast(b(5:8), 'uint32'));
    pwm = double(typecast(b(9:8+2*nch), 'uint16')); pwm = pwm(:);

    if frame < lastFrame                    % SITL restarted
        m.reset(); if holdV, m.volt = opts.BattVolt; end; connected = false;
        if opts.Verbose, fprintf('SITL reset - physics reset\n'); end
    elseif frame == lastFrame               % duplicate: resend last answer
        if ~isempty(reply), sock.send(java.net.DatagramPacket(reply, numel(reply), addr, port)); end
        continue
    end
    lastFrame = frame;
    if ~connected
        connected = true; addr = rxPkt.getAddress(); port = rxPkt.getPort();
        if opts.Verbose, fprintf('Connected to SITL at %s:%d (%d Hz frames)\n', char(addr.getHostAddress()), port, rate); end
    end

    dt = 1 / max(rate, 50);
    u = pwm(1:4);
    if opts.MotorDelay > 0                  % FIFO of motor commands, MotorDelay * frame rate deep
        nd = max(1, round(opts.MotorDelay * rate));
        if size(pwmBuf, 2) ~= nd, pwmBuf = repmat(u, 1, nd); bufIdx = 1; end
        uOld = pwmBuf(:, bufIdx); pwmBuf(:, bufIdx) = u; bufIdx = mod(bufIdx, nd) + 1;
        u = uOld;
    end
    m.step(u, dt); if holdV, m.volt = opts.BattVolt; end
    s = m.sensors();
    g = s.gyro + opts.GyroNoise * randn(3,1);
    a = s.accel_body + opts.AccelNoise * randn(3,1);
    if useVib && ~m.onGround
        Tr = max(s.thrust(:), 0) / Thov;                     % thrust relative to hover
        vibPh = vibPh + 2*pi*opts.VibHoverHz*sqrt(Tr)*dt;
        amp = Tr;                                            % vibration grows with thrust
        sv = zeros(6,1);
        for kk = 1:6, sv(kk) = sum(amp .* sin(vibPh + vibTheta(kk,:)')); end
        sv = sv / sqrt(2);                                    % 4 unit-amplitude sines -> RMS 1.41 at hover
        a = a + opts.VibAccel(:) .* sv(1:3);
        g = g + opts.VibGyro(:) .* sv(4:6);
    end
    js = sprintf(['\n{"timestamp":%.6f,"imu":{"gyro":[%.6f,%.6f,%.6f],"accel_body":[%.5f,%.5f,%.5f]},' ...
        '"position":[%.5f,%.5f,%.5f],"quaternion":[%.8f,%.8f,%.8f,%.8f],"velocity":[%.5f,%.5f,%.5f]}\n'], ...
        s.time, g, a, s.position, s.quaternion, s.velocity);
    reply = int8(uint8(js));
    sock.send(java.net.DatagramPacket(reply, numel(reply), addr, port));
    frames = frames + 1;

    if mod(frames, opts.LogDecim) == 0
        nLog = nLog + 1;
        if nLog > cap, Lg = [Lg; zeros(cap, 22)]; cap = 2*cap; end %#ok<AGROW>
        Lg(nLog, :) = [s.time, s.position(:)', s.velocity(:)', s.euler(:)', s.gyro(:)', pwm(1:4)', s.thrust(:)', s.volt];
    end
    if opts.Verbose && toc(tPrint) > 5
        e = rad2deg(s.euler);
        fprintf('  sim %6.1f s | %4.0f fps (%3.0f%% real time) | alt %6.2f m | roll %6.1f pitch %6.1f yaw %6.1f deg | motors %4.0f %4.0f %4.0f %4.0f us\n', ...
            s.time, frames/toc(tWall), 100*s.time/toc(tWall), -s.position(3), e, pwm(1:4));
        tPrint = tic;
    end
    if mod(frames, 400) == 0 && opts.StopFcn(), break; end
end
Lg = Lg(1:nLog, :);
L = array2table(Lg, 'VariableNames', ["t","pN","pE","pD","vN","vE","vD","roll","pitch","yaw", ...
    "p","q","r","pwm1","pwm2","pwm3","pwm4","T1","T2","T3","T4","volt"]);
if opts.Verbose, fprintf('Bridge stopped after %d frames, %.1f s simulated.\n', frames, m.t); end
end
