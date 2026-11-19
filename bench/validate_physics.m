function R = validate_physics(opts)
%VALIDATE_PHYSICS Validate quad_physics against analytic solutions and
%independent reference models.
%
%   V1  Free fall                     analytic  z = z0 + g t^2 / 2
%   V2  Constant torque spin-up       analytic  w = M t / I
%   V3  Torque-free precession        analytic  Euler equations, axisymmetric body
%   V4  Energy + angular momentum     conservation, asymmetric body (Dzhanibekov spin)
%   V5  Hover trim                    equilibrium from the inverse thrust curve
%   V6  Motor effect signs            ArduPilot QuadX mixer convention (AP_MotorsMatrix)
%   V7  MathWorks Aerospace Blockset  6DOF (Quaternion) block, random forces/moments
%   V8  ArduPilot reference physics   SIM_multicopter.m from the ArduPilot repo,
%                                     same airframe, per-motor pulse responses
%
%   R = validate_physics();             % runs everything, saves figures
%   R = validate_physics(Simulink=false)

arguments
    opts.Simulink (1,1) logical = true
    opts.OutDir (1,1) string = ""
end
here = fileparts(mfilename('fullpath'));
if opts.OutDir == "", opts.OutDir = fullfile(here, "results"); end
if ~isfolder(opts.OutDir), mkdir(opts.OutDir); end
if usejava('desktop'), vis = 'on'; else, vis = 'off'; end

R = table('Size', [0 5], 'VariableTypes', ["string","string","string","string","string"], ...
    'VariableNames', ["Test","Reference","Measured","Limit","Result"]);
add = @(R, a, b, c, d, ok) [R; {string(a), string(b), string(c), string(d), ternary(ok)}];

P0 = quad_params();
fprintf('\n=== Physics validation (airframe: %.0f g, arm %.0f mm, Ixx %.2e Iyy %.2e Izz %.2e kg m^2)\n', ...
    P0.mass*1e3, P0.arm*1e3, P0.Ixx, P0.Iyy, P0.Izz);

%% V1 free fall (no drag)
P = quad_params(cd_xy=0, cd_z=0); m = quad_physics(P); m.reset(); m.x(3) = -50; m.onGround = false;
dt = 1e-3; T = 2;
for k = 1:round(T/dt), m.step([1000 1000 1000 1000], dt); end
zA = -50 + 0.5*P.g*T^2; err = abs(m.x(3) - zA);
fprintf('V1 free fall: z(2 s) = %.9f m, analytic %.9f m, error %.2e m\n', m.x(3), zA, err);
R = add(R, "V1 Free fall 2 s", "analytic", sprintf('err %.1e m', err), "< 1e-6 m", err < 1e-6);

%% V2 constant torque spin-up (pure rigid body)
P = quad_params(); M = [0.01; 0; 0]; x = rb0(); tEnd = 0.5; h = 1e-3;
for k = 1:round(tEnd/h), x = rbStep(x, [0;0;0], M, P.mass, P.I, 0, h); end
wA = M(1)/P.Ixx*tEnd; err = abs(x(11) - wA)/wA;
fprintf('V2 torque spin-up: wx = %.6f rad/s, analytic %.6f, rel error %.2e\n', x(11), wA, err);
R = add(R, "V2 Constant torque", "analytic", sprintf('rel err %.1e', err), "< 1e-9", err < 1e-9);

%% V3 torque-free precession, axisymmetric body
Ia = 1e-3; Ic = 1.8e-3; I = diag([Ia Ia Ic]); W = 20; w0 = 0.5;
lam = (Ic - Ia)/Ia * W; x = rb0(); x(11:13) = [w0; 0; W]; h = 5e-4; tEnd = 5;
n = round(tEnd/h); wx = zeros(n,1); wy = wx; tt = (1:n)'*h;
for k = 1:n, x = rbStep(x, [0;0;0], [0;0;0], 1, I, 0, h); wx(k) = x(11); wy(k) = x(12); end
wxA = w0*cos(lam*tt); wyA = w0*sin(lam*tt);
err = max(hypot(wx - wxA, wy - wyA)) / w0;
fprintf('V3 precession: lambda = %.3f rad/s, max rel error %.2e over %g s\n', lam, err, tEnd);
R = add(R, "V3 Torque-free precession", "analytic (Euler eqs)", sprintf('rel err %.1e', err), "< 1e-6", err < 1e-6);

%% V4 conservation, asymmetric body, intermediate-axis spin
I = diag([0.8e-3 1.2e-3 1.9e-3]); x = rb0(); x(11:13) = [0.01; 15; 0.01]; h = 5e-4; tEnd = 20;
E0 = 0.5*x(11:13)'*I*x(11:13); H0 = quad_physics.quat2dcm(x(7:10))*I*x(11:13);
n = round(tEnd/h); wyLog = zeros(n,1);
for k = 1:n, x = rbStep(x, [0;0;0], [0;0;0], 1, I, 0, h); wyLog(k) = x(12); end
E1 = 0.5*x(11:13)'*I*x(11:13); H1 = quad_physics.quat2dcm(x(7:10))*I*x(11:13);
dE = abs(E1 - E0)/E0; dH = norm(H1 - H0)/norm(H0); flips = sum(abs(diff(sign(wyLog))) > 0);
fprintf('V4 conservation: energy drift %.2e, momentum drift %.2e, intermediate-axis flips %d\n', dE, dH, flips);
R = add(R, "V4 Energy & momentum", "conservation laws", sprintf('dE %.1e, dH %.1e', dE, dH), "< 1e-7", dE < 1e-7 && dH < 1e-7);
R = add(R, "V4 Dzhanibekov effect", "rigid-body theory", sprintf('%d flips in 20 s', flips), ">= 2 (unstable axis)", flips >= 2);

%% V5 hover trim
P = quad_params(); m = quad_physics(P); m.reset(); m.x(3) = -20; m.onGround = false;
m.volt = m.Vref; pwmH = hoverPwm(P, 1.0);
m.x(14:17) = P.mass*P.g/4;
for k = 1:round(3/dt), m.step(pwmH*[1 1 1 1], dt); m.volt = m.Vref; end
dz = abs(m.x(3) + 20); att = max(abs(rad2deg(quad_physics.quat2eul(m.x(7:10)))));
fprintf('V5 hover: PWM %.1f us, altitude change %.4f m, attitude drift %.2e deg after 3 s\n', pwmH, dz, att);
R = add(R, "V5 Hover equilibrium", "trim from thrust curve", sprintf('dz %.1e m, att %.1e deg', dz, att), "< 0.01 m, < 1e-6 deg", dz < 0.01 && att < 1e-6);

%% V6 motor signs (ArduPilot QuadX)
% expected sign of [roll pitch yaw] acceleration when motor i increases
exp6 = [-1 +1 +1;   % M1 front-right CCW
        +1 -1 +1;   % M2 rear-left  CCW
        +1 +1 -1;   % M3 front-left CW
        -1 -1 -1];  % M4 rear-right CW
ok6 = true; names = ["M1 front-right CCW","M2 rear-left CCW","M3 front-left CW","M4 rear-right CW"];
for i = 1:4
    m = quad_physics(P); m.reset(); m.x(3) = -20; m.onGround = false; m.x(14:17) = P.mass*P.g/4; m.volt = m.Vref;
    pw = pwmH*[1 1 1 1]; pw(i) = pw(i) + 50;
    for k = 1:100, m.step(pw, dt); m.volt = m.Vref; end
    s = sign(m.x(11:13))';
    good = isequal(s, exp6(i,:)); ok6 = ok6 && good;
    fprintf('V6 %s: d(roll,pitch,yaw) rate signs %+d %+d %+d (expected %+d %+d %+d) %s\n', names(i), s, exp6(i,:), ternary(good));
end
R = add(R, "V6 Motor order & directions", "ArduPilot AP_MotorsMatrix QuadX", "4/4 motors checked", "all signs match", ok6);

%% V7 MathWorks Aerospace Blockset 6DOF (Quaternion)
if opts.Simulink
    try
        [e7, d7] = compareAero6dof(P, opts.OutDir, vis);
        fprintf('V7 Aerospace 6DOF: max |dX| %.2e m, |dV| %.2e m/s, |dEuler| %.2e deg, |dW| %.2e rad/s\n', e7.pos, e7.vel, e7.eul, e7.w);
        ok = e7.pos < 1e-6 && e7.vel < 1e-6 && e7.eul < 1e-5 && e7.w < 1e-6;
        R = add(R, "V7 Rigid body vs 6DOF block", "MathWorks Aerospace Blockset", ...
            sprintf('dX %.0e m, dV %.0e m/s, dAtt %.0e deg', e7.pos, e7.vel, e7.eul), "< 1e-6", ok);
    catch err
        fprintf('V7 skipped: %s\n', err.message); d7 = []; %#ok<NASGU>
        R = add(R, "V7 Rigid body vs 6DOF block", "MathWorks Aerospace Blockset", "error: " + err.message, "-", false);
    end
end

%% V8 ArduPilot reference multicopter physics
[r8, fig8] = compareArduPilotRef(here, opts.OutDir, vis);
for i = 1:4
    fprintf('V8 %s: rate response correlation roll %.3f pitch %.3f yaw %.3f | peak ratio %.2f %.2f %.2f\n', ...
        names(i), r8.corr(i,:), r8.ratio(i,:));
end
fprintf('V8 hover: reference thrust %.2f N vs model %.2f N; climb accel after +100 us: ref %.2f, model %.2f m/s^2\n', ...
    r8.hoverRef, r8.hoverMine, r8.climbRef, r8.climbMine);
okCorr = all(r8.corr(:) > 0.97); okRat = all(abs(r8.ratio(:) - 1) < 0.10);
R = add(R, "V8 Sign/shape vs ArduPilot model", "ArduPilot SIM_multicopter.m", sprintf('min corr %.3f', min(r8.corr(:))), "> 0.97", okCorr);
R = add(R, "V8 Magnitude vs ArduPilot model", "ArduPilot SIM_multicopter.m", sprintf('peak ratio %.2f-%.2f', min(r8.ratio(:)), max(r8.ratio(:))), "within 10 %", okRat);
R = add(R, "V8 Vertical response", "ArduPilot SIM_multicopter.m", sprintf('%.2f vs %.2f m/s^2', r8.climbMine, r8.climbRef), "within 10 %", abs(r8.climbMine/r8.climbRef - 1) < 0.10);
if ~isempty(fig8) && strcmp(vis, 'off'), delete(fig8); end

%% report
fprintf('\n==================== PHYSICS VALIDATION REPORT ====================\n');
disp(R);
fprintf('PASS %d / %d\n', sum(R.Result == "PASS"), height(R));
writetable(R, fullfile(opts.OutDir, "physics_validation.csv"));
end

%% ---------------------------------------------------------------------
function x = rb0()
x = zeros(13,1); x(7) = 1;
end

function x = rbStep(x, F, M, m, I, g, h)
f = @(x) rbDeriv(x, F, M, m, I, g);
k1 = f(x); k2 = f(x + h/2*k1); k3 = f(x + h/2*k2); k4 = f(x + h*k3);
x = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
x(7:10) = x(7:10)/norm(x(7:10));
end

function xd = rbDeriv(x, F, M, m, I, g)
[vd, qd, wd] = quad_physics.rigid(x(7:10), x(11:13), F, M, m, I, g);
xd = [x(4:6); vd; qd; wd];
end

function pwm = hoverPwm(P, vRatio)
% invert ArduPilot thrust curve for T = W/4
Tn = (P.mass*P.g/4) / (P.Tmax * vRatio^2);
e = P.expo;
if e > 0, u = (-(1-e) + sqrt((1-e)^2 + 4*e*Tn)) / (2*e); else, u = Tn; end
c = P.spin_min + u*(P.spin_max - P.spin_min);
pwm = P.pwm_min + c*(P.pwm_max - P.pwm_min);
end

function s = ternary(ok)
if ok, s = "PASS"; else, s = "FAIL"; end
end

%% ---------------------------------------------------------------------
function [e, data] = compareAero6dof(P, outDir, vis)
% Smooth multi-sine body forces/moments, sampled at h/2 so every RK4 stage
% time of the ode4 solver is an exact sample point (no input-timing error).
h = 1e-3; tEnd = 5;
fF = @(t) [0.5*sin(2.1*t) + 0.3*cos(5.3*t); 0.4*sin(3.7*t + 1); -0.6*cos(1.3*t) + 0.2*sin(7.1*t)];
fM = @(t) [2e-3*sin(4.3*t); 1.5e-3*cos(2.9*t) + 1e-3*sin(8.1*t); 1e-3*sin(1.7*t + 0.5)];
ts = (0:h/2:tEnd + h)';
Fts = timeseries(cell2mat(arrayfun(@(t) fF(t)', ts, 'UniformOutput', false)), ts);
Mts = timeseries(cell2mat(arrayfun(@(t) fM(t)', ts, 'UniformOutput', false)), ts);
mdl = 'qp_val6dof';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl); load_system('aerolib6dof2');
add_block('simulink/Sources/From Workspace', [mdl '/F'], 'VariableName', 'Fts', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Sources/From Workspace', [mdl '/M'], 'VariableName', 'Mts', 'Interpolate', 'on', 'OutputAfterFinalValue', 'Holding final value');
add_block('aerolib6dof2/6DOF (Quaternion)', [mdl '/SixDOF']);
set_param([mdl '/SixDOF'], 'units', 'Metric (MKS)', 'mtype', 'Fixed', 'xme_0', '[0 0 -10]', 'Vm_0', '[0 0 0]', ...
    'eul_0', '[0 0 0]', 'pm_0', '[0 0 0]', 'mass_0', 'massV', 'inertia', 'Iv');
add_line(mdl, 'F/1', 'SixDOF/1'); add_line(mdl, 'M/1', 'SixDOF/2');
outs = ["Ve","Xe","Eul","Quat","DCM","Vb","Wb"];   % 6DOF (Quaternion) output order
for i = 1:7
    add_block('simulink/Sinks/To Workspace', [mdl '/o' char(outs(i))], 'VariableName', char("o_" + outs(i)), 'SaveFormat', 'Timeseries');
    add_line(mdl, sprintf('SixDOF/%d', i), ['o' char(outs(i)) '/1']);
end
set_param(mdl, 'Solver', 'ode4', 'FixedStep', num2str(h), 'StopTime', num2str(tEnd));
in = Simulink.SimulationInput(mdl);
in = in.setVariable('Fts', Fts).setVariable('Mts', Mts).setVariable('massV', P.mass).setVariable('Iv', P.I);
out = sim(in);
close_system(mdl, 0);
% same inputs through quad_physics.rigid with RK4 (gravity off: the block has none)
x = rb0(); x(3) = -10; N = round(tEnd/h);
X = zeros(N+1, 13); X(1,:) = x';
for k = 1:N
    t = (k-1)*h;
    f = @(x, tt) rbDeriv(x, fF(tt), fM(tt), P.mass, P.I, 0);
    k1 = f(x, t); k2 = f(x + h/2*k1, t + h/2); k3 = f(x + h/2*k2, t + h/2); k4 = f(x + h*k3, t + h);
    x = x + h/6*(k1 + 2*k2 + 2*k3 + k4); x(7:10) = x(7:10)/norm(x(7:10));
    X(k+1,:) = x';
end
tt = (0:N)'*h;
Xe = sig(out.o_Xe, tt); Ve = sig(out.o_Ve, tt); Eu = sig(out.o_Eul, tt); Wb = sig(out.o_Wb, tt);
myEul = zeros(N+1, 3);
for k = 1:N+1, myEul(k,:) = quad_physics.quat2eul(X(k,7:10)')'; end
dEul = rad2deg(abs(atan2(sin(myEul - Eu), cos(myEul - Eu))));
e.pos = max(abs(X(:,1:3) - Xe), [], 'all'); e.vel = max(abs(X(:,4:6) - Ve), [], 'all');
e.eul = max(dEul, [], 'all'); e.w = max(abs(X(:,11:13) - Wb), [], 'all');
data = struct('t', tt, 'mine', X, 'Xe', Xe, 'Ve', Ve, 'Eul', Eu, 'Wb', Wb);
f = figure('Visible', vis, 'Color', 'w', 'Position', [60 60 1300 700]);
tl = tiledlayout(2, 2, 'TileSpacing', 'compact'); title(tl, 'V7: quad\_physics vs MathWorks Aerospace Blockset 6DOF (Quaternion)');
nexttile; plot(tt, Xe, 'k-', 'LineWidth', 2); hold on; plot(tt, X(:,1:3), '--', 'LineWidth', 1.2); title('Position NED (m): 6DOF solid, quad\_physics dashed'); grid on
nexttile; plot(tt, rad2deg(Eu), 'k-', 'LineWidth', 2); hold on; plot(tt, rad2deg(myEul), '--', 'LineWidth', 1.2); title('Euler angles (deg)'); grid on
nexttile; plot(tt, Wb, 'k-', 'LineWidth', 2); hold on; plot(tt, X(:,11:13), '--', 'LineWidth', 1.2); title('Body rates (rad/s)'); grid on
nexttile; semilogy(tt, max(abs(X(:,1:3) - Xe), [], 2) + 1e-16); hold on; semilogy(tt, max(dEul, [], 2) + 1e-16); title('Absolute difference'); legend('position (m)', 'attitude (deg)'); grid on
exportgraphics(f, fullfile(outDir, "V7_aerospace_6dof.png"), 'Resolution', 110);
if strcmp(vis, 'off'), delete(f); end
end

function D = sig(ts, tt)
% timeseries -> [numel(tt) x n] regardless of how Simulink stored the dims
D = ts.Data; nt = numel(ts.Time);
if size(D, 1) == nt, D = reshape(D, nt, []); else, D = reshape(D, [], nt)'; end
D = interp1(ts.Time, D, tt);
end

%% ---------------------------------------------------------------------
function [r, f] = compareArduPilotRef(here, outDir, vis)
refDir = fullfile(fileparts(here), "sitl", "ardupilot_ref");
S = load(fullfile(refDir, "Copter_Hexsoon.mat"), 'copter');
dt = 1e-3;
st0 = refInit(S.copter, dt);
% --- characterise the reference motors: steady thrust vs PWM, time constant, torque/thrust
pw = 1150:50:1900; Tss = zeros(size(pw)); Qss = Tss;
for i = 1:numel(pw)
    st = st0; st.position = [0;0;-1e6];
    for k = 1:3000, st = ap_ref_multicopter(pw(i)*ones(1,16), st); end
    Tss(i) = st.copter.motors(1).thrust; Qss(i) = abs(st.copter.motors(1).torque);
end
u = (pw - 1100)/800;
A = [u(:) u(:).^2]; coef = A \ Tss(:);       % T = a u + b u^2
Tmax = sum(coef); expo = coef(2)/Tmax;
kq_kt = median(Qss ./ Tss);
% time constant from a 1500 -> 1600 step
st = st0; st.position = [0;0;-1e6];
for k = 1:3000, st = ap_ref_multicopter(1500*ones(1,16), st); end
T0 = st.copter.motors(1).thrust; Tr = zeros(1500,1);
for k = 1:1500, st = ap_ref_multicopter(1600*ones(1,16), st); Tr(k) = st.copter.motors(1).thrust; end
tau = find(Tr >= T0 + 0.632*(Tr(end) - T0), 1) * dt;
% --- matched quad_physics configuration
c = S.copter; Pm = quad_params( ...
    'mass', c.mass, 'arm', norm(c.motors(1).location(1:2)), 'Ixx', c.inertia(1,1), 'Iyy', c.inertia(2,2), 'Izz', c.inertia(3,3), ...
    'TW', 4*Tmax/(c.mass*9.80665), 'tau_m', tau, 'kq_kt', kq_kt, 'expo', expo, 'spin_min', 0, 'spin_max', 1, ...
    'pwm_min', 1100, 'pwm_max', 1900, 'cd_xy', 0, 'cd_z', 0, 'kd_rot', 0, 'kd_rot2', 0.2, 'r_batt', 0, ...
    'v_full', c.battery.voltage, 'v_empty', c.battery.voltage - 1e-6, 'capacity_ah', 1e6);
pH = hoverPwm(Pm, 1.0);
% --- per-motor pulse tests. Reference motor i (ArduPilot channel ch) vs model motor ch
chans = [1 2 3 4]; nT = 1500; pulse = 50; corrM = zeros(4,3); ratio = corrM;
f = figure('Visible', vis, 'Color', 'w', 'Position', [60 60 1300 800]);
tl = tiledlayout(4, 3, 'TileSpacing', 'compact');
title(tl, 'V8: quad\_physics (dashed) vs ArduPilot SIM\_multicopter.m (solid), +50 us pulse on one motor');
ax = ["roll rate","pitch rate","yaw rate"];
for i = 1:4
    ch = chans(i);
    st = st0; st.position = [0;0;-1000];
    m = quad_physics(Pm); m.reset(); m.x(3) = -1000; m.onGround = false; m.Vref = c.battery.voltage; m.volt = m.Vref;
    for k = 1:3000   % settle both at hover thrust
        p16 = pH*ones(1,16); st = ap_ref_multicopter(p16, st); st.gyro = [0;0;0]; st.dcm = eye(3); st.velocity = [0;0;0];
        m.step(pH*[1 1 1 1], dt); m.volt = m.Vref; m.x(11:13) = 0; m.x(7:10) = [1;0;0;0]; m.x(4:6) = 0;
    end
    wr = zeros(nT,3); wm = wr;
    for k = 1:nT
        p16 = pH*ones(1,16); if k <= 300, p16(ch) = pH + pulse; end
        st = ap_ref_multicopter(p16, st); m.step(p16(1:4), dt); m.volt = m.Vref;
        wr(k,:) = st.gyro'; wm(k,:) = m.x(11:13)';
    end
    for a = 1:3
        cc = corrcoef(wr(:,a), wm(:,a)); corrM(i,a) = cc(1,2);
        ratio(i,a) = max(abs(wm(:,a))) / max(abs(wr(:,a)));
        nexttile; plot((1:nT)*dt, wr(:,a), 'k-', 'LineWidth', 1.6); hold on; plot((1:nT)*dt, wm(:,a), '--', 'LineWidth', 1.4);
        title(sprintf('M%d: %s', ch, ax(a))); grid on
    end
end
exportgraphics(f, fullfile(outDir, "V8_ardupilot_reference.png"), 'Resolution', 110);
% --- vertical: +100 us on all motors from hover
st = st0; st.position = [0;0;-1000];
m = quad_physics(Pm); m.reset(); m.x(3) = -1000; m.onGround = false; m.Vref = c.battery.voltage; m.volt = m.Vref;
for k = 1:3000
    st = ap_ref_multicopter(pH*ones(1,16), st); st.velocity = [0;0;0];
    m.step(pH*[1 1 1 1], dt); m.volt = m.Vref; m.x(4:6) = 0;
end
r.hoverRef = sum([st.copter.motors.thrust]); r.hoverMine = sum(m.x(14:17));
v0r = st.velocity(3); v0m = m.x(6);
for k = 1:500, st = ap_ref_multicopter((pH+100)*ones(1,16), st); m.step((pH+100)*[1 1 1 1], dt); m.volt = m.Vref; end
r.climbRef = -(st.velocity(3) - v0r)/0.5; r.climbMine = -(m.x(6) - v0m)/0.5;
r.corr = corrM; r.ratio = ratio; r.ref = struct('Tmax', Tmax, 'expo', expo, 'tau', tau, 'kq_kt', kq_kt);
fprintf('V8 reference motor characterised: Tmax %.2f N, expo %.2f, tau %.0f ms, kq/kt %.4f m, hover PWM %.0f us\n', Tmax, expo, tau*1e3, kq_kt, pH);
end

function st = refInit(copter, dt)
st.copter = copter;
for i = 1:numel(st.copter.motors), st.copter.motors(i).rpm = 0; st.copter.motors(i).current = 0; end
st.environment.density = 1.225; st.gravity_mss = 9.80665; st.delta_t = dt;
st.gyro = [0;0;0]; st.dcm = eye(3); st.attitude = [0;0;0]; st.accel = [0;0;0];
st.velocity = [0;0;0]; st.position = [0;0;0]; st.bf_velo = [0;0;0];
end
