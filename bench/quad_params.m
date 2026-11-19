function P = quad_params(varargin)
%QUAD_PARAMS Airframe parameters for the 6-DOF test bench.
%
%   Airframe (from the owner): 280 g all-up, 220 mm motor-to-motor diagonal,
%   4-inch props, 1404 3700 KV motors, AM32 ESCs, 2S 5000 mAh (Li-ion assumed).
%   Values marked (EST) are estimates; (LOG) values are calibrated to the
%   9 Nov 2026 flight log (hover throttle 0.50, identified roll/pitch/yaw
%   authority 297/224/36 rad/s^2 per unit controller output).
%   Override any field:  P = quad_params(mass=0.30, tau_m=0.025);
%
%   Conventions: body FRD (x forward, y right, z down), earth NED.
%   Motor order = ArduPilot QuadX: M1 front-right CCW, M2 rear-left CCW,
%   M3 front-left CW, M4 rear-right CW.

o = struct( ...
    'mass',        0.280, ...  % kg all-up (owner)
    'arm',         0.110, ...  % m centre-to-motor = 220 mm diagonal / 2 (owner)
    'Ixx',         [], ...     % kg m^2 (derived from identified authority if empty)
    'Iyy',         [], ...
    'Izz',         [], ...
    'TW',          2.0, ...    % max thrust / weight (LOG: hover throttle 0.50 on linearised thrust)
    'tau_m',       0.030, ...  % s 1404 motor + 4" prop spin-up time constant (EST, LOG range 26-48 ms)
    'kq_kt',       0.012, ...  % m, prop torque/thrust ratio, 4-inch 2-blade (EST)
    'expo',        0.49, ...   % MOT_THST_EXPO of the real motor/prop (matches the FC setting)
    'spin_min',    0.09, ...   % MOT_SPIN_MIN
    'spin_max',    0.95, ...   % MOT_SPIN_MAX
    'pwm_min',     1000, ...
    'pwm_max',     2000, ...
    'cells',       2, ...
    'v_full',      8.4, ...    % V
    'v_empty',     6.0, ...    % Li-ion 2S
    'capacity_ah', 5.0, ...    % (owner)
    'r_batt',      0.060, ...  % ohm 2S Li-ion pack + wiring (EST)
    'hover_amps',  7.5, ...    % A at hover (EST: ~60 W for 280 g on 4" props)
    'cd_xy',       0.012, ...  % N/(m/s)^2 = 0.5*rho*Cd*A, Cd~1, A~0.02 m^2 (220 mm frame, EST)
    'cd_z',        0.018, ...  % larger plan-form area vertically (EST)
    'kd_rot',      5e-5, ...   % N m/(rad/s) rotational damping (EST)
    'kd_rot2',     0, ...      % N m/(rad/s)^2 quadratic rotational drag (ArduPilot ref model uses 0.2)
    'auth_roll',   297, ...    % rad/s^2 per unit roll output (identified)
    'auth_pitch',  224, ...
    'auth_yaw',    36, ...
    'g',           9.80665, ...
    'rho',         1.225);

for i = 1:2:numel(varargin)
    o.(varargin{i}) = varargin{i+1};
end
P = o;

% motor geometry (ArduPilot QuadX angles, degrees clockwise from forward)
ang = [45 -135 -45 135];
P.mpos = P.arm * [cosd(ang); sind(ang); zeros(1,4)];   % 3x4 body positions
P.mdir = [+1 +1 -1 -1];      % +1 = CCW prop (reaction torque +yaw), -1 = CW

W = P.mass * P.g;
P.Tmax = P.TW * W / 4;       % N per motor at full command
P.Qmax = P.kq_kt * P.Tmax;   % N m per motor

% ArduPilot QuadX mixer: full roll output moves each motor by +/-0.5*Tmax
% (factors normalised so the largest is 0.5). Torque per unit output:
lever = P.arm * sind(45);
tRoll = 4 * 0.5 * P.Tmax * lever;
tYaw  = 4 * 0.5 * P.Qmax;
if isempty(P.Ixx), P.Ixx = tRoll / P.auth_roll; end
if isempty(P.Iyy), P.Iyy = tRoll / P.auth_pitch; end
if isempty(P.Izz), P.Izz = max(tYaw / P.auth_yaw, P.Ixx + P.Iyy - 1e-6); end
P.I = diag([P.Ixx P.Iyy P.Izz]);

% current model: I = k * T^1.5 per motor, calibrated to hover_amps
P.k_amp = P.hover_amps / (4 * (W/4)^1.5);
end
