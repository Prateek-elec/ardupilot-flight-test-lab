classdef quad_physics < handle
    %QUAD_PHYSICS 6-DOF quadrotor rigid-body model (Newton-Euler, quaternion).
    %
    %   Equations (Mahony, Kumar & Corke, IEEE RAM 2012, NED/FRD form):
    %     p_dot = v
    %     v_dot = R(q) * F_b / m + [0 0 g]'
    %     q_dot = 0.5 * q (x) [0 w]
    %     w_dot = I^-1 ( M_b - w x I w )
    %   Rotor i: T_i_dot = (T_ss(c_i, V) - T_i) / tau_m,  Q_i = kq_kt * T_i
    %   T_ss = Tmax (V/Vref)^2 [(1-e) u + e u^2],  u = (c - spin_min)/(spin_max - spin_min)
    %   F_b = [0 0 -sum T_i]' - drag,   M_b = sum r_i x F_i + sum dir_i Q_i z - kd w
    %
    %   q = [w x y z] rotates body -> NED. Motors in ArduPilot QuadX order.
    %
    %   m = quad_physics(quad_params());
    %   m.step(pwm(1:4), dt);  s = m.sensors();

    properties
        P                       % parameters (quad_params)
        x                       % state: [p(3) v(3) q(4) w(3) T(4)]
        ah = 0                  % battery charge used (Ah)
        volt                    % battery terminal voltage (V)
        amps = 0                % battery current (A)
        wind = [0;0;0]          % steady wind, NED (m/s)
        gust = @(t) [0;0;0]     % extra wind function of time, NED (m/s)
        dist_torque = @(t) [0;0;0] % external torque disturbance, body (N m)
        t = 0                   % sim time (s)
        onGround = true
        accBody = [0;0;0]       % last specific force (accelerometer), body
        Vref = 7.6              % voltage at which TW was measured (hover average in log)
        substeps = 2            % RK4 sub-steps per call
    end

    methods
        function obj = quad_physics(P)
            if nargin < 1, P = quad_params(); end
            obj.P = P;
            obj.reset();
        end

        function reset(obj)
            obj.x = zeros(17,1); obj.x(7) = 1;
            obj.t = 0; obj.ah = 0; obj.volt = obj.P.v_full; obj.amps = 0;
            obj.onGround = true; obj.accBody = [0;0;-obj.P.g];
        end

        function step(obj, pwm, dt)
            % Advance by dt with motor PWM commands (us), ArduPilot motor order.
            pwm = pwm(:);
            c = (pwm(1:4) - obj.P.pwm_min) / (obj.P.pwm_max - obj.P.pwm_min);
            h = dt / obj.substeps;
            for k = 1:obj.substeps
                obj.x = obj.rk4(obj.x, c, h, obj.t);
                obj.t = obj.t + h;
                obj.groundContact();
            end
            nq = norm(obj.x(7:10)); obj.x(7:10) = obj.x(7:10) / nq;
            [~, aB] = obj.deriv(obj.x, c, obj.t);
            if obj.onGround
                R = quad_physics.quat2dcm(obj.x(7:10));
                aB = -R.' * [0;0;obj.P.g];
            end
            obj.accBody = aB;
            % battery
            T = max(obj.x(14:17), 0);
            obj.amps = sum(obj.P.k_amp * T.^1.5) + 0.3;
            obj.ah = obj.ah + obj.amps * dt / 3600;
            soc = max(0, 1 - obj.ah / obj.P.capacity_ah);
            voc = obj.P.v_empty + (obj.P.v_full - obj.P.v_empty) * soc;
            obj.volt = voc - obj.amps * obj.P.r_batt;
        end

        function s = sensors(obj)
            s.time = obj.t;
            s.gyro = obj.x(11:13);
            s.accel_body = obj.accBody;
            s.position = obj.x(1:3);
            s.velocity = obj.x(4:6);
            s.quaternion = obj.x(7:10);
            s.euler = quad_physics.quat2eul(obj.x(7:10));
            s.thrust = obj.x(14:17);
            s.volt = obj.volt; s.amps = obj.amps;
        end

        function [xd, aB] = deriv(obj, x, c, t)
            P = obj.P;
            v = x(4:6); q = x(7:10); w = x(11:13); T = x(14:17);
            R = quad_physics.quat2dcm(q);
            % rotor steady-state thrust from command (ArduPilot thrust curve)
            u = (c - P.spin_min) / (P.spin_max - P.spin_min);
            u = min(max(u, 0), 1);
            vs = (obj.volt / obj.Vref)^2;
            Tss = P.Tmax * vs * ((1 - P.expo) * u + P.expo * u.^2);
            Tss(c <= 0.02) = 0;
            Td = (Tss - T) / P.tau_m;
            Tpos = max(T, 0);
            % aerodynamics (air-relative)
            vAir = R.' * (v - obj.wind - obj.gust(t));
            drag = -[P.cd_xy*abs(vAir(1))*vAir(1); P.cd_xy*abs(vAir(2))*vAir(2); P.cd_z*abs(vAir(3))*vAir(3)];
            Fb = [0; 0; -sum(Tpos)] + drag;
            % moments
            Mb = [0;0;0];
            for i = 1:4
                r = P.mpos(:,i);
                Mb = Mb + [ -r(2)*Tpos(i); r(1)*Tpos(i); 0 ];
            end
            Mb(3) = Mb(3) + sum(P.mdir(:) .* (P.kq_kt * Tpos));
            Mb = Mb - P.kd_rot * w - P.kd_rot2 * sign(w) .* w.^2 + obj.dist_torque(t);
            % rigid body
            aB = Fb / P.mass;                         % specific force (accelerometer)
            [vd, qd, wd] = quad_physics.rigid(q, w, Fb, Mb, P.mass, P.I, P.g);
            xd = [v; vd; qd; wd; Td];
        end
    end

    methods (Access = private)
        function x = rk4(obj, x, c, h, t)
            k1 = obj.deriv(x, c, t);
            k2 = obj.deriv(x + h/2*k1, c, t + h/2);
            k3 = obj.deriv(x + h/2*k2, c, t + h/2);
            k4 = obj.deriv(x + h*k3, c, t + h);
            x = x + h/6 * (k1 + 2*k2 + 2*k3 + k4);
        end

        function groundContact(obj)
            % ground at z = 0 (NED). Resting when below ground and not climbing.
            if obj.x(3) >= 0
                lift = sum(max(obj.x(14:17), 0));
                if obj.x(6) >= 0 || lift < obj.P.mass * obj.P.g
                    obj.x(3) = 0; obj.x(4:6) = 0; obj.x(11:13) = 0;
                    % keep yaw, level roll/pitch on the ground
                    e = quad_physics.quat2eul(obj.x(7:10));
                    obj.x(7:10) = quad_physics.eul2quat([0 0 e(3)]);
                    obj.onGround = true;
                    return
                end
            end
            obj.onGround = false;
        end
    end

    methods (Static)
        function [vd, qd, wd] = rigid(q, w, Fb, Mb, m, I, g)
            % Newton-Euler rigid body, body forces/moments, NED gravity.
            R = quad_physics.quat2dcm(q);
            vd = R * (Fb / m) + [0; 0; g];
            wd = I \ (Mb - cross(w, I * w));
            qd = 0.5 * quad_physics.quatmult(q, [0; w]);
        end

        function R = quat2dcm(q)
            % body -> NED rotation matrix from q = [w x y z]
            w = q(1); x = q(2); y = q(3); z = q(4);
            R = [1-2*(y^2+z^2), 2*(x*y-w*z),   2*(x*z+w*y);
                 2*(x*y+w*z),   1-2*(x^2+z^2), 2*(y*z-w*x);
                 2*(x*z-w*y),   2*(y*z+w*x),   1-2*(x^2+y^2)];
        end

        function r = quatmult(a, b)
            r = [a(1)*b(1) - a(2)*b(2) - a(3)*b(3) - a(4)*b(4);
                 a(1)*b(2) + a(2)*b(1) + a(3)*b(4) - a(4)*b(3);
                 a(1)*b(3) - a(2)*b(4) + a(3)*b(1) + a(4)*b(2);
                 a(1)*b(4) + a(2)*b(3) - a(3)*b(2) + a(4)*b(1)];
        end

        function e = quat2eul(q)
            % [roll pitch yaw] (rad), ZYX
            w = q(1); x = q(2); y = q(3); z = q(4);
            e = [atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2));
                 asin(max(min(2*(w*y - z*x), 1), -1));
                 atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))];
        end

        function q = eul2quat(e)
            cr = cos(e(1)/2); sr = sin(e(1)/2); cp = cos(e(2)/2); sp = sin(e(2)/2);
            cy = cos(e(3)/2); sy = sin(e(3)/2);
            q = [cr*cp*cy + sr*sp*sy; sr*cp*cy - cr*sp*sy; cr*sp*cy + sr*cp*sy; cr*cp*sy - sr*sp*cy];
        end
    end
end
