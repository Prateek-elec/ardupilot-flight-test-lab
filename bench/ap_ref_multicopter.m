function state = ap_ref_multicopter(pwm_in, state)
%AP_REF_MULTICOPTER ArduPilot's reference MATLAB multicopter physics, one step.
%   Equations copied unchanged from ArduPilot
%   libraries/SITL/examples/JSON/MATLAB/Copter/SIM_multicopter.m (GPLv3),
%   wrapped as a callable function for side-by-side validation.
%   state must contain: copter (from Copter.m / Hexsoon.mat), environment.density,
%   gravity_mss, delta_t, gyro, dcm, attitude, accel, velocity, position, bf_velo.

state.copter.battery.current = sum([state.copter.motors.current]);
state.copter.battery.dropped_voltage = state.copter.battery.voltage - state.copter.battery.resistance * state.copter.battery.current;

for i = 1:numel(state.copter.motors)
    motor = state.copter.motors(i);
    throttle = (pwm_in(motor.channel) - 1100) / 800;
    throttle = max(throttle,0);
    throttle = min(throttle,1);
    voltage = throttle * state.copter.battery.dropped_voltage;
    Kt = 1/(motor.electrical.kv * ( (2*pi)/60) );
    current = ((motor.electrical.kv * voltage) - motor.rpm) / ((motor.electrical.resistance + motor.esc.resistance) * motor.electrical.kv);
    torque = current * Kt;
    prop_drag = motor.prop.PConst * state.environment.density * (motor.rpm/60)^2 * motor.prop.diameter^5;
    w = motor.rpm * ((2*pi)/60);
    w1 = w + ((torque-prop_drag) / motor.prop.inertia) * state.delta_t;
    rps = w1 * (1/(2*pi));
    rps = max(rps,0);
    thrust = 2.2 * motor.prop.TConst * state.environment.density * rps^2 * motor.prop.diameter^4;
    moment_roll = thrust * motor.location(1);
    moment_pitch = thrust * motor.location(2);
    moment_yaw = -torque * motor.direction;
    state.copter.motors(i).torque = torque;
    state.copter.motors(i).current = current;
    state.copter.motors(i).rpm = rps * 60;
    state.copter.motors(i).thrust = thrust;
    state.copter.motors(i).moment_roll = moment_roll;
    state.copter.motors(i).moment_pitch = moment_pitch;
    state.copter.motors(i).moment_yaw = moment_yaw;
end

drag = sign(state.bf_velo) .* state.copter.cd .* state.copter.cd_ref_area .* 0.5 .* state.environment.density .* state.bf_velo.^2;
force = [0;0;-sum([state.copter.motors.thrust])] - drag;
rotational_drag = 0.2 * sign(state.gyro) .* state.gyro.^2;
moments = [-sum([state.copter.motors.moment_roll]);sum([state.copter.motors.moment_pitch]);sum([state.copter.motors.moment_yaw])] - rotational_drag;
state.moments = moments;
state = update_dynamics(state,force,moments);
end

function state = update_dynamics(state,force,moments)
rot_accel = (moments' / state.copter.inertia)';
state.gyro = state.gyro + rot_accel * state.delta_t;
state.gyro = max(state.gyro,deg2rad(-2000));
state.gyro = min(state.gyro,deg2rad(2000));
[state.dcm, state.attitude] = rotate_dcm(state.dcm,state.gyro * state.delta_t);
state.accel = force / state.copter.mass;
accel_ef = state.dcm * state.accel;
accel_ef(3) = accel_ef(3) + state.gravity_mss;
if state.position(3) >= 0 && accel_ef(3) > 0
    accel_ef(3) = 0;
end
state.accel = state.dcm' * (accel_ef + [0; 0; -state.gravity_mss]);
state.velocity = state.velocity + accel_ef * state.delta_t;
state.position = state.position + state.velocity * state.delta_t;
if state.position(3) >= 0
    state.position(3) = 0;
    state.velocity = [0;0;0];
    state.gyro = [0;0;0];
end
state.bf_velo = state.dcm' * state.velocity;
end

function [dcm, euler] = rotate_dcm(dcm, ang)
delta = [dcm(1,2) * ang(3) - dcm(1,3) * ang(2),         dcm(1,3) * ang(1) - dcm(1,1) * ang(3),      dcm(1,1) * ang(2) - dcm(1,2) * ang(1);
         dcm(2,2) * ang(3) - dcm(2,3) * ang(2),         dcm(2,3) * ang(1) - dcm(2,1) * ang(3),      dcm(2,1) * ang(2) - dcm(2,2) * ang(1);
         dcm(3,2) * ang(3) - dcm(3,3) * ang(2),         dcm(3,3) * ang(1) - dcm(3,1) * ang(3),      dcm(3,1) * ang(2) - dcm(3,2) * ang(1)];
dcm = dcm + delta;
a = dcm(1,:); b = dcm(2,:);
error = a * b';
t0 = a - (b *(0.5 * error));
t1 = b - (a *(0.5 * error));
t2 = cross(t0,t1);
dcm(1,:) = t0 * (1/norm(t0));
dcm(2,:) = t1 * (1/norm(t1));
dcm(3,:) = t2 * (1/norm(t2));
euler = [atan2(dcm(3,2),dcm(3,3)); -asin(dcm(3,1)); atan2(dcm(2,1),dcm(1,1))];
end
