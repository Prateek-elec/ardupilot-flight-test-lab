function stopFcn = fc_fake_drone(opts)
%FC_FAKE_DRONE Send simulated ArduPilot MAVLink data to a UDP port.
%   Use it to try fc_live_test without hardware:
%       stopFake = fc_fake_drone();
%       fc_live_test(Interactive=false);
%       stopFake();
%
%   Inject faults to see FAIL rows, e.g.:
%       stopFake = fc_fake_drone(CurrentA=13.6, Sats=5, PreArmText="PreArm: Compass not calibrated");

arguments
    opts.Port (1,1) double = 14560
    opts.CurrentA (1,1) double = 0.3
    opts.Sats (1,1) double = 14
    opts.PreArmText (1,1) string = ""
end

dialect = mavlinkdialect("ardupilotmega.xml");
mav = mavlinkio(dialect, SystemID=1, ComponentID=1);
connect(mav, "UDP");

dt = 0.04;   % 25 Hz
k = 0;
tmr = timer(ExecutionMode="fixedRate", Period=dt, BusyMode="drop", TimerFcn=@(~,~) tick());
start(tmr);
stopFcn = @stopAll;

    function tick()
        k = k + 1;
        tms = round(k * dt * 1000);
        if mod(k, 25) == 1
            send("HEARTBEAT", type=2, autopilot=3, base_mode=81, system_status=3, mavlink_version=3);
            bits = 1+2+4+8+32+65536+2097152+33554432+268435456;
            send("SYS_STATUS", onboard_control_sensors_present=bits, onboard_control_sensors_enabled=bits, ...
                 onboard_control_sensors_health=bits, voltage_battery=8070, ...
                 current_battery=round(opts.CurrentA*100), load=350, battery_remaining=90);
            send("POWER_STATUS", Vcc=5050 + round(10*randn), Vservo=0, flags=3);
            send("GPS_RAW_INT", time_usec=tms*1000, fix_type=ternary(opts.Sats >= 6, 3, 1), ...
                 satellites_visible=opts.Sats, eph=85, lat=298743327, lon=779017391, alt=300000);
            send("EKF_STATUS_REPORT", flags=831, velocity_variance=0.05, pos_horiz_variance=0.04, ...
                 pos_vert_variance=0.06, compass_variance=0.03);
        end
        if strlength(opts.PreArmText) > 0 && mod(k, 50) == 1
            send("STATUSTEXT", severity=3, text=char(opts.PreArmText));
        end
        send("ATTITUDE", time_boot_ms=tms, roll=0.005 + 0.0005*randn, pitch=-0.01 + 0.0005*randn, yaw=1.0 + 0.0005*randn);
        send("RAW_IMU", time_usec=tms*1000, xacc=round(5+3*randn), yacc=round(-3+3*randn), zacc=round(-1000+3*randn), ...
             xgyro=round(1+0.8*randn), ygyro=round(-1+0.8*randn), zgyro=round(0.8*randn), ...
             xmag=round(200+2*randn), ymag=round(50+2*randn), zmag=round(400+2*randn));
        send("SCALED_IMU2", time_boot_ms=tms, xacc=round(8+4*randn), yacc=round(-1+4*randn), zacc=round(-998+4*randn), ...
             xgyro=round(1.5*randn), ygyro=round(1.5*randn), zgyro=round(1.5*randn));
        send("SCALED_PRESSURE", time_boot_ms=tms, press_abs=1013.25 + 0.01*randn, temperature=3100);
        send("RC_CHANNELS", time_boot_ms=tms, chancount=16, chan1_raw=1500, chan2_raw=1500, chan3_raw=988, ...
             chan4_raw=1500, chan5_raw=1000, rssi=254);
        send("VIBRATION", time_usec=tms*1000, vibration_x=2.5+0.2*randn, vibration_y=3+0.2*randn, vibration_z=4+0.2*randn);
    end

    function send(name, varargin)
        % send("MSG_NAME", field1=value1, field2=value2, ...)
        msg = createmsg(dialect, name);
        for i = 1:2:numel(varargin)
            f = string(varargin{i});
            v = varargin{i+1};
            cls = class(msg.Payload.(f));
            if ischar(v)
                if ischar(msg.Payload.(f))
                    buf = repmat(char(0), size(msg.Payload.(f)));
                    n = min(numel(v), numel(buf));
                    buf(1:n) = v(1:n);
                else
                    buf = zeros(size(msg.Payload.(f)), cls);
                    n = min(numel(v), numel(buf));
                    buf(1:n) = cast(double(v(1:n)), cls);
                end
                msg.Payload.(f) = buf;
            else
                msg.Payload.(f)(:) = cast(v, cls);
            end
        end
        sendudpmsg(mav, msg, "127.0.0.1", opts.Port);
    end

    function stopAll()
        stop(tmr); delete(tmr);
        disconnect(mav);
    end
end

function out = ternary(c, a, b)
if c, out = a; else, out = b; end
end
