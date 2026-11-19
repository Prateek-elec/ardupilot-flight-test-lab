"""Your FC HIL Level 1 - real-board I/O test (real firmware, real sensors, PROPS OFF).

What it does (MAVLink over USB, no Mission Planner needed):
  1. Reads board info and key parameters.
  2. Records 10 s at rest (attitude noise, CPU load, vibration).
  3. Optional (--arm): arms in STABILIZE with RC override, holds 1300 us throttle and
     sends roll / pitch / yaw stick steps. Checks that each motor moves the right way
     (ArduPilot Quad-X numbering) and measures stick-to-motor latency.
  4. Optional (--tilt): you tilt the board by hand; checks the motors push back.
  5. Optional (--rc-failsafe): you switch the transmitter off; checks the failsafe.
  6. Always disarms and releases RC override on exit, also on Ctrl+C or error.

Results: hil/results/L1_<time>/samples.csv, events.csv, summary.json
Analyse in MATLAB:  hil_l1_analysis("<that folder>")

Usage:
  python hil_l1_io_test.py COM8                      # rest test only, motors never armed
  python hil_l1_io_test.py COM8 --arm --tilt         # full test (PROPS OFF!)
  python hil_l1_io_test.py tcp:127.0.0.1:5760 --arm  # dry run against SITL
"""
import argparse
import csv
import json
import os
import sys
import time
from datetime import datetime

from pymavlink import mavutil

MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "LOITER": 5, "RTL": 6, "LAND": 9}
MODE_NAMES = {v: k for k, v in MODES.items()}
# ArduPilot Quad-X: M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW
EXPECT = {
    # name: (rc channel values, differential function over motors 1..4, description)
    "ROLL right":    (dict(roll=1700), lambda m: (m[1] + m[2]) - (m[0] + m[3]), "left motors (M2,M3) up"),
    "ROLL left":     (dict(roll=1300), lambda m: (m[0] + m[3]) - (m[1] + m[2]), "right motors (M1,M4) up"),
    "PITCH forward": (dict(pitch=1300), lambda m: (m[1] + m[3]) - (m[0] + m[2]), "rear motors (M2,M4) up"),
    "PITCH back":    (dict(pitch=1700), lambda m: (m[0] + m[2]) - (m[1] + m[3]), "front motors (M1,M3) up"),
    "YAW right":     (dict(yaw=1700), lambda m: (m[0] + m[1]) - (m[2] + m[3]), "CCW motors (M1,M2) up"),
    "YAW left":      (dict(yaw=1300), lambda m: (m[2] + m[3]) - (m[0] + m[1]), "CW motors (M3,M4) up"),
}
PARAMS = ["FRAME_CLASS", "FRAME_TYPE", "RC2_REVERSED", "ARMING_CHECK", "MOT_PWM_TYPE", "MOT_SPIN_ARM",
          "MOT_SPIN_MIN", "SERVO1_FUNCTION", "SERVO2_FUNCTION", "SERVO3_FUNCTION", "SERVO4_FUNCTION",
          "FS_THR_ENABLE", "RC_OVERRIDE_TIME", "INS_GYRO_FILTER", "SCHED_LOOP_RATE", "LOG_BITMASK"]


class Rig:
    def __init__(self, addr, baud):
        if addr.upper().startswith("COM") or addr.startswith("/dev/"):
            self.m = mavutil.mavserial(addr, baud=baud, source_system=255)
        else:
            self.m = mavutil.mavlink_connection(addr, source_system=255)
        self.t0 = time.time()
        self.s = dict(t_boot=0.0, armed=False, mode=-1, srv=[0] * 8, att=[0.0] * 6, rc=[0] * 8,
                      load=0.0, vib=[0.0] * 3, clip=[0] * 3, volt=0.0)
        self.rows, self.events, self.phase = [], [], "connect"
        self.override = None          # last RC override sent (tuple of 8)

    def now(self):
        return time.time() - self.t0

    def event(self, text):
        self.events.append((round(self.now(), 4), self.phase, text))
        print(f"[{self.now():7.2f}s {self.phase:>10}] {text}", flush=True)

    def set_rate(self, msg_id, hz):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                     msg_id, int(1e6 / hz), 0, 0, 0, 0, 0)

    def pump(self, dur):
        """Read messages for dur seconds, keep RC override alive, record a row per servo sample."""
        t_end = time.time() + dur
        last_rc = 0.0
        while time.time() < t_end:
            if self.override is not None and time.time() - last_rc > 0.05:
                self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, *self.override)
                last_rc = time.time()
            msg = self.m.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            typ = msg.get_type()
            s = self.s
            if typ == "HEARTBEAT" and msg.get_srcComponent() == 1:
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                if armed != s["armed"]:
                    self.event("ARMED" if armed else "DISARMED")
                if msg.custom_mode != s["mode"]:
                    self.event(f"mode {MODE_NAMES.get(msg.custom_mode, msg.custom_mode)}")
                s["armed"], s["mode"] = armed, msg.custom_mode
            elif typ == "ATTITUDE":
                s["t_boot"] = msg.time_boot_ms / 1000.0
                s["att"] = [msg.roll, msg.pitch, msg.yaw, msg.rollspeed, msg.pitchspeed, msg.yawspeed]
            elif typ == "RC_CHANNELS":
                s["rc"] = [msg.chan1_raw, msg.chan2_raw, msg.chan3_raw, msg.chan4_raw,
                           msg.chan5_raw, msg.chan6_raw, msg.chan7_raw, msg.chan8_raw]
            elif typ == "SYS_STATUS":
                s["load"] = msg.load / 10.0
                s["volt"] = msg.voltage_battery / 1000.0
            elif typ == "VIBRATION":
                s["vib"] = [msg.vibration_x, msg.vibration_y, msg.vibration_z]
                s["clip"] = [msg.clipping_0, msg.clipping_1, msg.clipping_2]
            elif typ == "STATUSTEXT":
                self.event(f"FC: {msg.text}")
            elif typ == "COMMAND_ACK" and msg.result != 0 and msg.command != 511:
                self.event(f"FC rejected command {msg.command} (result {msg.result})")
            elif typ == "SERVO_OUTPUT_RAW":
                s["srv"] = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw, msg.servo4_raw,
                            msg.servo5_raw, msg.servo6_raw, msg.servo7_raw, msg.servo8_raw]
                ov = self.override or (0,) * 8
                self.rows.append([round(self.now(), 4), round(s["t_boot"], 3), self.phase, int(s["armed"]), s["mode"],
                                  *s["srv"][:4], *[round(x, 5) for x in s["att"]], *s["rc"][:5], *ov[:4],
                                  s["load"], round(s["volt"], 2), *[round(v, 3) for v in s["vib"]], *s["clip"]])

    def rc(self, roll=1500, pitch=1500, thr=1000, yaw=1500):
        # CH5 (arm switch on this drone) is released (0) so the radio keeps its own value
        self.override = (roll, pitch, thr, yaw, 0, 0, 0, 0)

    def release(self):
        self.override = None
        for _ in range(3):
            self.m.mav.rc_channels_override_send(self.m.target_system, self.m.target_component, 0, 0, 0, 0, 0, 0, 0, 0)
            time.sleep(0.05)

    def get_params(self, names):
        out = {}
        for n in names:
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, n.encode(), -1)
            msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
            while msg is not None and msg.param_id != n:
                msg = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1.5)
            out[n] = None if msg is None else round(msg.param_value, 6)
        return out

    def set_mode(self, name):
        self.m.mav.set_mode_send(self.m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODES[name])
        self.pump(1.0)
        return self.s["mode"] == MODES[name]

    def arm(self, on, force=False):
        # force=True only for DISARM on exit: 21196 is ArduPilot's emergency-stop magic number
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1 if on else 0,
                                     21196 if (force and not on) else 0, 0, 0, 0, 0, 0)
        self.pump(2.0)
        return self.s["armed"] == on


def motor_mean(rows, t_from, t_to):
    sel = [r for r in rows if t_from <= r[0] <= t_to]
    if not sel:
        return None
    return [sum(r[5 + k] for r in sel) / len(sel) for k in range(4)]


def latency(rows, t_cmd, fdiff, base, thresh=8.0):
    for r in rows:
        if r[0] >= t_cmd and fdiff([r[5 + k] for k in range(4)]) - fdiff(base) > thresh:
            return round((r[0] - t_cmd) * 1000.0, 1)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("addr", help="COM8, /dev/ttyACM0 or tcp:127.0.0.1:5760 (SITL dry run)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--arm", action="store_true", help="arm and spin motors (PROPS OFF)")
    ap.add_argument("--tilt", action="store_true", help="interactive hand-tilt test (needs --arm)")
    ap.add_argument("--rc-failsafe", action="store_true", help="interactive transmitter-off test (needs --arm)")
    ap.add_argument("--yes", action="store_true", help="skip the props-off confirmation (SITL only)")
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "results"))
    a = ap.parse_args()

    out = os.path.join(a.out, "L1_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(out, exist_ok=True)
    rig = Rig(a.addr, a.baud)
    summary = {"addr": a.addr, "started": datetime.now().isoformat(timespec="seconds"), "steps": {}, "tilt": {}}
    try:
        rig.event(f"connecting to {a.addr}")
        hb = rig.m.wait_heartbeat(timeout=30)
        if hb is None:
            sys.exit("No heartbeat - is Mission Planner holding the port? Close it and retry.")
        rig.event(f"heartbeat from system {rig.m.target_system}, autopilot type {hb.autopilot}")
        for mid, hz in [(36, 100), (30, 100), (65, 25), (1, 5), (241, 5)]:   # servo, attitude, rc, sys_status, vibration
            rig.set_rate(mid, hz)
        rig.m.mav.command_long_send(rig.m.target_system, rig.m.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
        v = rig.m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
        if v is not None:
            fw = v.flight_sw_version
            summary["firmware"] = f"{fw >> 24 & 255}.{fw >> 16 & 255}.{fw >> 8 & 255}"
            summary["board_version"] = v.board_version
        summary["params"] = rig.get_params(PARAMS)
        rig.event(f"params {summary['params']}")

        rig.phase = "rest"
        rig.event("recording 10 s at rest - do not touch the board")
        rig.pump(10.0)

        if a.arm:
            if not a.yes:
                ans = input("\n*** Motors will spin. Type PROPS OFF to confirm all propellers are removed: ")
                if ans.strip().upper() != "PROPS OFF":
                    sys.exit("Not confirmed - stopping without arming.")
            rig.phase = "arm"
            rig.rc(thr=1000)
            rig.pump(0.5)
            if not rig.set_mode("STABILIZE"):
                rig.event("could not select STABILIZE")
            ok = False
            for _ in range(5):
                if rig.arm(True):
                    ok = True
                    break
                rig.pump(1.0)
            summary["armed_ok"] = ok
            if not ok:
                rig.event("ARMING FAILED - see FC messages above (pre-arm checks). No motor tests run.")
            else:
                rig.phase = "baseline"
                rig.rc(thr=1300)
                rig.pump(3.0)
                for name, (ch, fdiff, desc) in EXPECT.items():
                    rig.phase = "base:" + name
                    rig.rc(thr=1300)
                    t_b = rig.now()
                    rig.pump(1.0)
                    base = motor_mean(rig.rows, t_b + 0.3, rig.now())
                    rig.phase = "step:" + name
                    t_cmd = rig.now()
                    rig.event(f"STEP {name}: expect {desc}")
                    rig.rc(thr=1300, **ch)
                    rig.pump(0.6)
                    resp = motor_mean(rig.rows, t_cmd + 0.15, rig.now())
                    rig.rc(thr=1300)
                    rig.phase = "settle"
                    rig.pump(1.5)
                    if base is None or resp is None:
                        summary["steps"][name] = {"result": "NO DATA"}
                        continue
                    d = fdiff(resp) - fdiff(base)
                    lat = latency(rig.rows, t_cmd, fdiff, base)
                    res = "PASS" if d > 8 else ("FAIL: wrong direction" if d < -8 else "FAIL: no response")
                    summary["steps"][name] = {"name": name, "expected": desc, "differential_us": round(d, 1),
                                              "latency_ms": lat, "result": res}
                    rig.event(f"  {name}: differential {d:+.0f} us, latency {lat} ms -> {res}")

                if a.tilt:
                    for name, axis, sign, fdiff, desc in [
                            ("right side down", 0, +1, lambda m: (m[0] + m[3]) - (m[1] + m[2]), "right motors (M1,M4) up"),
                            ("nose down", 1, -1, lambda m: (m[0] + m[2]) - (m[1] + m[3]), "front motors (M1,M3) up")]:
                        rig.phase = "tilt:" + name
                        rig.rc(thr=1300)
                        t_b = rig.now()
                        rig.pump(1.0)
                        base = motor_mean(rig.rows, t_b + 0.3, rig.now())
                        input(f"\nTILT: hold the board level, then tilt it {name} about 20 deg and hold. "
                              f"Press Enter as you start tilting ... ")
                        t_s = rig.now()
                        rig.pump(4.0)
                        sel = [r for r in rig.rows if r[0] >= t_s + 1.0]
                        ang = max((sign * r[9 + axis] * 57.2958 for r in sel), default=0.0)
                        resp = motor_mean(rig.rows, t_s + 1.0, rig.now())
                        d = (fdiff(resp) - fdiff(base)) if (resp and base) else 0.0
                        res = "PASS" if (ang > 8 and d > 8) else ("NOT TILTED" if ang <= 8 else "FAIL")
                        summary["tilt"][name] = {"max_tilt_deg": round(ang, 1), "differential_us": round(d, 1),
                                                 "expected": desc, "result": res}
                        rig.event(f"  tilt {name}: {ang:.0f} deg, differential {d:+.0f} us -> {res}")
                        input("Put the board back level and press Enter ... ")

                if a.rc_failsafe:
                    rig.phase = "rc_failsafe"
                    rig.rc(thr=1000)
                    rig.pump(1.0)
                    rig.release()
                    input("\nRC FAILSAFE: set the transmitter throttle LOW, then press Enter and switch the "
                          "transmitter OFF within 5 s ... ")
                    t_s = rig.now()
                    rig.pump(15.0)
                    got = [e for e in rig.events if e[0] >= t_s and ("DISARMED" in e[2] or "ailsafe" in e[2]
                                                                       or e[2].startswith("mode"))]
                    summary["rc_failsafe"] = {"events": [e[2] for e in got],
                                              "result": "PASS" if got else "FAIL: no failsafe reaction in 15 s"}
                    rig.event(f"  rc failsafe -> {summary['rc_failsafe']['result']}")
                    input("Switch the transmitter back ON and press Enter ... ")
    except KeyboardInterrupt:
        rig.event("stopped by user")
    finally:
        rig.phase = "disarm"
        try:
            rig.rc(thr=1000)
            rig.pump(0.3)
            if rig.s["armed"] and not rig.arm(False):
                rig.event("normal disarm refused - sending emergency motor stop")
                rig.arm(False, force=True)
            rig.release()
            rig.pump(0.5)
        except Exception as e:      # never leave the board armed silently
            print(f"WARNING: could not confirm disarm ({e}) - unplug the battery.", flush=True)
        hdr = ["t_host", "t_boot", "phase", "armed", "mode", "m1", "m2", "m3", "m4",
               "roll", "pitch", "yaw", "p", "q", "r", "rc1", "rc2", "rc3", "rc4", "rc5",
               "ov1", "ov2", "ov3", "ov4", "load_pct", "volt", "vibx", "viby", "vibz", "clip0", "clip1", "clip2"]
        with open(os.path.join(out, "samples.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            w.writerows(rig.rows)
        with open(os.path.join(out, "events.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["t_host", "phase", "text"])
            w.writerows(rig.events)
        summary["samples"] = len(rig.rows)
        summary["still_armed_at_exit"] = rig.s["armed"]
        with open(os.path.join(out, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\nResults: {out}\nMATLAB:  hil_l1_analysis(\"{out}\")", flush=True)
        if rig.s["armed"]:
            print("WARNING: board still reports ARMED - disarm with the transmitter or unplug the battery.")


if __name__ == "__main__":
    main()
