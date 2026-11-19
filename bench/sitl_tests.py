"""Scripted test flight for ArduCopter SITL (flying the MATLAB physics).

Sequence (simulated time):
  wait for EKF/GPS -> GUIDED arm -> take off to 5 m -> hover
  ALT_HOLD stick steps: roll +/-, pitch +/-, yaw, throttle climb
  LOITER position hold in wind -> RTL -> land and disarm

Usage: python sitl_tests.py [tcp:127.0.0.1:5760]
       python sitl_tests.py COM8        (HIL: Simulation-on-Hardware firmware on the real board)
"""
import sys
import time

from pymavlink import mavutil

ADDR = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760"
MODES = {"STABILIZE": 0, "ALT_HOLD": 2, "GUIDED": 4, "LOITER": 5, "RTL": 6, "LAND": 9}


def log(msg):
    print(f"[test {sim_t():7.1f}s] {msg}", flush=True)


_state = {"t_boot": 0.0, "alt": 0.0, "armed": False, "mode": None}


def sim_t():
    return _state["t_boot"]


def pump(m, dur=0.0, until=None):
    """Process messages for `dur` simulated seconds (or until predicate true)."""
    t_end = sim_t() + dur
    wall_end = time.time() + (600.0 if (until is not None and dur == 0) else max(60.0, dur * 20))
    while True:
        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is not None:
            typ = msg.get_type()
            if typ == "SYSTEM_TIME" or typ == "ATTITUDE":
                _state["t_boot"] = msg.time_boot_ms / 1000.0
            elif typ == "GLOBAL_POSITION_INT":
                _state["t_boot"] = msg.time_boot_ms / 1000.0
                _state["alt"] = msg.relative_alt / 1000.0
            elif typ == "HEARTBEAT" and msg.get_srcComponent() == 1:
                _state["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                _state["mode"] = msg.custom_mode
            elif typ == "STATUSTEXT":
                log(f"FC: {msg.text}")
            elif typ == "COMMAND_ACK" and msg.result != 0:
                log(f"FC rejected command {msg.command} (result {msg.result})")
        if until is not None and until():
            return True
        if dur > 0 and sim_t() >= t_end:
            return until is None
        if time.time() > wall_end:
            return False


def set_mode(m, name):
    m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, MODES[name])
    ok = pump(m, until=lambda: _state["mode"] == MODES[name])
    log(f"mode {name}" + ("" if ok else " (not confirmed)"))


def rc(m, roll=1500, pitch=1500, thr=1500, yaw=1500):
    # CH5 (flight-mode switch) is held at a fixed value so it never changes mode
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    roll, pitch, thr, yaw, 1000, 0, 0, 0)


def hold_rc(m, dur, **ch):
    t_end = sim_t() + dur
    while sim_t() < t_end:
        rc(m, **ch)
        pump(m, 0.05)


def main():
    log(f"connecting to {ADDR}")
    m = None
    for _ in range(120):
        try:
            if ADDR.upper().startswith("COM") or ADDR.startswith("/dev/"):   # HIL: real board on USB
                m = mavutil.mavserial(ADDR, baud=115200, source_system=255)
            else:
                m = mavutil.mavlink_connection(ADDR, source_system=255)
            break
        except OSError:
            time.sleep(1)
    if m is None:
        sys.exit("could not connect to SITL")
    m.wait_heartbeat(timeout=120)
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)
    log("connected, waiting for EKF and GPS")

    # Fly like a pilot: ALT_HOLD, throttle down, arm, throttle up to climb.
    hold_rc(m, 0.5, thr=1000)
    set_mode(m, "ALT_HOLD")
    armed = False
    for attempt in range(60):
        rc(m, thr=1000)
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        t_try = sim_t() + 3.0
        while sim_t() < t_try and not _state["armed"]:
            rc(m, thr=1000)
            pump(m, 0.05)
        if _state["armed"]:
            armed = True
            break
    if not armed:
        sys.exit("could not arm")
    log("ARMED in ALT_HOLD - climbing to 5 m with throttle 1750")
    t_lim = sim_t() + 30
    while _state["alt"] < 4.7 and sim_t() < t_lim:
        rc(m, thr=1750)
        pump(m, 0.05)
    log(f"altitude {_state['alt']:.2f} m - throttle centre, hover 5 s")
    hold_rc(m, 5)
    steps = [("ROLL right", dict(roll=1700)), ("ROLL left", dict(roll=1300)),
             ("PITCH forward", dict(pitch=1300)), ("PITCH back", dict(pitch=1700))]
    for name, ch in steps:
        log(f"STEP {name} (stick 200 us = 40 % for 1.0 s)")
        hold_rc(m, 1.0, **ch)
        hold_rc(m, 4.0)
    log("STEP YAW right (stick 200 us for 2.0 s)")
    hold_rc(m, 2.0, yaw=1700)
    hold_rc(m, 4.0)
    log("STEP CLIMB (throttle 1750 for 2.0 s)")
    hold_rc(m, 2.0, thr=1750)
    hold_rc(m, 5.0)
    log(f"altitude {_state['alt']:.2f} m")

    set_mode(m, "LOITER")
    hold_rc(m, 10.0)
    log("LOITER hold done - RTL")
    set_mode(m, "RTL")
    t_lim = sim_t() + 120
    while _state["armed"] and sim_t() < t_lim:   # keep sticks centred, CH5 fixed
        rc(m)
        pump(m, 0.1)
    ok = not _state["armed"]
    log("LANDED and disarmed" if ok else "RTL did not finish in time")
    log("TESTS COMPLETE")


if __name__ == "__main__":
    main()
