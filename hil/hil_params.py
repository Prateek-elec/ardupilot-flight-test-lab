"""Back up, load and verify ArduPilot parameters over MAVLink (USB or TCP).

  python hil_params.py COM8 --backup backup.parm              # save every parameter
  python hil_params.py COM8 --load file.parm [--reboot]       # set + verify each one
  python hil_params.py COM8 --backup b.parm --load f.parm --reboot

.parm format: one "NAME VALUE" (or "NAME,VALUE") per line, # starts a comment.
Close Mission Planner first: a COM port can only be open once.
"""
import argparse
import sys
import time

from pymavlink import mavutil


def connect(addr, baud):
    if addr.upper().startswith("COM") or addr.startswith("/dev/"):
        m = mavutil.mavserial(addr, baud=baud, source_system=255)
    else:
        m = mavutil.mavlink_connection(addr, source_system=255)
    if m.wait_heartbeat(timeout=30) is None:
        sys.exit("No heartbeat - is Mission Planner holding the port?")
    print(f"Connected to system {m.target_system} on {addr}", flush=True)
    return m


def backup(m, path):
    m.mav.param_request_list_send(m.target_system, m.target_component)
    got, total, t_last = {}, None, time.time()
    while time.time() - t_last < 5:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            if total and len(got) < total:      # ask again for missing ones
                for i in range(total):
                    if i not in got:
                        m.mav.param_request_read_send(m.target_system, m.target_component, b"", i)
            continue
        total = msg.param_count
        got[msg.param_index] = (msg.param_id, msg.param_value)
        t_last = time.time()
        if total and len(got) >= total:
            break
    with open(path, "w") as f:
        f.write(f"# parameter backup {time.strftime('%Y-%m-%d %H:%M:%S')}, {len(got)} of {total}\n")
        for name, val in sorted(got.values()):
            f.write(f"{name},{val:.7g}\n")
    print(f"Backed up {len(got)} of {total} parameters to {path}", flush=True)
    return total is not None and len(got) >= total


def read_parm(path):
    out = []
    for line in open(path):
        line = line.split("#")[0].strip().replace(",", " ")
        if not line:
            continue
        parts = line.split()
        if len(parts) >= 2:
            out.append((parts[0], float(parts[1])))
    return out


def load(m, path):
    bad, ok = [], 0
    for name, val in read_parm(path):
        done = False
        for _ in range(3):
            m.mav.param_set_send(m.target_system, m.target_component, name.encode(), val,
                                 mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t_end = time.time() + 1.5
            while time.time() < t_end:
                msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is not None and msg.param_id == name:
                    done = abs(msg.param_value - val) <= 1e-4 * max(1.0, abs(val))
                    break
            if done:
                break
        if done:
            ok += 1
        else:
            bad.append(name)
    print(f"Set {ok} parameters from {path}", flush=True)
    if bad:
        print(f"NOT set ({len(bad)}): {' '.join(bad)}", flush=True)
        print("Unknown names usually mean the firmware lacks that feature (e.g. SIM_* on normal firmware),"
              " or the parameter appears only after a reboot (set again after rebooting).", flush=True)
    return bad


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("addr")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--backup")
    ap.add_argument("--load")
    ap.add_argument("--reboot", action="store_true")
    a = ap.parse_args()
    m = connect(a.addr, a.baud)
    if a.backup and not backup(m, a.backup):
        sys.exit("Backup incomplete - not loading anything. Retry.")
    bad = load(m, a.load) if a.load else []
    if a.reboot:
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
        print("Reboot sent - wait 10 s before reconnecting.", flush=True)
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
