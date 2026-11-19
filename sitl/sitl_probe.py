"""Connect to a running ArduCopter SITL on tcp:127.0.0.1:5760 and print version + status."""
import sys
import time

from pymavlink import mavutil

addr = sys.argv[1] if len(sys.argv) > 1 else "tcp:127.0.0.1:5760"
m = mavutil.mavlink_connection(addr, source_system=255)
hb = m.wait_heartbeat(timeout=30)
if hb is None:
    sys.exit("no heartbeat from SITL")
print(f"heartbeat: system {m.target_system}, type {hb.type}, autopilot {hb.autopilot}")
m.mav.command_long_send(m.target_system, m.target_component,
                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
t0 = time.time()
while time.time() - t0 < 15:
    msg = m.recv_match(type=["STATUSTEXT", "AUTOPILOT_VERSION"], blocking=True, timeout=2)
    if msg is None:
        continue
    if msg.get_type() == "AUTOPILOT_VERSION":
        v = msg.flight_sw_version
        print(f"firmware version {v >> 24 & 255}.{v >> 16 & 255}.{v >> 8 & 255} (type {v & 255})")
    else:
        print("text:", msg.text)
