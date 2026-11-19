"""MAVLink bridge: flight controller serial/USB port -> UDP for MATLAB and Mission Planner.

Use when Mission Planner's Mavlink Mirror does not work.

    1. In Mission Planner click DISCONNECT (the COM port can only be open once).
    2. Run:  python fc_bridge.py COM8
    3. MATLAB:  fc_live_test()                         (listens on UDP 14560)
       Mission Planner (optional): choose "UDP", click Connect, port 14550.

Reconnects automatically if the USB cable is unplugged or the board reboots.
Stop with Ctrl+C.
"""
import argparse
import socket
import time

import serial
from pymavlink import mavutil


def open_fc(args):
    """Open the serial port and wait for an ArduPilot heartbeat; retry until it works."""
    while True:
        try:
            fc = mavutil.mavserial(args.port, baud=args.baud, source_system=254)
            fc.wait_heartbeat(timeout=10)
            if fc.target_system:
                print(f"FC connected on {args.port}: system {fc.target_system}.", flush=True)
                return fc
            fc.close()
            print(f"No heartbeat on {args.port} (is Mission Planner using it?). Retrying ...", flush=True)
        except (serial.SerialException, OSError):
            pass
        time.sleep(2)


def forward(args, fc, sock, targets):
    """Forward until the serial link fails (raises)."""
    last_req = 0.0
    count = 0
    while True:
        now = time.time()
        if now - last_req > 5:      # keep data streams on even when no GCS asks for them
            fc.mav.request_data_stream_send(fc.target_system, fc.target_component,
                                            mavutil.mavlink.MAV_DATA_STREAM_ALL, args.rate, 1)
            last_req = now

        msg = fc.recv_match(blocking=True, timeout=0.05)
        if msg is not None and msg.get_type() != "BAD_DATA":
            buf = msg.get_msgbuf()
            for t in targets:
                try:
                    sock.sendto(buf, t)
                except OSError:
                    pass
            count += 1
            if count % 2000 == 0:
                print(f"  forwarded {count} messages", flush=True)

        try:                        # Mission Planner -> FC (params, commands)
            while True:
                data, _ = sock.recvfrom(4096)
                fc.write(data)
        except (BlockingIOError, ConnectionResetError):
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("port", help="FC serial port, e.g. COM8")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--matlab", type=int, default=14560, help="UDP port MATLAB listens on")
    ap.add_argument("--mp", type=int, default=14550, help="UDP port Mission Planner listens on")
    ap.add_argument("--rate", type=int, default=10, help="stream rate to request from the FC (Hz)")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    targets = [("127.0.0.1", args.matlab), ("127.0.0.1", args.mp)]
    print(f"Bridge {args.port} -> UDP {args.matlab} (MATLAB), {args.mp} (Mission Planner). Ctrl+C to stop.", flush=True)

    while True:
        fc = open_fc(args)
        try:
            forward(args, fc, sock, targets)
        except (serial.SerialException, OSError) as e:
            print(f"Lost {args.port} ({e.__class__.__name__}). Waiting for the board ...", flush=True)
            try:
                fc.close()
            except Exception:
                pass
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("Bridge stopped.")
