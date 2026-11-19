# HIL Simulation Runbook

2026-11-10 · Hardware-in-the-loop test procedure for the flight controller under test.
Online copy (with the architecture diagram): https://claude.ai/artifact/3t96mysamp6muJYDbBfM8Z

## 1. Purpose and test levels

This runbook takes the flight controller under test through hardware-in-the-loop (HIL) testing in two levels that can be run today, plus one experimental level. Each level puts more of the real board in the loop than the PC simulation (SITL) already done.

| Level | Real | Simulated | What it proves | Time | Status |
| --- | --- | --- | --- | --- | --- |
| 1. Real-board I/O | Board, firmware, IMUs, barometer, RC link, motor outputs, ESCs | Nothing: the board sits on the bench, props off | Sensors, stick-to-motor mixing and direction, latency, tilt correction, RC failsafe, on the real hardware | 45 min | Scripts tested against ArduCopter SITL; ready for the board |
| 2. Simulation on Hardware | Board processor running the full flight code and EKF3 in real time | IMU, GPS, compass and the airframe (this drone's model file) | The board flies the same mission as the PC reference with the same tune; timing, CPU load and flight metrics match | 2–3 h first time, 30 min after | PC reference tested; firmware build and board run not yet done |
| 3. MATLAB physics over PPP | Board processor and flight code | MATLAB 6-DOF model over a serial network link | The validated MATLAB model in the loop with the real board | Days | Experimental, not built |

Level 2 compares the board with a PC reference run that uses the same firmware tune, the same physics model and the same scripted mission. Any difference therefore comes from the flight code running on the flight controller's processor.

Architecture (drawn in the online copy):

- Level 1: PC (Python pilot + MATLAB) <-> USB MAVLink, 100 Hz <-> board under test, all real (IMUs, barometer, RC link, ArduCopter 4.6.3) -> real motor outputs -> ESCs and motors (props removed). You tilt the board and switch the radio off to test correction and failsafe.
- Level 2: PC (mission pilot + MATLAB) <-> USB MAVLink + board log <-> board under test with HIL firmware: flight code + EKF3 on the H743 in real time <-> simulated sensors + vehicle.json airframe. A PC reference (ArduCopter SITL with the same tune, mission and vehicle.json) provides the reference log for the comparison.

## 2. Safety rules

All propellers stay off for every HIL level; Level 1 spins the real motors.

1. Remove all four propellers before connecting the battery. Check twice.
2. Clamp or tape the drone to the bench so motor torque cannot move it.
3. Keep the transmitter on and in reach; the arm switch (CH5) and kill switch (CH9) still work during Level 1.
4. Back up all parameters before Level 2 (Section 5, step 3). The Sim-on-Hardware firmware replaces the flight firmware and its parameters.
5. Leave SIM_OH_MASK at 0 in Level 2, so the simulated flight never drives the real motors.
6. Never fly the drone while it carries HIL firmware or HIL parameters. Restore it first (Section 7).
7. If anything behaves unexpectedly, unplug the battery. The test scripts always try to disarm on exit, including an emergency motor stop if a normal disarm is refused.

## 3. Setting up a new PC

Everything needed lives in one folder, `Documents\MATLAB\fc_test`, including the flight log, the PC simulator and all scripts. Copy that folder, install four programs, and run one check.

| Software | Version used | Needed for | Install |
| --- | --- | --- | --- |
| MATLAB with UAV Toolbox, Aerospace Blockset, Control System Toolbox | R2026a | All analysis | MathWorks installer |
| Python 3 with pymavlink and pyserial | 3.11, pymavlink 2.4.49 | Test pilots, parameter tool | python.org, then `pip install pymavlink pyserial` |
| Mission Planner | Latest | Flashing firmware, downloading logs | ardupilot.org |
| WSL with Ubuntu 24.04 (or 22.04) | – | Level 2 firmware build only | `wsl --install -d Ubuntu-24.04` in an admin PowerShell, then reboot |

**Steps**

1. Get the folder onto the PC. If both PCs use the same OneDrive account it syncs by itself. Otherwise copy the whole `fc_test` folder to `Documents\MATLAB\fc_test`.
2. Install the software above.
3. Plug in the flight controller by USB. Note its COM port in Device Manager (Ports). This runbook writes `COM8`; use yours.
4. In MATLAB:

```matlab
cd(fullfile(userpath, "fc_test", "hil"))
hil_check_setup
```

Every item except WSL must say OK before Level 1. WSL is needed only for the Level 2 firmware build.

**Folder contents (fc_test\hil)**

| File | Purpose |
| --- | --- |
| hil_check_setup.m | Checks software, files and serial ports |
| hil_l1_io_test.py, hil_l1_analysis.m | Level 1 test and its MATLAB report |
| hil_params.py | Back up, load and verify parameters over USB |
| hil_l2_make_params.m | Builds Level 2 parameter files from the flight log plus Tune C |
| hil_l2_run.m | Flies the scripted mission on the PC reference or on the board |
| hil_l2_compare.m | Board vs PC report: table and figure |
| l2\build_sim_on_hw.sh, l2\vehicle.json | Firmware build script and this drone's airframe model |
| data\flight_2026-11-09.bin | Real flight log the tune comes from |
| results\ | Every run's output; L2_sitl_reference_example is a finished PC reference |

## 4. Level 1: real-board I/O test

The real board, with its normal flight firmware and real sensors, is commanded over USB while its motor outputs are recorded at 100 Hz; the run takes about 3 minutes plus the hand tests.

**Before you start:** props off (Section 2), battery connected, transmitter on with the arm switch in the disarmed position, Mission Planner disconnected from the COM port.

**Run it** (Windows PowerShell, in `fc_test\hil`):

```powershell
python hil_l1_io_test.py COM8
python hil_l1_io_test.py COM8 --arm --tilt --rc-failsafe
```

The first command only records 10 s at rest and never arms; use it to check the link. The second runs everything:

1. Reads the firmware version and 16 key parameters (frame, motor outputs, RC2_REVERSED, arming checks, failsafe).
2. Records 10 s at rest: attitude and gyro noise, CPU load, vibration.
3. Asks you to type `PROPS OFF`, then arms in STABILIZE through RC override and holds 1300 µs throttle.
4. Sends six stick steps (roll, pitch and yaw, both ways). For each it checks that the correct motor pair speeds up and measures stick-to-motor latency.
5. Tilt test: you tilt the board right-side-down, then nose-down, about 20°. The motors on the low side must speed up.
6. RC failsafe test: throttle low, then switch the transmitter off. The board must disarm or change mode within 15 s. Switch it back on when asked.
7. Disarms and releases RC override, also on Ctrl+C or error.

**Analyse** (MATLAB, in `fc_test\hil`):

```matlab
hil_l1_analysis()      % newest results\L1_* folder
```

It prints a pass/fail table and saves `l1_report.png` and `l1_results.csv` in the results folder.

| Check | Pass criterion |
| --- | --- |
| Attitude noise at rest (roll, pitch) | < 0.1° std |
| Gyro noise at rest (p, q, r) | < 0.5 °/s std |
| CPU load at rest | < 70 % |
| Vibration at rest | < 3 m/s² per axis |
| RC2_REVERSED | 1 (this drone's pitch channel) |
| Motor outputs 1–4 | SERVO1–4_FUNCTION = 33, 34, 35, 36 |
| Roll right / left | Left motors (M2, M3) / right motors (M1, M4) speed up by > 8 µs |
| Pitch forward / back | Rear motors (M2, M4) / front motors (M1, M3) speed up |
| Yaw right / left | Counter-clockwise motors (M1, M2) / clockwise motors (M3, M4) speed up |
| Stick-to-motor latency, median | < 50 ms, including USB and the 100 Hz telemetry stream |
| Hand tilt right side down / nose down | > 8° tilt seen, and right motors / front motors speed up |
| RC loss | Disarm or failsafe mode change within 15 s |
| End of test | Board disarmed |

**Dry run without the board.** The script was verified against ArduCopter SITL on the PC: all six stick steps passed with the correct motor pairs, and the forced disarm on exit worked. To repeat it, start SITL as in Section 8 and run `python hil_l1_io_test.py tcp:127.0.0.1:5760 --arm --yes`.

## 5. Level 2: Simulation on Hardware

The board runs a special ArduCopter build that simulates its own sensors and airframe and flies the full flight code, including EKF3, on the board's processor in real time. The same mission is flown on the PC reference, and the two logs are compared.

**Airframe model.** Both runs use ArduPilot's built-in multicopter physics loaded with `l2\vehicle.json`: 280 g, 220 mm X frame, 2S, hover throttle 0.50, thrust expo 0.49. ArduPilot estimates inertia from mass and size as 8.5 × 10⁻⁴ / 1.7 × 10⁻³ kg·m² (roll / yaw), close to the 7.2 × 10⁻⁴ / 1.7 × 10⁻³ identified from the real flight. The frame string must be `x`: in ArduPilot's simulator `quad` means a plus layout.

**Parameters.** The flight tune comes from the real flight log plus the recommended Tune C (yaw P 0.34, yaw I 0.034, pitch D 0.0034). Simulation settings follow ArduPilot's sitl-on-hardware defaults, with EKF3 running on the board.

### Steps

1. **Make the parameter files** (MATLAB, `fc_test\hil`):

```matlab
hil_l2_make_params()
```

2. **Fly the PC reference** (about 3 min; it must end with "Mission COMPLETE"):

```matlab
ref = hil_l2_run("sitl");
```

3. **Back up the board.** Close Mission Planner, then in PowerShell (`fc_test\hil`):

```powershell
python hil_params.py COM8 --backup data\board_backup_before_hil.parm
```

Also save the parameters in Mission Planner (Config > Full Parameter List > Save to file) as a second copy.

4. **Build the firmware** in WSL Ubuntu. The first build takes 45–60 min: it clones ArduPilot, installs the ARM toolchain (asks for the Ubuntu password) and compiles.

```bash
cd "/mnt/c/Users/$WINUSER/OneDrive/Documents/MATLAB/fc_test/hil/l2"
bash build_sim_on_hw.sh
```

Replace `$WINUSER` with the Windows user name, or use the actual path of the folder. The default ArduPilot version is Copter-4.7.3, matching the PC simulator. The result is `l2\SimOnHW_Copter-4.7.3.apj`. The board's normal firmware uses the ArduPilot target MicoAir743, so the build uses that target.

5. **Flash it.** Mission Planner > Setup > Install Firmware > Load custom firmware > choose the .apj. Wait for the reboot.
6. **Load the HIL parameters** (Mission Planner disconnected). Run it twice: some SIM parameters appear only after the first reboot.

```powershell
python hil_params.py COM8 --load l2\hil.parm --reboot
python hil_params.py COM8 --load l2\hil.parm --reboot
```

The second run must report no "NOT set" names.

7. **Check in Mission Planner.** Connect: the HUD should show a level attitude, a simulated GPS fix and the home position -35.363262, 149.165237. Then disconnect.
8. **Fly the mission on the board** (MATLAB):

```matlab
hil = hil_l2_run("COM8");
```

9. **Get the board's log.** Mission Planner > DataFlash Logs > Download DataFlash Log via Mavlink, or read the SD card's `APM\LOGS`. Copy the newest .BIN into the `hil.runDir` folder.
10. **Compare** (MATLAB):

```matlab
T = hil_l2_compare("<path to board .BIN>", ref.log);
```

This prints the table below and saves `l2_comparison.png` and `l2_comparison.csv`.

| Metric | Pass criterion |
| --- | --- |
| MATLAB controller replica error, roll / pitch / yaw | < 1 % of full scale on the board log |
| Roll/pitch step settling | Board within 150 ms of PC |
| Roll/pitch step overshoot | Within 5 percentage points |
| Roll/pitch tracking RMS | Within 0.5° |
| Yaw rate achieved / commanded | Within 10 percentage points |
| Altitude hold RMS | Within 0.1 m |
| Loiter position RMS | Within 0.3 m |
| CPU load, mean | < 80 % on the board |
| Long loops | < 5 % of loops |
| Maximum loop time | < 10 ms |

**PC reference result (already run).** Steps settle in 0.70–0.90 s with 0–0.7 % overshoot and 0.16–0.21° tracking error. Altitude hold is 0.057 m RMS, loiter 0.03 m RMS in calm air, and yaw reaches 80 % of the commanded rate. The MATLAB controller replica matches the firmware within 0.03–0.05 % of full scale. Full log: `results\L2_sitl_reference_example`.

## 6. Level 3 (experimental): MATLAB physics over PPP

Level 3 would put the validated MATLAB 6-DOF model in the loop with the real board. Nothing for it is built or tested; start it only after Levels 1 and 2 pass.

ArduPilot's Simulation on Hardware can take external physics through its JSON backend, but only over a network link. The board has no Ethernet. ArduPilot can run a network over a serial port with PPP on H7 boards, using a custom build.

| Piece | What is needed | Source |
| --- | --- | --- |
| Firmware | Sim-on-Hardware build with `--simclass JSON`, `define AP_SIM_JSON_ENABLED 1`, plus PPP (`--enable-PPP`) | ArduPilot Simulation on Hardware and Network pages |
| Board settings | `SERIAL1_PROTOCOL 48` (PPP) on a free UART, `NET_ENABLE 1`, static IP | ArduPilot Network page |
| Link | USB–UART adapter; 921600 baud without flow control (up to 12.5 Mbaud with flow control) | ArduPilot Network page |
| PC side | `pppd` in WSL with the adapter passed through (usbipd-win), and sitl_bridge.m listening on that network | To be built |

**Open risks**

- Bandwidth: each physics exchange is a few hundred bytes. At 400 Hz that is roughly 100–200 kB/s, more than 921600 baud (about 90 kB/s) carries. A flow-controlled UART at higher baud may be required.
- Real time: MATLAB must answer every board request within one loop (2.5 ms at 400 Hz). The current bridge runs at 92–96 % of real time on the PC SITL, which is not enough for a board that cannot wait.
- ArduPilot's documentation warns the board will not appear to boot when the external physics is unavailable.

Estimated effort: several days. Level 2 already exercises the board processor in real time with the drone's airframe model.

## 7. Restoring the flight firmware after Level 2

The drone must not fly until the normal firmware and the backed-up parameters are back, and the checks below pass.

1. Mission Planner > Setup > Install Firmware > choose the multirotor (Copter) icon. Mission Planner detects the board's target. Install ArduCopter 4.6.3, or the latest stable version.
2. Wipe the HIL parameters: Config > Full Parameter List > set `FORMAT_VERSION` to 0 > Write Params > reboot.
3. Load the backup (Mission Planner disconnected):

```powershell
python hil_params.py COM8 --load data\board_backup_before_hil.parm --reboot
```

Some names may report "NOT set" on the first pass; run it a second time after the reboot. Alternatively load the Mission Planner copy (Full Parameter List > Load from file > Write Params).

4. Redo the accelerometer calibration and the compass calibration (outdoors).
5. Radio check in Setup > Radio Calibration: pushing the pitch stick forward must lower CH2 (RC2_REVERSED = 1).
6. Check flight modes, arm switch (CH5), kill switch (CH9), RTL (CH11), battery failsafe and fence.
7. Run the Level 1 rest test (`python hil_l1_io_test.py COM8`) and `hil_l1_analysis()`. All rest and parameter checks must pass.
8. Only then refit the propellers for a short, low hover.

## 8. Results record and troubleshooting

Fill one row per run; the files named are what a reviewer needs to check it.

| Date | Level | Firmware | Checks passed | Result folder | Notes |
| --- | --- | --- | --- | --- | --- |
| 2026-11-10 | 1, SITL dry run | ArduCopter 4.7.3 SITL | 14 of 15 (RC2_REVERSED is 0 in SITL, as expected) | sitl\run_hil_dryrun\results | Scripts verified, not the board |
| 2026-11-10 | 2, PC reference | ArduCopter 4.7.3 SITL + vehicle.json | Mission complete | results\L2_sitl_reference_example | Reference for the board run |
|  | 1, board |  |  |  |  |
|  | 2, board |  |  |  |  |

**PC simulator for dry runs.** In PowerShell, from `fc_test\sitl\run_hil_dryrun` (a parameter file `dry.parm` is there):

```powershell
cmd /c "..\bin\ArduCopter.elf --model x --speedup 1 --defaults dry.parm --wipe -I0"
```

Leave it running and connect the script to `tcp:127.0.0.1:5760`.

**Troubleshooting**

| Symptom | Cause | Fix |
| --- | --- | --- |
| "No heartbeat" | Mission Planner or another program holds the COM port | Disconnect Mission Planner; check the port number |
| Arming refused: "Gyros inconsistent" or "Accels inconsistent" for the first 10–20 s | IMUs still warming up | Normal: the scripts retry for 15 s (Level 1) or 3 min (Level 2) |
| Arming refused: compass, GPS or EKF messages in Level 1 | Real pre-arm checks on the bench | Level 1 uses STABILIZE, which needs no GPS; fix the named problem (e.g. compass calibration). Never force-arm |
| Arming refused mentioning the arm switch | CH5 arm switch state | Put the arm switch in the disarmed position, then rerun |
| Level 2: "Potential Thrust Loss", one motor at maximum | Frame layout mismatch | Model string must be `x:vehicle.json` (PC) or `x:@ROMFS/models/vehicle.json` (board); `quad` is a plus frame |
| Level 2: arming refused "Battery low voltage" | Battery thresholds not loaded | Regenerate with `hil_l2_make_params` and reload; BATT_LOW_VOLT must be 0 |
| `sitl_analysis` error "never got airborne" | Mission did not take off | Read `test_pilot.txt` in the run folder for the arming or thrust message |
| `hil_params.py` reports "NOT set" | Parameter appears only after a reboot, or is read-only | Run the load again after the reboot |
| Build: "Tag not found" | Version name differs | `git tag -l 'Copter-4.*'` in `~/ardupilot`, then `bash build_sim_on_hw.sh <tag>` |
| Build fails at the ROMFS line or the model file is missing at boot | The ROMFS path handling was not verified in a build | Check the build log for vehicle.json; as a fallback, copy vehicle.json into `~/ardupilot/libraries/AP_HAL_ChibiOS/hwdef/MicoAir743/` and use `ROMFS models/vehicle.json libraries/AP_HAL_ChibiOS/hwdef/MicoAir743/vehicle.json` |
| Build: flash overflow | Firmware larger than 2 MB | Disable unused features with define lines in the extra hwdef, one at a time (ArduPilot's build options list names them), and note each change |

## 9. Instructions for Claude on the other PC

This section is written for a Claude Code session asked to run these tests. This file is `fc_test\hil\HIL_RUNBOOK.md`.

**Context.** The test vehicle is a custom STM32H743 flight controller on a 280 g, 220 mm, 2S quadcopter running ArduCopter 4.6.3 (ArduPilot target MicoAir743). It was already tested in MATLAB: a validated 6-DOF model, ArduCopter SITL on that model, a tune comparison (Tune C recommended), and a benchmark against public logs. Scripts for that work are in `fc_test\bench`.

**Order of work**

1. Run `hil_check_setup` in MATLAB. Report anything MISSING to the user and wait.
2. Level 1 (Section 4): ask the user for the COM port. Run the rest-only test first, then the full test. The user must type PROPS OFF themselves.
3. Run `hil_l1_analysis()`, show the table and the figure, and record the result in Section 8.
4. Level 2 (Section 5), in order. Steps 3 (backup), 4 (WSL build), 5 (flash) and 6 (load parameters) change the board or the PC: explain each and get the user's go-ahead before running it.
5. After Level 2, follow Section 7 to restore the board before any flight.

**Rules**

- Never pass `--yes` to `hil_l1_io_test.py` with a real board; it is for SITL dry runs only.
- Never force-arm, and never set SIM_OH_MASK above 0 unless the user confirms all props are off and asks for motor output.
- Never flash firmware or load parameters without a confirmed backup file.
- Installing WSL needs administrator rights and a reboot: the user runs it.
- Do not change tune values by hand; regenerate with `hil_l2_make_params` (options: `TuneC=false`, `EKF="sim"`).
- Report failures as measured, with the result folder; do not rerun until a test passes without saying so.

**Verified vs not yet verified (as of this runbook)**

| Item | Status |
| --- | --- |
| hil_l1_io_test.py and hil_l1_analysis.m | Verified against ArduCopter 4.7.3 SITL; not yet run on the board |
| hil_params.py backup | Verified on SITL (1351 of 1351 parameters) |
| hil_l2_make_params, hil_l2_run("sitl"), hil_l2_compare | Verified end to end on the PC reference |
| build_sim_on_hw.sh | Written from ArduPilot's sitl-on-hw.py and hwdef sources; not yet run (no WSL on the first PC) |
| Board running Sim-on-Hardware firmware | Not yet run |
