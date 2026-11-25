<div align="center">

<img src="docs/banner.png" alt="ArduPilot Flight Test Lab" width="100%"/>

**My drone crashed. The flight log knew why before I did.**

So I built a MATLAB lab that reads ArduPilot flight logs, finds the problem, identifies the airframe, designs better PID gains with LQR, proves them on real ArduCopter firmware (SITL) and on the real board (HIL), then writes the `.param` file to load.

![MATLAB](https://img.shields.io/badge/MATLAB-R2026a-E16737?style=for-the-badge)
![ArduPilot](https://img.shields.io/badge/ArduPilot-ArduCopter-0B8ED8?style=for-the-badge&logo=ardupilot&logoColor=white)
![Control](https://img.shields.io/badge/control-LQR%20%E2%86%92%20PID-8b5cf6?style=for-the-badge)
![HIL](https://img.shields.io/badge/testing-SITL%20%2B%20HIL-22c55e?style=for-the-badge)
![License](https://img.shields.io/badge/license-MIT-22c55e?style=for-the-badge)

**No MATLAB?** The log health check is also available as a free Python tool: [ArduPilot Log Doctor](https://github.com/Prateek-elec/ardupilot-log-doctor).

</div>

---

## Highlights

- 📜 **Flight log in, tuned drone out:** pick a DataFlash `.BIN`, press Tune, load the `.param` it writes.
- 🧪 **Real firmware, simulated physics:** ArduCopter SITL flies a validated MATLAB 6-DOF quad model.
- 🎯 **Control theory, not guesswork:** the rate loop is identified from 400 Hz flight data, then LQR (LQI) designs the PID gains with phase- and gain-margin checks.
- 🔌 **Tested on the real board:** two-level hardware-in-the-loop, from I/O checks to full simulation-on-hardware missions.
- 📊 **Benchmark your board:** compare noise and tracking against 11 commercial flight controllers using public flight logs.

## The story

1. 💥 **The crash:** a 2S quad went down mid-flight.
2. 🔍 **The log review** (`fc_log_analysis`, `logs_review`) showed vibration jumping **~500×** (0.04 → 19 m/s²) right before it. The cause was mechanical: motor screws. That became the [post-crash checklist](post_crash_checklist.md).
3. 🧠 **Same logs, new tune:** the rate loop was identified from 400 Hz flight data and LQR designed new PID gains with proper margins.
4. 🧪 **Proved before flying:** the new tune was flown on ArduCopter SITL with turbulence and vibration, then on the real board in HIL.
5. 📜 **One command for next time:** all of it is packaged so any ArduPilot log goes in and a tuned `.param` comes out.

## The pipeline

```text
 ArduPilot .BIN log
        │
        ▼
 fc_log_analysis ──────────── health report (pass / fail)
        │
        ▼
 fc_identify_rate / fc_simulation ── grey-box airframe model per axis (K, τ, trim)
        │
        ▼
 lqr_pid_tuner / fc_tune_from_log ── LQR-designed rate + angle gains, margin-checked
        │
        ├──► run_sitl_bench ───── real ArduCopter (SITL) flying the MATLAB 6-DOF model
        ├──► hil_l1 / hil_l2 ──── the same tests on your physical flight controller
        ▼
 .param file + PDF tuning report (fc_tune_report)
```

## Results from a real tuning campaign

<table>
<tr>
<td width="50%"><img src="bench/results/tune_comparison.png" alt="Tune comparison on ArduCopter SITL"/><br/><sub><b>Three tunes on SITL</b> with vibration and turbulence: step response, motor noise, hold accuracy</sub></td>
<td width="50%"><img src="fleet_results/fleet_identification.png" alt="System identification from flight data"/><br/><sub><b>System identification</b> from 400 Hz flight data, with rate-loop margins</sub></td>
</tr>
<tr>
<td width="50%"><img src="fleet_results/lqr_tuner/2_loops.png" alt="LQR-designed loops"/><br/><sub><b>LQR-designed loops:</b> open-loop Nichols plots and closed-loop rate steps</sub></td>
<td width="50%"><img src="sim_results/3_step_and_gust.png" alt="Step and gust rejection"/><br/><sub><b>Current vs optimised tune:</b> attitude step and gust rejection</sub></td>
</tr>
</table>

## Quick start

### 1. Before you fly: log at 400 Hz

The tuner and the simulation identify your airframe from fast rate data. ArduPilot's default logging is too slow for that, so set this once in Mission Planner (Config → Full Parameter List), then fly a normal 3–5 minute flight with some stick inputs:

```text
LOG_BITMASK = 180223      (adds ATTITUDE_FAST and PID logging at 400 Hz)
```

The log health check works with any log; only tuning and simulation need the 400 Hz data.

### 2. What you need

| | Needed for |
|---|---|
| **MATLAB** (tested on R2026a) | everything |
| **UAV Toolbox** | reading `.BIN` logs (`ardupilotreader`): nearly every tool |
| **Control System Toolbox** | LQR design, stability margins |
| **Signal Processing Toolbox** | system identification |
| **Statistics and Machine Learning Toolbox** | the log health check |
| Simulink + Aerospace Blockset *(optional)* | one cross-check in `validate_physics` |
| MATLAB Report Generator *(optional)* | PDF tuning reports |

### 3. Run it

Put your `.BIN` logs in the `logs/` folder, open MATLAB in this folder, then:

```matlab
addpath(genpath(pwd))
```

| I want to… | Run |
|---|---|
| Check a log for problems (any log rate) | `fc_log_analysis("logs/my_flight.bin")` |
| Get a new tune, with a GUI | `fc_tuner_app` |
| Get a new tune from a script | `out = fc_tune_from_log("logs/my_flight.bin")` |
| Simulate my drone and compare tunes | `run_sim` (uses the newest log in `logs/`) |
| See the physics model validated, no log needed | `validate_physics` (in `bench/`) |
| Test on SITL or real hardware | see [Bring your own](#bring-your-own) and `hil/HIL_RUNBOOK.md` |

**Windows launcher (optional):** `launcher/build_launcher.bat` builds a small `FC Tuner.exe` that opens the tuner without the MATLAB desktop. `Open_FC_Tuner.bat` does the same without building anything.

### If something stops

Every tool stops with a message saying what's missing. The common ones:

| Message | What to do |
|---|---|
| *"Rate data was logged at only 10 Hz…"* | The log is too slow for tuning. Set `LOG_BITMASK = 180223` and fly again. The health check still works on it. |
| *"No flight logs found in …/logs"* | Copy a `.BIN` into `logs/`, or pass a file name. |
| *"…fleet.mat not found. This script replays the original test campaign…"* | That script reproduces the author's own campaign, whose logs aren't included. Use `fc_tune_from_log` / `fc_tuner_app` for your aircraft. |
| *"Unrecognized function or variable 'ardupilotreader'"* | Install UAV Toolbox. |
| *"V8 skipped: … not found"* | Optional check; see [Bring your own](#bring-your-own). |

## What's inside

| Path | What it does |
|---|---|
| `fc_simulation.m` | Flight-log-calibrated ArduCopter simulation: rebuilds the 400 Hz attitude and rate controllers, identifies the airframe, runs linear analysis and time-domain tests, and searches for a better tune |
| `fc_log_analysis.m` | Pass / fail health report from a DataFlash log |
| `fc_tuner_app.m` | **FC PID Tuner** app: pick logs, press Tune, get a `.param` |
| `fc_live_test.m`, `fc_fake_drone.m`, `fc_bridge.py` | Live MAVLink bench test, a fake-drone MAVLink source, and a serial-to-UDP bridge |
| `bench/` | 6-DOF physics (`quad_physics`, validated against analytic cases), SITL bridge, LQR tuner, PID design space, fleet analysis, board benchmark |
| `hil/` | Hardware-in-the-loop: setup check, Level 1 (board I/O, props off) and Level 2 (simulation on hardware). Start with `hil/HIL_RUNBOOK.md` |
| `fleet_results/`, `logs_review/`, `sim_results/`, `bench/results/` | Example outputs from a real tuning campaign: figures, CSV tables and the generated `.param` |
| `post_crash_checklist.md`, `*.param` | A post-crash checklist and example parameter changes |

## Bring your own

These aren't in the repo. Add them locally if you need those features:

| What | Needed for | Where to get it |
|---|---|---|
| Your flight logs (`logs/*.BIN`) | Everything | Your flight controller's SD card or Mission Planner |
| ArduCopter SITL build (`sitl/bin/`) | SITL runs | Build from the [ArduPilot](https://github.com/ArduPilot/ardupilot) repo |
| ArduPilot's MATLAB SITL reference files (`sitl/ardupilot_ref/`) | `validate_physics` V8 check | ArduPilot repo, MATLAB SITL examples |
| Public PX4 logs (`bench/px4_logs/`) | Board benchmark | [PX4 Flight Review](https://review.px4.io) |

## Safety

Bench-test with **props off**, check the generated parameters before flying, and fly the first test somewhere open, with a failsafe configured.

## License

[MIT](LICENSE) © Prateek Sarkar

---

<div align="center">

### Found this useful?

**Drop a ⭐ so more drone builders find it, and share it with someone fighting their PID tune.**

[![Share on X](https://img.shields.io/badge/share%20on-X-000000?style=for-the-badge&logo=x&logoColor=white)](https://twitter.com/intent/tweet?text=Flight%20log%20in%2C%20tuned%20drone%20out%3A%20an%20open%20MATLAB%20toolkit%20for%20ArduPilot%20tuning%20%F0%9F%93%88&url=https%3A%2F%2Fgithub.com%2FPrateek-elec%2Fardupilot-flight-test-lab)
[![Share on LinkedIn](https://img.shields.io/badge/share%20on-LinkedIn-0A66C2?style=for-the-badge&logo=linkedin&logoColor=white)](https://www.linkedin.com/sharing/share-offsite/?url=https%3A%2F%2Fgithub.com%2FPrateek-elec%2Fardupilot-flight-test-lab)
[![Follow](https://img.shields.io/badge/follow-@Prateek--elec-181717?style=for-the-badge&logo=github&logoColor=white)](https://github.com/Prateek-elec)

</div>
