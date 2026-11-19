# Your FC post-crash checklist (12 Nov 2026)

Do these in order. Props OFF until step 6.

## 1. Mechanical (the cause of the crash)
- [ ] All 16 motor screws: correct length (must not touch the windings), blue thread-lock, tight.
- [ ] Each motor sits flat and straight on its arm; no twist. Check arms for cracks or bends.
- [ ] Props balanced and undamaged; replace any that hit the ground.
- [ ] Flight controller soft-mounted (rubber grommets / foam), not touching the frame or wires.
- [ ] No loose battery: strap it tight so it cannot move in flight.

## 2. Compass
- [ ] GPS/compass on a mast, as far as possible from the battery, power leads, ESCs and the XT60.
- [ ] No steel screws, standoffs or a magnetised tool near it (use nylon or brass).
- [ ] Arrow on the GPS module points forward (COMPASS_ORIENT matches the mounting).

## 3. Load parameters
- [ ] Mission Planner: CONFIG > Full Parameter List > Save to file (backup first).
- [ ] Load from file: `post_crash_fix.param` > Write Params > reboot.

## 4. Calibrate (outdoors, away from cars and metal)
- [ ] Accelerometer calibration.
- [ ] Compass calibration (rotate on all axes). Pass condition: offsets within +-600 per axis, and
      no "Check mag field" pre-arm message on the HUD.

## 5. Bench checks (props off)
- [ ] Radio Calibration: pitch stick forward must LOWER CH2. If it raises it, reverse CH2 on the
      transmitter OR set RC2_REVERSED = 1, not both.
- [ ] Motor test (SETUP > Optional Hardware > Motor Test): A = front-right, B = rear-right,
      C = rear-left, D = front-left, each spinning in the direction of the diagram for QUAD/X.
      (ArduPilot outputs: M1 front-right CCW, M2 rear-left CCW, M3 front-left CW, M4 rear-right CW.)
- [ ] Props: CCW props on front-right and rear-left, CW props on front-left and rear-right.
- [ ] Optional: `python hil\hil_l1_io_test.py COM8 --arm --tilt` (props off) checks motor
      directions automatically.

## 6. Test hover
- [ ] Props on, open area, STABILIZE or ALT_HOLD, 1 m for 1-2 min, then land.
- [ ] Send the log. Pass: vibration < 30 m/s^2, 0 clipping, no motor at 1950 in hover,
      motors within about 100 us of each other, no compass or EKF messages.
