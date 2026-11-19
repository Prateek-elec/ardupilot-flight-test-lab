#!/usr/bin/env bash
# Build ArduCopter "Simulation on Hardware" firmware for the flight controller (ArduPilot target MicoAir743).
# Run inside WSL Ubuntu 22.04/24.04, NOT as root:
#     bash build_sim_on_hw.sh                # default tag Copter-4.7.3 (matches the PC SITL)
#     bash build_sim_on_hw.sh Copter-4.6.3   # match the firmware on the drone instead
# Output: SimOnHW_<tag>.apj next to this script -> flash with Mission Planner
#         (Install Firmware > Load custom firmware).
set -euo pipefail

TAG="${1:-Copter-4.7.3}"
BOARD="MicoAir743"
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="$HOME/ardupilot"

if [ "$(id -u)" = "0" ]; then echo "Do not run as root."; exit 1; fi

if [ ! -d "$SRC/.git" ]; then
    echo "== Cloning ArduPilot into $SRC (about 5 min)"
    git clone https://github.com/ArduPilot/ardupilot.git "$SRC"
fi
cd "$SRC"
git fetch --tags --quiet
if git rev-parse -q --verify "refs/tags/$TAG" >/dev/null; then
    git checkout -q "$TAG"
else
    echo "!! Tag $TAG not found - building master. List tags with: git tag -l 'Copter-4.*'"
    git checkout -q master && git pull -q
fi
git submodule update --init --recursive

if [ ! -f "$HOME/.ardupilot_prereqs_done" ]; then
    echo "== Installing build tools (asks for your Ubuntu password; about 15 min)"
    Tools/environment_install/install-prereqs-ubuntu.sh -y
    touch "$HOME/.ardupilot_prereqs_done"
fi
# toolchain and python venv paths added by install-prereqs
set +u; source "$HOME/.profile"; set -u

if [ ! -f Tools/scripts/sitl-on-hardware/sitl-on-hw.py ]; then
    echo "!! This ArduPilot version has no sitl-on-hw.py. Use a newer tag or master."; exit 1
fi
if [ ! -d "libraries/AP_HAL_ChibiOS/hwdef/$BOARD" ]; then
    echo "!! Board $BOARD not in this ArduPilot version."; exit 1
fi

# The simulated airframe = this drone (vehicle.json: 280 g, 220 mm X, 2S). The frame string is
# set here instead of --frame, because sitl-on-hw.py only accepts plain frame names.
# "x" = X layout (in ArduPilot's simulator "quad" means PLUS).
cp "$HERE/vehicle.json" "$SRC/Tools/autotest/models/vehicle.json"
EXTRA="$(mktemp --suffix=.dat)"
cat > "$EXTRA" <<EOF
define AP_SIM_FRAME_STRING "x:@ROMFS/models/vehicle.json"
define AP_MOTORS_FRAME_QUAD_ENABLED 1
ROMFS models/vehicle.json $SRC/Tools/autotest/models/vehicle.json
EOF
echo "== Extra hwdef:"; cat "$EXTRA"

echo "== Building Simulation-on-Hardware Copter for $BOARD ($TAG)"
python3 Tools/scripts/sitl-on-hardware/sitl-on-hw.py \
    --board "$BOARD" --vehicle copter --simclass MultiCopter \
    --extra-hwdef "$EXTRA" \
    --defaults "$HERE/build_defaults.parm"

APJ="build/$BOARD/bin/arducopter.apj"
[ -f "$APJ" ] || { echo "!! Build finished but $APJ is missing"; exit 1; }
OUT="$HERE/SimOnHW_${TAG}.apj"
cp "$APJ" "$OUT"
echo "== Done: $OUT"
ls -l "$OUT"
