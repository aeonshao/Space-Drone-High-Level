#!/usr/bin/env bash
# Starts gazebo with our tool_field world and then PX4 SITL (x500 with a downward camera)
# attached to it. Basically does what `make px4_sitl gz_x500_mono_cam_down` does but
# with our own world, since PX4 only looks for worlds inside its own folder.
#
# usage:  ./sim/run_sim.sh            (with gui)
#         HEADLESS=1 ./sim/run_sim.sh (no gui, for laptops that cant handle it)
#
# source /opt/ros/jazzy/setup.bash first, the gz binary comes from the ros vendor packages
#
# Ctrl+C kills both.

PX4_DIR=${PX4_DIR:-$HOME/PX4-Autopilot}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD=tool_field

if [ ! -f "$PX4_DIR/build/px4_sitl_default/bin/px4" ]; then
    echo "px4 isnt built, run 'make px4_sitl_default' in $PX4_DIR first"
    exit 1
fi

# same stuff as PX4's gz_env.sh
export PX4_GZ_MODELS=$PX4_DIR/Tools/simulation/gz/models
export PX4_GZ_WORLDS=$PX4_DIR/Tools/simulation/gz/worlds
export GZ_SIM_RESOURCE_PATH=$SCRIPT_DIR/models:$PX4_GZ_MODELS:$PX4_GZ_WORLDS:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_SERVER_CONFIG_PATH=$PX4_DIR/src/modules/simulation/gz_bridge/server.config

cleanup() {
    echo "shutting down..."
    kill $GZ_PID $GZ_GUI_PID 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
}
trap cleanup EXIT

# -r = run right away, -s = server only
gz sim -r -s "$SCRIPT_DIR/worlds/$WORLD.sdf" &
GZ_PID=$!

if [ -z "$HEADLESS" ]; then
    gz sim -g > /dev/null 2>&1 &
    GZ_GUI_PID=$!
fi

# 4014 = gz_x500_mono_cam_down airframe
cd "$PX4_DIR/build/px4_sitl_default" || exit 1
PX4_GZ_STANDALONE=1 \
PX4_SYS_AUTOSTART=4014 \
PX4_SIM_MODEL=gz_x500_mono_cam_down \
PX4_GZ_WORLD=$WORLD \
    ./bin/px4 $PX4_ARGS
# PX4_ARGS=-d for daemon mode (no pxh> shell). needed when running from a script/ssh
# without a terminal, otherwise pxh spams the prompt forever and fills up your disk (ask me how i know)
