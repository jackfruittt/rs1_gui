#!/bin/bash

# Multi-drone composition launcher - uses new sensor+controller composition architecture
# Usage: ./launch_multi_drone_composition.sh <num_drones> [extra_ros2_args]
# Example: ./launch_multi_drone_composition.sh 3
# Example: ./launch_multi_drone_composition.sh 4 gazebo:=true
# Example: ./launch_multi_drone_composition.sh 2 gazebo:=true rviz:=true

set -e

# Default to 1 drone if no argument provided
NUM_DRONES=${1:-1}
shift

# Validate input
if ! [[ "$NUM_DRONES" =~ ^[1-9][0-9]*$ ]] || [ "$NUM_DRONES" -gt 10 ]; then
    echo "Error: Please provide a valid number of drones (1-10)"
    echo "Usage: $0 <num_drones> [extra_ros2_args]"
    echo "Examples:"
    echo "  $0 3                    # 3 drones headless"
    echo "  $0 4 gazebo:=true       # 4 drones with Gazebo GUI"
    echo "  $0 2 gazebo:=true rviz:=true  # 2 drones with GUI + RViz"
    exit 1
fi

echo "=========================================="
echo "Multi-Drone Composition Launcher"
echo "Architecture: Sensor+Controller per thread"
echo "Spawning $NUM_DRONES drones..."
echo "=========================================="

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -f "ign gazebo" || true
pkill -f "parameter_bridge" || true
pkill -f "robot_state_publisher" || true
pkill -f "multi_drone_composition_controller" || true
pkill -f "controller_node" || true
sleep 2

# ~/RS1/rs1_ws$ source install/setup.bash 

# Source workspace
# if [ -f "$HOME/RS1/rs1_ws/install/setup.bash" ]; then
#     source "$HOME/RS1/rs1_ws/install/setup.bash"
#     echo "Sourced ROS2 workspace from ~/RS1/rs1_ws"
# elif [ -f "rs1_ws/install/setup.bash" ]; then
#     source rs1_ws/install/setup.bash
#     echo "Sourced ROS2 workspace from ./rs1_ws"
# else
#     echo "Warning: Could not find rs1_ws workspace setup.bash"
#     echo "Make sure your workspace is built and sourced"
# fi

if [ -f "$HOME/RS1/rs1_ws/install/setup.bash" ]; then
  . "$HOME/RS1/rs1_ws/install/setup.bash"
  echo "Sourced ROS2 workspace from ~/RS1/rs1_ws"
elif [ -f "$HOME/rs1_ws/install/setup.bash" ]; then
  . "$HOME/rs1_ws/install/setup.bash"
  echo "Sourced ROS2 workspace from ~/rs1_ws"
elif [ -f "rs1_ws/install/setup.bash" ]; then
  . rs1_ws/install/setup.bash
  echo "Sourced ROS2 workspace from ./rs1_ws"
else
  echo "Warning: Could not find rs1_ws workspace setup.bash"
  echo "Make sure your workspace is built and sourced"
fi

# --- Neutralise Qt poisoning (OpenCV injects its own plugins) ---
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH
export QT_QPA_PLATFORM=xcb
if [[ " $* " == *" gui:=false "* ]]; then
  export QT_QPA_PLATFORM=offscreen   # no X/Wayland, no GUI
  export RS1_IGN_HEADLESS=1          # we'll use this in the launch (see below)
fi

echo ""
echo "Launching composition-based multi-drone system..."
echo "Architecture: Each drone = SensorProcessor + CompositeDroneController on same thread"
echo "Drones: $NUM_DRONES"
echo "Launch arguments: $@"
echo ""

echo "[spawner] QT_PLUGIN_PATH='${QT_PLUGIN_PATH:-<unset>}'"
echo "[spawner] QT_QPA_PLATFORM='${QT_QPA_PLATFORM:-<unset>}'"
echo "[spawner] RS1_IGN_HEADLESS='${RS1_IGN_HEADLESS:-<unset>}'"

# Launch the remote controller node to run in the background - Controller needs to be connected when launched otherwise
# when connected after, it causes an issue with cmd_vel values causing drone to spiral out of control
ros2 launch rs1_teensyjoy rs1_teensyjoy.launch.py \
  drone_count:=$NUM_DRONES \
  "$@" &

# # Launch the composition spawner with all arguments
# ros2 launch rs1_robot rs1_swarm_composed.py num_drones:=$NUM_DRONES "$@"
ros2 launch rs1_robot rs1_swarm_composed.py \
  num_drones:=$NUM_DRONES \
  "$@"


echo ""
echo "=========================================="
echo "Multi-drone composition launch completed!"
echo "Available topics for individual control:"
for i in $(seq 1 $NUM_DRONES); do
    echo "  /rs1_drone_${i}/waypoints"
done
echo ""
echo "Global mission control:"
echo "  /mission_control service"
echo "  /waypoints topic (auto-distributed)"
echo "=========================================="
