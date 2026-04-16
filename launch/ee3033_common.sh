#!/usr/bin/env bash
# ============================================================
# ee3033_common.sh — shared environment for EE3033 scripts
# ============================================================
# SOURCE THIS FILE; do not execute it directly.
#
# !! EDIT THE VARIABLES BELOW BEFORE YOUR DEMO !!
# ============================================================

# ----- Robot credentials -----
export EE3033_ROBOT_USER="wheeltec"
export EE3033_ROBOT_IP="192.168.0.100"
export EE3033_ROBOT_SSH="${EE3033_ROBOT_USER}@${EE3033_ROBOT_IP}"

# ----- Laptop IP on the robot WiFi network -----
# Find your IP: run `ip addr` or `ifconfig` after connecting to robot WiFi.
export EE3033_LAPTOP_IP="192.168.0.163"

# ----- Workspace paths -----
# EE3033_WS: the root of your catkin workspace (contains src/ devel/ build/).
# On Linux VM/laptop, this is usually $HOME/ee3033-main.
# If you are on macOS for development only, point this to your actual path.
export EE3033_WS="$HOME/ee3033-main"

# The Python node script name (used by rosrun fallback, but roslaunch is preferred)
export EE3033_NAV_NODE="multi_waypoint_nav.py"

# Optional: absolute path to a .rviz config file.  Leave empty to open RViz blank.
export EE3033_RVIZ_CONFIG=""

# ----- ROS network configuration -----
# ROS master runs on the ROBOT; laptop communicates back via its own IP.
export ROS_MASTER_URI="http://${EE3033_ROBOT_IP}:11311"
export ROS_HOSTNAME="${EE3033_LAPTOP_IP}"
export ROS_IP="${EE3033_LAPTOP_IP}"

# ----- Source ROS and workspace -----
# Adjust for Noetic: change melodic -> noetic
if [ -f /opt/ros/melodic/setup.bash ]; then
  source /opt/ros/melodic/setup.bash
elif [ -f /opt/ros/noetic/setup.bash ]; then
  source /opt/ros/noetic/setup.bash
else
  echo "[ERROR] Neither /opt/ros/melodic nor /opt/ros/noetic found." >&2
  echo "[ERROR] Install ROS on this machine." >&2
fi

if [ -f "${EE3033_WS}/devel/setup.bash" ]; then
  source "${EE3033_WS}/devel/setup.bash"
else
  echo "[WARN] Workspace devel not found: ${EE3033_WS}/devel/setup.bash" >&2
  echo "[WARN] Run 'cd ${EE3033_WS} && catkin_make' first." >&2
fi

# ----- Helper: run a command on the robot over SSH -----
# Usage: robot_ros_cmd 'roslaunch turn_on_wheeltec_robot mapping.launch'
robot_ros_cmd() {
  # 'ros1' is Wheeltec's alias that sets up their ROS environment.
  # The 'exec bash' at the end keeps gnome-terminal tabs open after the
  # command exits so you can read the output.
  ssh -t "${EE3033_ROBOT_SSH}" \
    "bash -lc 'source /opt/ros/melodic/setup.bash 2>/dev/null || true; ros1 >/dev/null 2>&1 || true; $*; exec bash'"
}

# ----- Helper: open a new terminal tab -----
# Works on Linux with GNOME (Ubuntu default desktop).
# On macOS: open each terminal manually; see comments in the calling script.
open_tab() {
  local title="$1"
  local cmd="$2"

  if command -v gnome-terminal &>/dev/null; then
    # GNOME terminal (Linux Ubuntu)
    gnome-terminal --tab --title="${title}" -- bash -lc "${cmd}; exec bash"
  elif command -v xterm &>/dev/null; then
    # Fallback: plain xterm
    xterm -T "${title}" -e "bash -lc '${cmd}; exec bash'" &
  else
    # macOS or headless — print the command for manual execution
    echo ""
    echo "========== OPEN A NEW TERMINAL AND RUN: =========="
    echo "  [${title}]"
    echo "  ${cmd}"
    echo "==================================================="
  fi
}
