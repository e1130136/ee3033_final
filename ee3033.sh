#!/usr/bin/env bash
# ============================================================
#  ee3033.sh — EE3033 Wheeltec Balancing Car one-stop script
# ============================================================
#  Usage:
#    ./ee3033.sh map          Start SLAM mapping on robot
#    ./ee3033.sh save [name]  Save map (default: ~/map)
#    ./ee3033.sh nav          Start navigation on robot
#    ./ee3033.sh cam          Start USB camera on robot
#    ./ee3033.sh repub        Start image republisher (laptop)
#    ./ee3033.sh yolo         Start darknet_ros YOLO (laptop)
#    ./ee3033.sh rviz         Start RViz (laptop)
#    ./ee3033.sh explore      Start multi_waypoint_nav (laptop)
#    ./ee3033.sh teleop       Start keyboard teleop on robot
#    ./ee3033.sh all_map      One-shot: map + rviz + teleop
#    ./ee3033.sh all_nav      One-shot: nav + cam + repub + yolo + rviz + explore
#    ./ee3033.sh check        Verify ROS topics are alive
#    ./ee3033.sh ssh          Plain SSH into robot
#
#  Dependencies:
#    sudo apt install sshpass     (auto SSH without typing password)
#    ROS Melodic or Noetic on laptop
#    catkin_make done in $LAPTOP_WS
#
#  First time setup:
#    chmod +x ee3033.sh
#    # Edit the variables below, then:
#    ./ee3033.sh check
# ============================================================

set -e

# ╔════════════════════════════════════════════════════════════╗
# ║  EDIT THESE VARIABLES FOR YOUR SETUP                      ║
# ╚════════════════════════════════════════════════════════════╝

ROBOT_USER="wheeltec"
ROBOT_IP="192.168.0.100"
ROBOT_PASS="dongguan"

# Your laptop's IP on the robot WiFi network.
# Find it: ip addr | grep 192.168   (or ifconfig)
LAPTOP_IP="192.168.0.163"

# Catkin workspace on your laptop (contains src/ devel/ build/)
LAPTOP_WS="$HOME/ee3033-main"

# RViz config file (leave empty for default)
RVIZ_CONFIG=""

# ╔════════════════════════════════════════════════════════════╗
# ║  DO NOT EDIT BELOW unless you know what you're doing      ║
# ╚════════════════════════════════════════════════════════════╝

ROBOT_SSH="${ROBOT_USER}@${ROBOT_IP}"

# ---- ROS environment ----
export ROS_MASTER_URI="http://${ROBOT_IP}:11311"
export ROS_HOSTNAME="${LAPTOP_IP}"
export ROS_IP="${LAPTOP_IP}"

# Source ROS
if [ -f /opt/ros/noetic/setup.bash ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/melodic/setup.bash ]; then
    source /opt/ros/melodic/setup.bash
else
    echo "[ERROR] ROS not found in /opt/ros/" >&2
    exit 1
fi

# Source workspace
if [ -f "${LAPTOP_WS}/devel/setup.bash" ]; then
    source "${LAPTOP_WS}/devel/setup.bash"
else
    echo "[WARN] Workspace not built: ${LAPTOP_WS}/devel/setup.bash" >&2
    echo "[WARN] Run: cd ${LAPTOP_WS} && catkin_make" >&2
fi

# ── Helpers ─────────────────────────────────────────────────

# Run a command on the robot over SSH with auto-password.
# Uses sshpass so you never have to type the password.
robot_cmd() {
    local cmd="$*"
    if command -v sshpass &>/dev/null; then
        sshpass -p "${ROBOT_PASS}" ssh -o StrictHostKeyChecking=no -t "${ROBOT_SSH}" \
            "bash -lc 'source /opt/ros/melodic/setup.bash 2>/dev/null; ros1 2>/dev/null; ${cmd}'"
    else
        echo "[WARN] sshpass not installed — you will be prompted for password"
        echo "[HINT] Install: sudo apt install sshpass"
        ssh -o StrictHostKeyChecking=no -t "${ROBOT_SSH}" \
            "bash -lc 'source /opt/ros/melodic/setup.bash 2>/dev/null; ros1 2>/dev/null; ${cmd}'"
    fi
}

# Open a new terminal window/tab to run a command.
# Works with gnome-terminal, xterm, or prints manual instructions.
new_term() {
    local title="$1"
    local cmd="$2"
    echo "[LAUNCH] ${title}"

    if command -v gnome-terminal &>/dev/null; then
        gnome-terminal --tab --title="${title}" -- bash -lc "${cmd}; echo '--- ${title} exited. Press Enter ---'; read"
    elif command -v xterm &>/dev/null; then
        xterm -T "${title}" -hold -e bash -lc "${cmd}" &
    else
        echo ""
        echo "  ┌──────────────────────────────────────────────────┐"
        echo "  │  OPEN A NEW TERMINAL AND RUN:                    │"
        echo "  │  Title: ${title}"
        echo "  │  ${cmd}"
        echo "  └──────────────────────────────────────────────────┘"
        echo ""
    fi
}

# Convenience: source this script's env in a sub-terminal
ENV_CMD="export ROS_MASTER_URI=http://${ROBOT_IP}:11311; export ROS_HOSTNAME=${LAPTOP_IP}; export ROS_IP=${LAPTOP_IP}"
SRC_ROS="source /opt/ros/melodic/setup.bash 2>/dev/null || source /opt/ros/noetic/setup.bash 2>/dev/null"
SRC_WS="source ${LAPTOP_WS}/devel/setup.bash 2>/dev/null || true"
LAPTOP_ENV="${SRC_ROS}; ${SRC_WS}; ${ENV_CMD}"

# Robot SSH command for use inside new_term
RSSH_PREFIX="sshpass -p ${ROBOT_PASS} ssh -o StrictHostKeyChecking=no -t ${ROBOT_SSH}"
ROBOT_ENV="source /opt/ros/melodic/setup.bash 2>/dev/null; ros1 2>/dev/null"

robot_term() {
    local title="$1"
    local rcmd="$2"
    if command -v sshpass &>/dev/null; then
        new_term "${title}" "${RSSH_PREFIX} \"bash -lc '${ROBOT_ENV}; ${rcmd}; exec bash'\""
    else
        new_term "${title}" "ssh -o StrictHostKeyChecking=no -t ${ROBOT_SSH} \"bash -lc '${ROBOT_ENV}; ${rcmd}; exec bash'\""
    fi
}

laptop_term() {
    local title="$1"
    local lcmd="$2"
    new_term "${title}" "${LAPTOP_ENV}; ${lcmd}"
}

# ── Commands ────────────────────────────────────────────────

do_map() {
    echo "=== Starting SLAM mapping on robot ==="
    robot_cmd "roslaunch turn_on_wheeltec_robot mapping.launch"
}

do_save() {
    local name="${1:-$HOME/map}"
    echo "=== Saving map to robot:${name} ==="
    robot_cmd "rosrun map_server map_saver -f ${name}"
    echo "=== Map saved: ${name}.pgm + ${name}.yaml ==="
}

do_nav() {
    echo "=== Starting navigation on robot ==="
    robot_cmd "roslaunch turn_on_wheeltec_robot navigation.launch"
}

do_cam() {
    echo "=== Starting USB camera on robot ==="
    robot_cmd "roslaunch usb_cam usb_cam-test.launch"
}

do_teleop() {
    echo "=== Starting keyboard teleop on robot ==="
    robot_cmd "roslaunch wheeltec_robot_rc keyboard_teleop.launch"
}

do_repub() {
    echo "=== Starting image republisher on laptop ==="
    rosrun image_transport republish compressed \
        in:=/usb_cam/image_raw \
        raw out:=/usb_cam/image_raw_uncompressed
}

do_yolo() {
    echo "=== Starting darknet_ros on laptop ==="
    roslaunch darknet_ros darknet_ros.launch \
        image:=/usb_cam/image_raw_uncompressed
}

do_rviz() {
    echo "=== Starting RViz on laptop ==="
    if [ -n "${RVIZ_CONFIG}" ] && [ -f "${RVIZ_CONFIG}" ]; then
        rviz -d "${RVIZ_CONFIG}"
    else
        rviz
    fi
}

do_explore() {
    echo "=== Starting multi_waypoint_nav on laptop ==="
    roslaunch maze_explore explore.launch
}

do_ssh() {
    echo "=== SSH into robot ==="
    if command -v sshpass &>/dev/null; then
        sshpass -p "${ROBOT_PASS}" ssh -o StrictHostKeyChecking=no "${ROBOT_SSH}"
    else
        ssh -o StrictHostKeyChecking=no "${ROBOT_SSH}"
    fi
}

do_check() {
    echo "=== Checking ROS connectivity ==="
    echo "ROS_MASTER_URI = ${ROS_MASTER_URI}"
    echo "ROS_HOSTNAME   = ${ROS_HOSTNAME}"
    echo ""

    echo "--- ping robot ---"
    ping -c 2 -W 2 "${ROBOT_IP}" 2>/dev/null && echo "OK" || echo "FAIL: cannot ping ${ROBOT_IP}"
    echo ""

    echo "--- rostopic list (first 15) ---"
    timeout 5 rostopic list 2>/dev/null | head -15 || echo "FAIL: cannot contact ROS master"
    echo ""

    echo "--- key topics ---"
    for t in /scan /map /amcl_pose /cmd_vel /usb_cam/image_raw /darknet_ros/bounding_boxes; do
        if timeout 3 rostopic info "$t" &>/dev/null; then
            echo "  [OK]   $t"
        else
            echo "  [MISS] $t"
        fi
    done
}

do_all_map() {
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  EE3033  ALL-IN-ONE MAPPING MODE                    ║"
    echo "║  Robot:  ${ROBOT_SSH}                               ║"
    echo "║  Laptop: ${LAPTOP_IP}                               ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""

    robot_term "1-robot-mapping" "roslaunch turn_on_wheeltec_robot mapping.launch"
    sleep 3
    laptop_term "2-rviz" "rviz"
    sleep 1
    robot_term "3-robot-teleop" "roslaunch wheeltec_robot_rc keyboard_teleop.launch"

    echo ""
    echo "=== Mapping mode launched ==="
    echo "  1. Drive robot with keyboard (terminal 3)"
    echo "  2. Watch map build in RViz (terminal 2)"
    echo "  3. When done:  ./ee3033.sh save"
    echo ""
}

do_all_nav() {
    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  EE3033  ALL-IN-ONE NAVIGATION + EXPLORE MODE       ║"
    echo "║  Robot:  ${ROBOT_SSH}                               ║"
    echo "║  Laptop: ${LAPTOP_IP}                               ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""

    # 1. Robot navigation (must be first — starts roscore + move_base + AMCL)
    robot_term "1-robot-nav" "roslaunch turn_on_wheeltec_robot navigation.launch"
    echo "  Waiting 5s for navigation to initialize..."
    sleep 5

    # 2. Robot camera
    robot_term "2-robot-cam" "roslaunch usb_cam usb_cam-test.launch"
    sleep 2

    # 3. Image republisher (laptop)
    laptop_term "3-repub" "rosrun image_transport republish compressed in:=/usb_cam/image_raw raw out:=/usb_cam/image_raw_uncompressed"
    sleep 1

    # 4. YOLO (laptop)
    laptop_term "4-yolo" "roslaunch darknet_ros darknet_ros.launch image:=/usb_cam/image_raw_uncompressed"
    sleep 2

    # 5. RViz (laptop)
    if [ -n "${RVIZ_CONFIG}" ] && [ -f "${RVIZ_CONFIG}" ]; then
        laptop_term "5-rviz" "rviz -d ${RVIZ_CONFIG}"
    else
        laptop_term "5-rviz" "rviz"
    fi

    echo ""
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║  ACTION REQUIRED:                                   ║"
    echo "║  1. In RViz (terminal 5), click '2D Pose Estimate'  ║"
    echo "║  2. Click robot's position on the map and drag      ║"
    echo "║  3. Wait for green particles to converge            ║"
    echo "║  4. Then press ENTER here to start exploration      ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""
    read -rp "Press ENTER when 2D Pose Estimate is done: "

    # 6. Multi-waypoint exploration (laptop)
    laptop_term "6-explore" "roslaunch maze_explore explore.launch"

    echo ""
    echo "=== All navigation terminals launched ==="
    echo "  Watch terminal 6 for FSM states:"
    echo "  WAIT_LOCALIZATION -> EXPLORE -> CANDIDATE_STOP -> MICRO_SCAN"
    echo "  -> ALIGN_TARGET -> APPROACH_TARGET -> RECORD_TARGET -> GO_HOME -> DONE"
    echo ""
}

# ── Main ────────────────────────────────────────────────────

case "${1:-}" in
    map)       do_map ;;
    save)      do_save "${2:-}" ;;
    nav)       do_nav ;;
    cam)       do_cam ;;
    repub)     do_repub ;;
    yolo)      do_yolo ;;
    rviz)      do_rviz ;;
    explore)   do_explore ;;
    teleop)    do_teleop ;;
    ssh)       do_ssh ;;
    check)     do_check ;;
    all_map)   do_all_map ;;
    all_nav)   do_all_nav ;;
    *)
        echo "EE3033 Wheeltec Balancing Car — one-stop script"
        echo ""
        echo "Usage: ./ee3033.sh <command>"
        echo ""
        echo "  Single commands (run one at a time):"
        echo "    map        SLAM mapping on robot"
        echo "    save [n]   Save map to robot (default: ~/map)"
        echo "    nav        Navigation on robot"
        echo "    cam        USB camera on robot"
        echo "    teleop     Keyboard teleop on robot"
        echo "    repub      Image republisher (laptop)"
        echo "    yolo       YOLO detection (laptop)"
        echo "    rviz       RViz (laptop)"
        echo "    explore    Waypoint nav node (laptop)"
        echo "    ssh        SSH into robot"
        echo "    check      Verify topics and connectivity"
        echo ""
        echo "  All-in-one (opens multiple terminals):"
        echo "    all_map    mapping + rviz + teleop"
        echo "    all_nav    nav + cam + repub + yolo + rviz + explore"
        echo ""
        exit 1
        ;;
esac
