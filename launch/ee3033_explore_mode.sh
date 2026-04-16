#!/usr/bin/env bash
# ============================================================
# ee3033_explore_mode.sh — physical robot exploration mode
# ============================================================
# Starts 6 terminals for the full demo pipeline:
#   1. Robot: navigation.launch  (move_base + AMCL + map)
#   2. Robot: usb_cam-test.launch  (camera)
#   3. Laptop: image republisher  (compressed -> raw)
#   4. Laptop: darknet_ros  (YOLO detection)
#   5. Laptop: RViz
#   6. Laptop: multi_waypoint_nav (via roslaunch to load YAML params)
#
# !! BEFORE RUNNING !!
# 1. Edit launch/ee3033_common.sh: set EE3033_WS, EE3033_LAPTOP_IP.
# 2. Edit config/multi_waypoint_nav_melodic_params.yaml: set waypoints
#    and target_class for your demo.
# 3. Have the saved map from mapping mode ready on the robot.
# 4. Run this script AFTER confirming you can `ping 192.168.0.100`.
# ============================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/ee3033_common.sh"

echo ""
echo "=========================================================="
echo " EE3033 Explore Mode"
echo " Robot:  ${EE3033_ROBOT_SSH}"
echo " Master: ${ROS_MASTER_URI}"
echo " Laptop: ${EE3033_LAPTOP_IP}"
echo "=========================================================="
echo ""

# Terminal 1: navigation on robot (move_base + AMCL + map server)
open_tab "1-robot-navigation" \
  "source ${SCRIPT_DIR}/ee3033_common.sh; \
   robot_ros_cmd 'roslaunch turn_on_wheeltec_robot navigation.launch'"

sleep 1

# Terminal 2: USB camera on robot
open_tab "2-robot-camera" \
  "source ${SCRIPT_DIR}/ee3033_common.sh; \
   robot_ros_cmd 'roslaunch usb_cam usb_cam-test.launch'"

sleep 1

# Terminal 3: image republisher on laptop
# Converts /usb_cam/image_raw (compressed transport) to a plain raw topic
# that darknet_ros can subscribe to without needing compressed support.
open_tab "3-republish" \
  "source ${SCRIPT_DIR}/ee3033_common.sh; \
   rosrun image_transport republish compressed \
     in:=/usb_cam/image_raw \
     raw out:=/usb_cam/image_raw_uncompressed"

sleep 1

# Terminal 4: darknet_ros on laptop
# - 'image' argument remaps camera/rgb/image_raw -> /usb_cam/image_raw_uncompressed
# - do NOT pass 'camera_info' — that parameter does not exist in darknet_ros.launch
# - The workspace is already sourced via ee3033_common.sh; no 'cd' needed.
open_tab "4-darknet-ros" \
  "source ${SCRIPT_DIR}/ee3033_common.sh; \
   roslaunch darknet_ros darknet_ros.launch \
     image:=/usb_cam/image_raw_uncompressed"

sleep 2

# Terminal 5: RViz on laptop
# After RViz opens: click '2D Pose Estimate' on the toolbar and click the
# robot's actual starting position on the map BEFORE launching terminal 6.
if [ -n "${EE3033_RVIZ_CONFIG}" ] && [ -f "${EE3033_RVIZ_CONFIG}" ]; then
  open_tab "5-rviz" \
    "source ${SCRIPT_DIR}/ee3033_common.sh; \
     rviz -d '${EE3033_RVIZ_CONFIG}'"
else
  open_tab "5-rviz" \
    "source ${SCRIPT_DIR}/ee3033_common.sh; \
     rviz"
fi

echo ""
echo "=========================================================="
echo " ACTION REQUIRED:"
echo "   In RViz (terminal 5), click '2D Pose Estimate' on the"
echo "   toolbar and mark the robot's position on the map."
echo "   Wait until the green particles cluster around the robot."
echo "   THEN run terminal 6 (multi_waypoint_nav)."
echo "=========================================================="
echo ""
read -rp "Press ENTER when 2D Pose Estimate is done and AMCL looks stable: "

# Terminal 6: multi_waypoint_nav via roslaunch
# roslaunch loads config/multi_waypoint_nav_melodic_params.yaml automatically.
# This is the ONLY correct way to start the node — 'rosrun' alone does NOT
# load the YAML parameters.
open_tab "6-multi-waypoint" \
  "source ${SCRIPT_DIR}/ee3033_common.sh; \
   cd '${EE3033_WS}'; \
   roslaunch maze_explore explore.launch"

echo ""
echo "All terminals launched."
echo "Watch terminal 6 for state machine transitions:"
echo "  WAIT_LOCALIZATION -> EXPLORING -> CLOSE_TARGET -> GO_HOME -> FINISHED"
