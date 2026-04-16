#!/usr/bin/env bash
# ===========================================================
# bringup.sh — SIMULATION ONLY (Gazebo + TurtleBot3)
# ===========================================================
# This script is for LOCAL Gazebo simulation testing only.
# It uses turtlebot3_slam, turtlebot3_navigation, and the
# ee3033_sim package (the 230-line simplified Python node).
#
# FOR THE PHYSICAL WHEELTEC ROBOT DEMO, use instead:
#   ./launch/ee3033_mapping_mode.sh   (SLAM + teleop)
#   ./launch/ee3033_explore_mode.sh   (navigation + detection)
#
# DO NOT use 'bringup.sh runpy' for the physical robot — it
# runs the wrong Python file (ee3033_sim version) and does not
# load the YAML parameters.
# ===========================================================
set -e

WS="$HOME/ee3033-main"
MAP_BASENAME="$HOME/map"
MODEL="burger"

source /opt/ros/melodic/setup.bash

# 🔥 ADD THESE LINES
unset ROS_IP
unset ROS_HOSTNAME
export ROS_MASTER_URI=http://localhost:11311
export TURTLEBOT3_MODEL="$MODEL"

run_in_terminal() {
  local title="$1"
  local cmd="$2"
  gnome-terminal --title="$title" -- bash -lc "$cmd; exec bash"
}

case "${1:-}" in
  map)
    "$0" stop >/dev/null 2>&1 || true
    sleep 1
    echo "Starting mapping workflow with camera-enabled TurtleBot3..."

    run_in_terminal "TB3 Camera Bringup" \
      "source /opt/ros/melodic/setup.bash; \
       cd $WS; source devel/setup.bash; \
       export TURTLEBOT3_MODEL=$MODEL; \
       roslaunch ee3033_sim tb3_camera_bringup.launch"

    sleep 4

    run_in_terminal "TB3 SLAM" \
      "source /opt/ros/melodic/setup.bash; \
       export TURTLEBOT3_MODEL=$MODEL; \
       roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping"

    sleep 3

    run_in_terminal "TB3 Teleop" \
      "source /opt/ros/melodic/setup.bash; \
       export TURTLEBOT3_MODEL=$MODEL; \
       roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch"

    echo "Mapping terminals launched."
    echo "Drive the robot around to build the map."
    echo "Then run:"
    echo "  ./bringup.sh save"
    ;;

  save)
    echo "Saving map to ${MAP_BASENAME}.pgm / ${MAP_BASENAME}.yaml ..."
    source /opt/ros/melodic/setup.bash
    rosrun map_server map_saver -f "$MAP_BASENAME"
    echo "Map saved."
    ;;

  stop)
    echo "Stopping ROS/Gazebo processes..."
    pkill -f ee3033_sim || true
    pkill -f turtlebot3_gazebo || true
    pkill -f turtlebot3_slam || true
    pkill -f turtlebot3_navigation || true
    pkill -f turtlebot3_teleop_key || true
    pkill -f robot_state_publisher || true
    pkill -f spawn_model || true
    pkill -f gzserver || true
    pkill -f gzclient || true
    # pkill -f roscore || true
    pkill -f roslaunch || true
    echo "Stopped."
    ;;

  nav)
    "$0" stop >/dev/null 2>&1 || true
    sleep 1
    echo "Starting navigation workflow with camera-enabled TurtleBot3..."
    if [[ ! -f "${MAP_BASENAME}.yaml" ]]; then
      echo "Map file not found: ${MAP_BASENAME}.yaml"
      echo "Run mapping first, then save the map."
      exit 1
    fi

    run_in_terminal "TB3 Camera Bringup" \
      "source /opt/ros/melodic/setup.bash; \
       cd $WS; source devel/setup.bash; \
       export TURTLEBOT3_MODEL=$MODEL; \
       roslaunch ee3033_sim tb3_camera_bringup.launch"

    sleep 4

    run_in_terminal "TB3 Navigation" \
      "source /opt/ros/melodic/setup.bash; \
       export TURTLEBOT3_MODEL=$MODEL; \
       roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=${MAP_BASENAME}.yaml"

    echo "Navigation terminals launched."
    echo "In RViz, use '2D Pose Estimate' once before running your Python."
    echo "Then run:"
    echo "  ./bringup.sh check"
    echo "  ./bringup.sh camcheck"
    echo "  ./bringup.sh runpy"
    ;;

  check)
    echo "Checking move_base topics..."
    source /opt/ros/melodic/setup.bash
    rostopic list | grep move_base || true
    echo
    echo "Checking one AMCL pose message..."
    rostopic echo -n 1 /amcl_pose
    ;;

  camcheck)
    echo "Checking camera topics..."
    source /opt/ros/melodic/setup.bash
    rostopic list | grep -E "camera|image_raw|camera_info" || true
    echo
    echo "Checking one image topic publisher..."
    rostopic info /camera/image_raw || true
    ;;

  viewcam)
    echo "Opening camera view..."
    source /opt/ros/melodic/setup.bash
    rosrun image_view image_view image:=/camera/image_raw
    ;;

  runpy)
    echo "Running multi_waypoint_nav.py ..."
    source /opt/ros/melodic/setup.bash
    cd "$WS"
    source devel/setup.bash
    rosrun ee3033_sim multi_waypoint_nav.py
    ;;
    *)
    echo "Usage:"
    echo "  ./bringup.sh map       # start camera-TB3 + SLAM + teleop"
    echo "  ./bringup.sh save      # save map to ~/map"
    echo "  ./bringup.sh stop      # stop Gazebo/ROS launch processes"
    echo "  ./bringup.sh nav       # start camera-TB3 + AMCL + move_base"
    echo "  ./bringup.sh check     # verify move_base and /amcl_pose"
    echo "  ./bringup.sh camcheck  # verify camera topics"
    echo "  ./bringup.sh viewcam   # open camera image viewer"
    echo "  ./bringup.sh runpy     # run your multi_waypoint_nav.py"
    exit 1
    ;;
esac
