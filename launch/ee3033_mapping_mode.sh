#!/usr/bin/env bash
set -e
source "$(dirname "$0")/ee3033_common.sh"

# Terminal 1: mapping on robot
open_tab "robot-mapping" "source $(dirname "$0")/ee3033_common.sh; robot_ros_cmd 'roslaunch turn_on_wheeltec_robot mapping.launch'"

# Terminal 2: RViz on laptop
if [ -n "$EE3033_RVIZ_CONFIG" ]; then
  open_tab "rviz" "source $(dirname "$0")/ee3033_common.sh; rviz -d '$EE3033_RVIZ_CONFIG'"
else
  open_tab "rviz" "source $(dirname "$0")/ee3033_common.sh; rviz"
fi

# Terminal 3: keyboard teleop on robot
open_tab "robot-teleop" "source $(dirname "$0")/ee3033_common.sh; robot_ros_cmd 'roslaunch wheeltec_robot_rc keyboard_teleop.launch'"

echo "Mapping mode launched."
echo "Remember: save the map BEFORE stopping mapping."
