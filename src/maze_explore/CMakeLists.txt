cmake_minimum_required(VERSION 3.0.2)
project(maze_explore)

# -----------------------------------------------------------------------
# maze_explore is a pure-Python ROS1 package.
# No C++ compilation. No message generation.
# darknet_ros_msgs is intentionally NOT listed as a required catkin
# component so that catkin_make succeeds even if darknet_ros is not yet
# built. The Python node handles missing darknet_ros_msgs via try/except.
# -----------------------------------------------------------------------

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  move_base_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
)

catkin_package()

# Install the main navigation node and any helper scripts
catkin_install_python(PROGRAMS
  scripts/multi_waypoint_nav.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config and launch directories so roslaunch / rosparam can find them
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
