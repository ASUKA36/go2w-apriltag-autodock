cmake_minimum_required(VERSION 3.0.2)
project(go2w_vision)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
  geometry_msgs
  rospy
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/tag_follower.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
