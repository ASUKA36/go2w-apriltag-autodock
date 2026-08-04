cmake_minimum_required(VERSION 3.0.2)
project(go2w_navigation)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  rospy
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/navigation_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
