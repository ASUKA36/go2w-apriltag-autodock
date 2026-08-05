cmake_minimum_required(VERSION 3.0.2)
project(go2w_vision)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros
)

catkin_package()

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
