cmake_minimum_required(VERSION 3.0.2)
project(go2w_parking)

find_package(catkin REQUIRED COMPONENTS
  apriltag_ros geometry_msgs nav_msgs rospy sensor_msgs std_msgs std_srvs tf
  visualization_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/parking_sim.py
  scripts/auto_dock.py
  scripts/mode_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
