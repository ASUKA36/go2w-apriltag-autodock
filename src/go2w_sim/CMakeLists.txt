cmake_minimum_required(VERSION 3.0.2)
project(go2w_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(gazebo REQUIRED)

catkin_package(LIBRARIES go2w_planar_plugin go2w_dynamics_plugin)

include_directories(${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

add_library(go2w_planar_plugin src/go2w_planar_plugin.cpp)
target_link_libraries(go2w_planar_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_library(go2w_dynamics_plugin src/go2w_dynamics_plugin.cpp)
target_link_libraries(go2w_dynamics_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_install_python(PROGRAMS
  scripts/smoke_test.py
  scripts/vision_test.py
  scripts/world_map_node.py
  scripts/teleop_keyboard.py
  scripts/unpause_after_spawn.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS go2w_planar_plugin go2w_dynamics_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY config launch media urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
