cmake_minimum_required(VERSION 3.0.2)
project(dofbot_control)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  sensor_msgs
  std_msgs
  joint_limits_interface
  urdf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dofbot_control
  CATKIN_DEPENDS roscpp hardware_interface controller_manager sensor_msgs std_msgs joint_limits_interface urdf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(dofbot_control
  src/dofbot_hw_interface.cpp
  src/dofbot_sim_interface.cpp
)

target_link_libraries(dofbot_control
  ${catkin_LIBRARIES}
)

add_executable(dofbot_hw_node
  src/dofbot_hw_node.cpp
)

target_link_libraries(dofbot_hw_node
  dofbot_control
  ${catkin_LIBRARIES}
)

install(TARGETS dofbot_control dofbot_hw_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
