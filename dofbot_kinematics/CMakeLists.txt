cmake_minimum_required(VERSION 3.0.2)
project(dofbot_kinematics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf2_eigen
  tf2_geometry_msgs
  moveit_msgs
  geometry_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES dofbot_kinematics
  CATKIN_DEPENDS roscpp pluginlib moveit_core moveit_msgs geometry_msgs tf2_geometry_msgs tf2_eigen
)

add_library(dofbot_kinematics
  src/dofbot_ik_plugin.cpp
)

target_include_directories(dofbot_kinematics PRIVATE
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(dofbot_kinematics
  ${catkin_LIBRARIES}
)

install(TARGETS dofbot_kinematics
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES
  dofbot_ik_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
