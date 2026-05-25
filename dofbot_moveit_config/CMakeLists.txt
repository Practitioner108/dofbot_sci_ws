cmake_minimum_required(VERSION 3.1.3)
project(dofbot_moveit_config)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS dofbot_description dofbot_control
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
