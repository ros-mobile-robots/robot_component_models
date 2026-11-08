cmake_minimum_required(VERSION 3.10)
project(robot_component_models)

# The package installs data only: the meshes with their manifest, and the license.
# ROS 1 (Noetic) builds it with catkin, ROS 2 (Humble and later) with ament_cmake.
# package.xml selects the build tool with the ROS_VERSION condition.
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_package()
  set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
else()
  find_package(ament_cmake REQUIRED)
  set(SHARE_DESTINATION share/${PROJECT_NAME})
endif()

install(DIRECTORY meshes DESTINATION ${SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${SHARE_DESTINATION})

if(NOT "$ENV{ROS_VERSION}" STREQUAL "1")
  ament_package()
endif()
