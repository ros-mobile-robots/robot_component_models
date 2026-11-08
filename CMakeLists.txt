# SPDX-FileCopyrightText: 2026 ROS Mobile Robots
# SPDX-License-Identifier: BSD-3-Clause
cmake_minimum_required(VERSION 3.10)
project(robot_component_models)

# The package installs data only: the meshes with their manifest and license files, the
# provenance of each part, and the license. ROS 1 (Noetic) builds it with catkin, ROS 2 (Humble
# and later) with ament_cmake. package.xml selects the build tool by ROS_VERSION, which the ROS
# setup script sets.
if("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_package()
  set(SHARE_DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  set(SHARE_DESTINATION share/${PROJECT_NAME})
else()
  message(FATAL_ERROR "ROS_VERSION is '$ENV{ROS_VERSION}', not 1 or 2. Source the setup script "
                      "of your ROS distribution, for example /opt/ros/humble/setup.bash, first.")
endif()

install(DIRECTORY meshes provenance DESTINATION ${SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${SHARE_DESTINATION})

if("$ENV{ROS_VERSION}" STREQUAL "2")
  ament_package()
endif()
