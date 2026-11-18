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

install(DIRECTORY meshes DESTINATION ${SHARE_DESTINATION})
install(FILES LICENSE DESTINATION ${SHARE_DESTINATION})

# A provenance page links to its generator and its pictures with paths relative to the
# repository, such as ../generators/rplidar_a2.py. The installed package holds the pages
# without the generators and pictures. So each installed page links to the same files in the
# repository on GitHub, at the tag of this version: a picture to the file itself (raw), every
# other file to its page (blob). scripts/check_install.py makes the same replacements and
# compares.
set(REPOSITORY_URL "https://github.com/ros-mobile-robots/robot_component_models")
file(READ package.xml PACKAGE_XML)
string(REGEX MATCH "<version>([^<]+)</version>" VERSION_ELEMENT "${PACKAGE_XML}")
set(PACKAGE_VERSION "${CMAKE_MATCH_1}")
if(PACKAGE_VERSION STREQUAL "")
  message(FATAL_ERROR "package.xml has no <version> element")
endif()
file(GLOB PROVENANCE_PAGES provenance/*.md)
foreach(PAGE ${PROVENANCE_PAGES})
  get_filename_component(PAGE_NAME "${PAGE}" NAME)
  file(READ "${PAGE}" PAGE_TEXT)
  string(REPLACE "](../docs/images/" "](${REPOSITORY_URL}/raw/${PACKAGE_VERSION}/docs/images/"
         PAGE_TEXT "${PAGE_TEXT}")
  string(REPLACE "](../" "](${REPOSITORY_URL}/blob/${PACKAGE_VERSION}/" PAGE_TEXT "${PAGE_TEXT}")
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/provenance/${PAGE_NAME}" "${PAGE_TEXT}")
endforeach()
install(DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/provenance" DESTINATION ${SHARE_DESTINATION})

if("$ENV{ROS_VERSION}" STREQUAL "2")
  ament_package()
endif()
