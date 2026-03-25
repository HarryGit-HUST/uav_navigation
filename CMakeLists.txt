cmake_minimum_required(VERSION 2.8.3)
project(uav_navigation)

## Compile as C++14 (required for PCL 1.10)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros
  mavros_msgs
  sensor_msgs
  nav_msgs
  pcl_ros
  pcl_conversions
  quadrotor_msgs
)

## System dependencies
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs mavros mavros_msgs sensor_msgs nav_msgs pcl_ros pcl_conversions quadrotor_msgs
  DEPENDS PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare C++ executables
add_executable(ego_controller_node
  src/ego_controller_node.cpp
)

target_link_libraries(ego_controller_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(cloud_extruder
  src/cloud_extruder.cpp
)

target_link_libraries(cloud_extruder
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS ego_controller_node cloud_extruder
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Mark other files for installation
install(DIRECTORY shell/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/shell
)

## Mark Python scripts for installation
catkin_install_python(PROGRAMS
  scripts/test_send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
