cmake_minimum_required(VERSION 3.0.2)
project(gazebo_gripping_plugin)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  Boost REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  rospy
  std_msgs
  gazebo REQUIRED
  gazebo_ros REQUIRED
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES gazebo_gripping_plugin
 #CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs
 #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(gazebo_gripping_plugin SHARED
  src/gazebo_gripping_plugin.cpp
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

target_link_libraries(gazebo_gripping_plugin 
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES})

#############
## Install ##
#############
install(TARGETS gazebo_gripping_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
