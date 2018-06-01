cmake_minimum_required(VERSION 2.8.3)
project(aero_nonofficial_api)

option(DEBUG "debug" OFF)
option(GAZEBO "gazebo" OFF)

if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
add_definitions(-DINDIGO_)
endif()

# debug = small loop test                                                                                                                                     
if(DEBUG)
add_definitions(-DDEBUG_)
endif()

if(GAZEBO)
add_definitions(-DUSE_GAZEBO_SIM_)
endif()

find_package(catkin REQUIRED COMPONENTS
  aero_std
  aero_recognition_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aero_mpi_ aero_npi_
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_library(aero_mpi_
  lib/AeroMotionPlanningInterface.cc
)
target_link_libraries(aero_mpi_ ${catkin_LIBRARIES})

add_library(aero_npi_
  lib/NonPrehensileGraspInterface.cc
)
target_link_libraries(aero_npi_ ${catkin_LIBRARIES})