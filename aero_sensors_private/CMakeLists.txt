cmake_minimum_required(VERSION 2.8.3)
project(aero_sensors_private)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs tf)

catkin_package(
 INCLUDE_DIRS kinect/include
 LIBRARIES aerosensors_rotatekinectaddon
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf
)

include_directories(
  kinect/include
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_library(aerosensors_rotatekinectaddon kinect/src/RotateKinectAddon.cc)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(dynamic_kinect_frame_publisher
  kinect/dynamic_kinect_controller/DynamicKinectFramePublisher.cc
  )
target_link_libraries(dynamic_kinect_frame_publisher
  ${catkin_LIBRARIES})

add_executable(look_at_kinect kinect/src/LookAtKinect.cc)
target_link_libraries(look_at_kinect
  ${catkin_LIBRARIES} aerosensors_rotatekinectaddon)

add_executable(look_at_kinect2 kinect/src/LookAtKinect2.cc)
target_link_libraries(look_at_kinect2
  ${catkin_LIBRARIES} aerosensors_rotatekinectaddon)