cmake_minimum_required(VERSION 2.8.3)
project(aero_sensor_plate)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs geometry_msgs tf message_generation)

add_message_files(
  FILES
  BoolStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS roscpp sensor_msgs geometry_msgs tf message_generation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# frame publisher
add_executable(sensor_plate_frame_publisher
  src/frame_publisher/SensorPlateFramePublisher.cc)
target_link_libraries(sensor_plate_frame_publisher ${catkin_LIBRARIES})

# direction controller
add_executable(sensor_direction_control
  src/control_utils/sensordirectioncontrol_onmobilebase.cc)
target_link_libraries(sensor_direction_control ${catkin_LIBRARIES})

# tf to msg
add_executable(dynamic_tfstamped src/control_utils/dynamic_tfstamped.cc)
target_link_libraries(dynamic_tfstamped ${catkin_LIBRARIES})

# direction filter
add_executable(sensordirection_filter src/control_utils/sensordirection_filter.cc)
target_link_libraries(sensordirection_filter ${catkin_LIBRARIES})
add_dependencies(sensordirection_filter aero_sensor_plate_generate_messages_cpp)