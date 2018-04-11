cmake_minimum_required(VERSION 2.8.3)
project(aero_maps)
find_package(catkin REQUIRED std_msgs geometry_msgs)
find_package(Boost REQUIRED COMPONENTS filesystem system signals thread)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

catkin_python_setup()

add_message_files(
  FILES
  IndexedPoseStamped.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package()
