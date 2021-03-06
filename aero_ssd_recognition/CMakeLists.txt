cmake_minimum_required(VERSION 2.8.3)
project(aero_ssd_recognition)

option(RGBA "point cloud type" ON)
if(RGBA)
  add_definitions(-DRGBA_)
endif()

OPTION(WITH_CUDA "Turn this option on to enable cuda support" OFF)
IF(WITH_CUDA)
 MESSAGE("Compiling with CUDA support")
 add_definitions(-DUSE_CUDA)
ENDIF(WITH_CUDA)

OPTION(DUMMY "Turn this option if you need only dummy mode" OFF)
IF(DUMMY)
  MESSAGE("Compiling for dummy mode")
ENDIF(DUMMY)

find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
  set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
endif()

find_package(catkin REQUIRED COMPONENTS std_msgs tf tf_conversions eigen_conversions cmake_modules)

IF(NOT DUMMY)
  find_package(catkin REQUIRED
    cv_bridge
    image_transport
    image_geometry
    compressed_image_transport
    aero_recognition_msgs
    dynamic_reconfigure
    jsk_topic_tools
    sensor_msgs
    geometry_msgs
    roscpp
    rospy
    nodelet
    pcl_conversions
    )

  find_package(PCL 1.7 REQUIRED)
  if("$ENV{ROS_DISTRO}" STREQUAL "indigo")
    find_package(OpenCV 2 REQUIRED)
    add_definitions(-DINDIGO_)
  elseif("$ENV{ROS_DISTRO}" STREQUAL "kinetic")
    find_package(OpenCV 3 REQUIRED)
    add_definitions(-DKINETIC_)
  endif()
  message("OPENCV ${OpenCV_VERSION} FOUND")

ELSE(NOT DUMMY)
find_package(catkin REQUIRED
  aero_recognition_msgs
  dynamic_reconfigure
  jsk_topic_tools
  sensor_msgs
  geometry_msgs
  roscpp
  rospy
  nodelet
  pcl_conversions
  )
ENDIF(NOT DUMMY)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/chainer/model_list.yaml
  COMMAND ./setup.sh ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)

add_custom_target(
  BuildDocs ALL
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/chainer/model_list.yaml
  )

find_package(Boost REQUIRED COMPONENTS filesystem system signals)
find_package(Eigen3 REQUIRED)

add_definitions(-std=c++11)

catkin_python_setup()

generate_dynamic_reconfigure_options(
  cfg/ObjectDetector.cfg
  cfg/Object3DProjector.cfg
)

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  tf
  tf_conversions
  eigen_conversions
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

IF(NOT DUMMY)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    )
ELSEIF(DUMMY)
  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS}
    )
ENDIF(NOT DUMMY)

macro(jsk_add_nodelet _nodelet_cpp _nodelet_class _single_nodelet_exec_name)
  if(NOT EXISTS "${PROJECT_SOURCE_DIR}/test/${_single_nodelet_exec_name}.test")
    list(APPEND ${PROJECT_NAME}_nodes_without_test ${_nodelet_cpp})
  endif()
  jsk_nodelet(${_nodelet_cpp} ${_nodelet_class} ${_single_nodelet_exec_name}
    ${PROJECT_NAME}_nodelet_sources ${PROJECT_NAME}_nodelet_executables)
endmacro()

IF(NOT DUMMY)
  jsk_add_nodelet(src/object_3d_projector.cpp "aero_ssd_recognition/Object3DProjector" "object_3d_projector")
ENDIF(NOT DUMMY)

jsk_add_nodelet(src/dummy/object_3d_projector.cpp "aero_ssd_recognition/DummyObject3DProjector" "dummy_object_3d_projector")

add_library(${PROJECT_NAME} SHARED ${aero_ssd_recognition_nodelet_sources})
IF(NOT DUMMY)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} yaml-cpp)
ELSEIF(DUMMY)
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
ENDIF(NOT DUMMY)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_gencpp)