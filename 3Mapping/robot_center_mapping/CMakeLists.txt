cmake_minimum_required(VERSION 2.8.3)
project(robot_center_mapping)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  tf
  tf_conversions
  eigen_conversions
  pcl_ros
  ugv_msgs
  kindr_ros
  dynamic_reconfigure
)
find_package(kindr)

generate_dynamic_reconfigure_options(
	cfg/center_mapping.cfg
)

catkin_package(
)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PkgConfig REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${kindr_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

 add_executable(robot_center_mapping_node
    src/robot_center_mapping_node.cpp
    src/robot_center_mapping.cpp
    include/robot_center_mapping.h
)

add_dependencies(robot_center_mapping_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


 target_link_libraries(robot_center_mapping_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
