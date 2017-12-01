cmake_minimum_required(VERSION 2.8.3)
project(local_planner)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  grid_map_cv
  grid_map_msgs
  grid_map_ros
  roscpp
  rospy
  std_msgs
  tf
  tf_conversions
  ugv_msgs
  message_filters
  geometry_msgs
)

generate_dynamic_reconfigure_options(
        cfg/local_planner.cfg
)


find_package(Boost REQUIRED COMPONENTS system)
catkin_package()
include_directories(
  include
  include/lib
  ${catkin_INCLUDE_DIRS}
)

add_executable(local_planner_node
    src/local_planner_node.cpp

    src/local_planner.cpp
    src/lib/bezier.cpp

    include/local_planner.h
    include/lib/bezier.h
)

add_dependencies(local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(local_planner_node
   ${catkin_LIBRARIES}
)
