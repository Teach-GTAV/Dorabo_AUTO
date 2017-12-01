cmake_minimum_required(VERSION 2.8.3)
project(inertial_tf)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  ugv_msgs
  geometry_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES inertial_tf
  CATKIN_DEPENDS roscpp sensor_msgs tf ugv_msgs geometry_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(inertial_tf_node 
	src/inertial_tf_node.cpp
	src/inertial_tf.cpp
	include/inertial_tf.h)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(inertial_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(inertial_tf_node
   ${catkin_LIBRARIES}
 )
