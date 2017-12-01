cmake_minimum_required(VERSION 2.8.3)
project(lidar_odometry)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  ugv_msgs
  tf
  tf_conversions
)


find_package(Eigen3 REQUIRED)

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(lidar_odometry_node
    src/lidar_odometry_node.cpp
    src/lidar_odometry.cpp
    include/lidar_odometry.h
    )

add_dependencies(lidar_odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(lidar_odometry_node
  ${catkin_LIBRARIES}
)
