cmake_minimum_required(VERSION 2.8.3)
project(map_segmenter)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  grid_map_cv
  grid_map_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  ugv_msgs
  pcl_ros
  pcl_conversions
  tf
  tf_conversions
  genmsg
  image_transport
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)

 generate_dynamic_reconfigure_options(
   cfg/map_seg.cfg
 )

catkin_package(
)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(map_segmenter_node 
	src/map_segmenter_node.cpp
	src/map_segmenter.cpp
        src/lib/Morphology.cpp
        #src/lib/math_function.cpp
        src/lib/util.cpp

	include/superpixel.hpp
	include/map_segmenter.hpp
	include/Morphology.h
        #include/math_function.h
)
target_link_libraries(map_segmenter_node
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES}
)
