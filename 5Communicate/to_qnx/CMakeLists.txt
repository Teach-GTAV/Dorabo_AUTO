CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
project(to_qnx)

find_package(catkin REQUIRED COMPONENTS 
	dynamic_reconfigure
	roscpp 
	rospy 
	std_msgs 
        message_generation
)

 generate_dynamic_reconfigure_options(
   cfg/ToQNX.cfg
 )

catkin_package(
  LIBRARIES to_qnx
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy std_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
)

add_executable(to_qnx_node src/to_qnx_node.cpp)
add_dependencies(to_qnx_node ${PROJECT_NAME}_gencfg)
target_link_libraries(to_qnx_node
   ${catkin_LIBRARIES}
)

