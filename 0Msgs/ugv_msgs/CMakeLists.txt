cmake_minimum_required(VERSION 2.8)
project (ugv_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs nav_msgs)

add_message_files(
  DIRECTORY msg
  FILES
	NCOM.msg
	Grid.msg
	Map.msg
	Gpfpd.msg
	Gpimu.msg
	VehiclePose.msg
	LmsStatus.msg
	Velodyne64Packet.msg
	Velodyne32Packet.msg
	VelodynePacket.msg
	VerticalMeasurements.msg
	CandidateTrajectories.msg
	CubicTrajectory.msg
	ExecutionTrajectory.msg
	MissionTargets.msg
	Target.msg
	VehicleStatus.msg
        graph.msg
)

add_service_files(
  DIRECTORY srv
  FILES 
        Mappingservice.srv
        Roadcentralline.srv
)


generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs geometry_msgs nav_msgs
)
