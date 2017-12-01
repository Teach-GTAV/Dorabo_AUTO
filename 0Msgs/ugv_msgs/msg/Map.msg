Header header
###########################################################
#Elevation Map Configuration
#half size of the map;
int32 nHalfSize
#Elevation Grid Resolution, in meter;
float32 fResolution
#x coordinate of the center of the map in world frame
float32 fCenterX
#y coordinate of the center of the map in world frame
float32 fCenterY
#maximum elevation of the map in world frame
float32 fMapMaxElevation
#minimum elevation of the map in world frame
float32 fMapMinElevation
#flag of image
int8 flagImage
###########################################################

###########################################################
#Vehicle Status
#vehicle x in world frame, in meters
float32 fVehicleX
#vehicle y in world frame, in meters
float32 fVehicleY
#vehicle z in world frame, in meters
float32 fVehicleZ
#vehicle heading angle, in rad
float32 fVehicleHeading
###########################################################
Grid[] map
