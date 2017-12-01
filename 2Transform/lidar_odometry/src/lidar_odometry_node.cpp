#include <ros/ros.h>
#include "lidar_odometry.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_odometry");
    ros::NodeHandle nh("~");

    lidar_odometry::odometry lo(nh);

    ros::spin();

    return 0;
}
