#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <robot_center_mapping/center_mappingConfig.h>
#include "robot_center_mapping.h"

Parameter param;


void Callback(robot_center_mapping::center_mappingConfig &config, uint32_t level)
{
    param.lower           = config.lower;
    param.upper           = config.upper;
    param.occupancy_max   = config.occupancy_max;
    param.occupancy_min   = config.occupancy_min;
    param.distance        = config.distance;

    param.sigma           = config.sigma;
    param.weight          = config.weight;
    param.radius          = config.radius;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_center_mapping");
    ros::NodeHandle nh("~");
    robot_center_mapping::Center_map map(nh);

    dynamic_reconfigure::Server<robot_center_mapping::center_mappingConfig> server;
    dynamic_reconfigure::Server<robot_center_mapping::center_mappingConfig>::CallbackType f;

    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        map.cfgCallback(param);
        map.output();
        loop.sleep();
    }

    return 0;
}
