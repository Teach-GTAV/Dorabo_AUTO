#include <ros/ros.h>
#include <iostream>
#include "local_planner.h"
#include <dynamic_reconfigure/server.h>
#include <local_planner/LocalPlannerConfig.h>
using namespace local_planner;
using namespace message_filters;
CostThreshold costParam;
void Callback(local_planner::LocalPlannerConfig &config, uint32_t level)
{
    costParam.cur_cost = config.cur_cost;
    costParam.dst_cost = config.dst_cost;
    costParam.obj_cost = config.obj_cost;
    costParam.var_cost = config.var_cost;
    costParam.cur_diff = config.cur_diff;
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_planner");
    ros::NodeHandle nh("~");
    LocalPlanner planner(nh);
//    message_filters::Subscriber<grid_map_msgs::GridMap> gridMap(nh, "/center_mapping/center_raw", 1);
//    message_filters::Subscriber<geometry_msgs::PointStamped> tragetPoint(nh,"/map_segmenter/TargetPoint" , 1);
//    typedef sync_policies::ApproximateTime<grid_map_msgs::GridMap, geometry_msgs::PointStamped> MySyncPolicy;
//    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), gridMap, tragetPoint);
//    sync.registerCallback(boost::bind(&LocalPlanner::gridMap_tragetPoint_CB, &planner, _1, _2));
    dynamic_reconfigure::Server<local_planner::LocalPlannerConfig> server;
    dynamic_reconfigure::Server<local_planner::LocalPlannerConfig>::CallbackType f;
    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);
    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        planner.cfgCallback(costParam);
    }
    return 0;
}
