#pragma once

#include <ros/ros.h>
#include <iostream>

#include <geometry_msgs/PointStamped.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
// Msg
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "arbiter.h"
#include "bezier.h"

namespace local_planner
{

using namespace std;
using namespace grid_map;

struct CostThreshold
{
    double obj_cost;
    double cur_cost;
    double dst_cost;
    double var_cost;
    double cur_diff;
};

class LocalPlanner
{
public:
    //! msg
    grid_map::GridMap map_;
    vector<Traj> trajs_;

    //! Cost Threshold
    CostThreshold cost_;

    //! MissionTarget
    Point missionTarget_;
    nav_msgs::Path generatedPath_;
    nav_msgs::Path bestPath_;
    ros::Time stamp_;

public:

    LocalPlanner(ros::NodeHandle nh);

    virtual ~LocalPlanner (void){}

    void InitParameters(void);

    void PathGeneration(void);
    void PathEluavation(void);
    void cfgCallback(const CostThreshold cfg);

    bool cmp(Traj a, Traj b)
    {
        return a.cost_sum <= b.cost_sum;
    }

    /**
     * @brief Determain the cost of each beziar trajectory
     * @param none
     * @note  none
     */
    void Arbitor(void);

    /**
     * @brief Take the path with minimum cost
     * @param none
     * @note  none
     */
    int Optimal(void);

    /**
     * @brief output the canditant trajectories
     * @param
     * @note
     */
    void output_candidates();

    /**
     * @brief output the optimal trajectories
     * @param
     * @note
     */
    void output_optimal();
    void targetCB(const geometry_msgs::PointStampedConstPtr &msg);
    void gridmapCB(const grid_map_msgs::GridMap &msg);
    void gridMap_tragetPoint_CB(const grid_map_msgs::GridMap& gridMap_msg,const geometry_msgs::PointStampedConstPtr &targetPoint_msg);

private:


    ros::NodeHandle nh_;
    tf::TransformListener tf_listen_;

    //! ros pub & sub
    ros::Subscriber subGridmap_;
    ros::Subscriber subTarget_;
    ros::Publisher  pubCandPath_;
    ros::Publisher  pubOptiPath_;

    //! string
    string strGridmap_;
    string strTarget_;
    string strCandPath_;
    string strOptiPath_;

    //! double
    double obstacle_threshold_;
    double curvature_threshold_;


};

}
