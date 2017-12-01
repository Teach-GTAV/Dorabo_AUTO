#include "local_planner.h"

using namespace local_planner;

LocalPlanner::LocalPlanner(ros::NodeHandle nh)
    : nh_(nh)
{
    InitParameters();

    subGridmap_     =   nh_.subscribe(strGridmap_, 1, &LocalPlanner::gridmapCB,  this);
    subTarget_      =   nh_.subscribe(strTarget_ , 1, &LocalPlanner::targetCB ,  this);
    pubCandPath_    =   nh_.advertise<nav_msgs::Path> (strCandPath_, 1);
    pubOptiPath_    =   nh_.advertise<nav_msgs::Path> (strOptiPath_, 1);
}

void LocalPlanner::InitParameters()
{

    nh_.param("candidit_path",          strCandPath_,          string("/candidate_path"));
    nh_.param("optimal_path",           strOptiPath_,          string("/optimal_path"));
    nh_.param("gridmap",                strGridmap_,           string("/center_mapping/center_raw"));
    nh_.param("targetPoint",            strTarget_,            string("/map_segmenter/TargetPoint"));
    nh_.param("object_threshold",       obstacle_threshold_,   double(0.3));
    nh_.param("curvature_threshold",    curvature_threshold_,  double(2));

}

void LocalPlanner::gridmapCB(const grid_map_msgs::GridMap& msg)
{
    GridMapRosConverter::fromMessage(msg, map_);
}

void LocalPlanner::targetCB(const geometry_msgs::PointStampedConstPtr& msg)
{
    missionTarget_.x = msg->point.x;
    missionTarget_.y = msg->point.y;

    stamp_ = msg->header.stamp;
    PathGeneration();
    PathEluavation();
}

void LocalPlanner::PathGeneration()
{
    trajs_.clear();
    trajs_= BezierGeneration(missionTarget_);
    output_candidates();
}

void LocalPlanner::PathEluavation()
{
    Arbitor();
    ROS_INFO("ok");
    int pathID = Optimal();
    if (pathID == -1)
    {
        ROS_INFO("no path");
    }
    else
    {
        bestPath_ = trajs_[pathID].path;
        output_optimal();
    }

}

void LocalPlanner::output_candidates()
{
    generatedPath_.poses.clear();
    for(size_t i = 0 ; i < trajs_.size(); i++)
    {
        if(trajs_[i].cost_sum >= 300) continue;

        Path path = trajs_[i].path;
        for(size_t j = 0; j < path.poses.size(); j++)
        {
            generatedPath_.poses.push_back(path.poses[j]);
        }
    }
    generatedPath_.header.frame_id = "vehicle";
    generatedPath_.header.stamp = stamp_;
    pubCandPath_.publish(generatedPath_);
}

void LocalPlanner::output_optimal()
{

    bestPath_.header.frame_id = "vehicle";
    bestPath_.header.stamp = stamp_;
    pubOptiPath_.publish(bestPath_);
}

void LocalPlanner::Arbitor()
{

    double min_curve = 10000;
    int curve_id = 0;

    double variance_min = 10000;
    double variance_max = 0;
    double variance_diff = 1.0;
    double obstacle_min = 10000;
    double obstacle_max = 0;
    double obstacle_diff = 1.0;
    double curve_min    = 10000;
    double curve_max    = 0;
    double curve_diff   = 1.0;
    double distance_min = 10000;
    double distance_max = 0;
    double distance_diff = 1.0;


    for(size_t i = 0; i < trajs_.size(); i++)
    {
        auto& traj = trajs_[i];
        traj.obstacle_cost = 0.0;
        traj.obstacle_fail = 0;
        double Ex2 = 0.0;
        double Ex = 0.0;

        int count=1;
        for(size_t j = 0; j < traj.path.poses.size(); j++)
        {
            Point point = traj.path.poses[j].pose.position;
            //! map travel cost ----------------------------------------------------------//
            Index index;
            grid_map::Position position(point.x, point.y);
            if (!map_.getIndex(position, index))
            {
                traj.obstacle_cost += 1.0;
            }
            else
            {

                double road_cost = 0.0;
                int circle_count = 0;
                for (CircleIterator circleIt(map_, position, 2);
                     !circleIt.isPastEnd(); ++circleIt)
                {
                    circle_count++;
                    grid_map::Position currentPositionInCircle;
                    map_.getPosition(*circleIt, currentPositionInCircle);

                    grid_map::Index circleIndex;
                    if(!map_.getIndex(currentPositionInCircle, circleIndex))
                    {
                        road_cost += 0.5;
                    }
                    else
                    {
                        road_cost += map_.at("road", circleIndex);
                        if (map_.at("road", circleIndex) >= 1)
                            traj.obstacle_fail = 1;
                    }

                }
                road_cost /= circle_count;
                traj.obstacle_cost += fabs(road_cost+1.7);
                count++;
                Ex2 += pow(road_cost,2);
                Ex  += road_cost;
            }
        }
        traj.variance_cost = Ex2/count - pow(Ex/count,2);

        traj.variance_cost = 1.0/(1.0+exp(-traj.variance_cost));
        traj.obstacle_cost = 1.0/(1.0+exp(-traj.obstacle_cost));

        variance_max = variance_max > traj.variance_cost ? variance_max:traj.variance_cost;
        variance_min = variance_min < traj.variance_cost ? variance_min:traj.variance_cost;
        variance_diff = variance_max - variance_min;
        obstacle_max = obstacle_max > traj.obstacle_cost ? obstacle_max:traj.obstacle_cost;
        obstacle_min = obstacle_min < traj.obstacle_cost ? obstacle_min:traj.obstacle_cost;
        obstacle_diff = obstacle_max - obstacle_min;
        curve_max = curve_max > traj.curve_cost ? curve_max:traj.curve_cost;
        curve_min = curve_min < traj.curve_cost ? curve_min:traj.curve_cost;
        curve_diff = curve_max - curve_min;
        distance_max = distance_max > traj.distance ? distance_max:traj.distance;
        distance_min = distance_min < traj.distance ? distance_min:traj.distance;
        distance_diff = distance_max - distance_min;



        if (traj.curve_cost <= min_curve)
        {
            min_curve = traj.curve_cost;
            curve_id = i;
        }
    }

    for(size_t i = 0; i < trajs_.size(); i++)
    {
        auto& traj = trajs_[i];
        traj.variance_cost = (traj.variance_cost-variance_min)/variance_diff;
        traj.obstacle_cost = (traj.obstacle_cost - obstacle_min )/obstacle_diff;
        traj.curve_cost    = (traj.curve_cost - curve_min)/curve_diff;
        traj.distance      = (traj.distance - distance_min)/distance_diff;



        if (i < curve_id)
        {
            traj.curve_cost += cost_.cur_diff;
        }

        traj.cost_sum = traj.obstacle_cost  *   cost_.obj_cost +
                        traj.curve_cost     *   cost_.cur_cost +
                        traj.variance_cost  *   cost_.var_cost +
                        traj.distance       *   cost_.dst_cost;

//        if(traj.curve_fail == 1 || traj.obstacle_fail == 1) traj.cost_sum = 100000;

        std::cerr << "  obstacle: "<<traj.obstacle_cost
                  << "  variance  "<<traj.variance_cost
                  << "  curve: "   <<traj.curve_cost
                  << "  disance: " <<traj.distance
                  << "  cost:    " <<traj.cost_sum<< std::endl;

    }
}

int LocalPlanner::Optimal()
{
    double min_traj = 100000000.0;
    int id = -1;
    for(size_t i = 0 ; i < trajs_.size(); i++)
    {
        if(trajs_[i].cost_sum >= 300) continue;

        if (trajs_[i].cost_sum < min_traj)
        {
            min_traj = trajs_[i].cost_sum;
            id = i;
        }
    }
    return id;
}
void LocalPlanner::cfgCallback(const CostThreshold cfg)
{
    cost_ = cfg;
}
