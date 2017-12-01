#ifndef LIDAR_ODOMETRY_H_
#define LIDAR_ODOMETRY_H_

#include <ros/ros.h>

// Msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/NavSatFix.h>
#include <ugv_msgs/VelodynePacket.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>


// Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <fstream>
#include <Eigen/Dense>
#include <iostream>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> Cloud;

typedef pcl::PointXYZ PointType;
typedef pcl::PointXY PointPlane;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointType> PointCloud;
typedef pcl::PointCloud<int> PointCloudIndex;
typedef pcl::PointCloud<PointType>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointType>::ConstPtr PointCloudConstPtr;

using namespace std;

namespace lidar_odometry {

struct regist_trans
{
    Eigen::Affine3f transform;
    double score;
};

class odometry
{
public:
    odometry(ros::NodeHandle nodehandle);

    virtual ~odometry() {}

    void lidarCallback(const sensor_msgs::PointCloud2ConstPtr& cloud);

    void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps);

    void init();

    void publish();

    Eigen::Matrix4f regist_icp();

    void voxel_filter();

private:
    void readParameters();

    ros::NodeHandle nh_;

    ros::Publisher pub_map_;
    ros::Publisher pub_sweep_;
    ros::Publisher pub_sweep_last_;

    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_gps_;

    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener listener_;
    tf::Transform tf_t_;
    tf::Quaternion tf_q_;

    string strSubCloudName_;
    string strPubGlobalMapName_;
    string strPubSweepMapName_;
    string strPubLastSweepMapName_;
    string strSubGpsName_;

    PointCloudPtr last_sweep_;
    PointCloudPtr current_sweep_;
    PointCloudPtr global_cloud_;

    sensor_msgs::PointCloud2 pub_SweepCloud_;
    sensor_msgs::PointCloud2 pub_GlobaCloud_;
    sensor_msgs::PointCloud2 pub_LastSweepCloud_;

    regist_trans s2s_;

    bool gps_valid_;

    double voxel_grid_;
};

}

#endif
