#include "lidar_odometry.h"
#include "Eigen/Core"
#include "Eigen/Geometry"

using namespace lidar_odometry;
using namespace Eigen;
using namespace std;

odometry::odometry(ros::NodeHandle nodehandle)
    : nh_(nodehandle)
{
    readParameters();

    init();

    pub_map_        = nh_.advertise<sensor_msgs::PointCloud2>(strPubGlobalMapName_,     1);
    pub_sweep_      = nh_.advertise<sensor_msgs::PointCloud2>(strPubSweepMapName_,      1);
    pub_sweep_last_ = nh_.advertise<sensor_msgs::PointCloud2>(strPubLastSweepMapName_,  1);

    sub_cloud_      = nh_.subscribe(strSubCloudName_, 1, &odometry::lidarCallback, this);
    sub_gps_        = nh_.subscribe(strSubGpsName_,   2, &odometry::gpsCallback,   this);

}

void odometry::readParameters()
{
    nh_.param("subCloud",       strSubCloudName_,           string("/cloud"));
    nh_.param("subGps",         strSubGpsName_,             string("/inertial_tf/gps"));

    nh_.param("pubGlobalMap",   strPubGlobalMapName_,       string("/global_map"));
    nh_.param("pubSweep",       strPubSweepMapName_,        string("/current_sweep"));
    nh_.param("pubLastSweep",   strPubLastSweepMapName_,    string("/last_sweep"));

    nh_.param("voxel_filter",   voxel_grid_,                double(1));
}

void odometry::init()
{

    s2s_.score = 0.0;
    s2s_.transform = Eigen::Affine3f::Identity();

    gps_valid_ = 0.0;
}

Eigen::Matrix4f odometry::regist_icp()
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    PointCloudPtr temp_source (new PointCloud);
    PointCloudPtr temp_target (new PointCloud);
    voxel.setLeafSize(voxel_grid_, voxel_grid_, voxel_grid_);
    voxel.setInputCloud(current_sweep_);
    voxel.filter(*temp_source);
    voxel.setInputCloud(last_sweep_);
    voxel.filter(*temp_target);

    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(temp_source);
    icp.setInputTarget(temp_target);
    icp.setMaxCorrespondenceDistance(1.0);
    icp.setTransformationEpsilon(1e-16);
    icp.setEuclideanFitnessEpsilon(1e-8);
    PointCloudPtr final (new PointCloud);
    icp.align(*final);

    Eigen::Matrix4f trans, icp_temp;
    icp_temp = icp.getFinalTransformation();
    trans.setZero(4,4);
    trans(0,3) = icp_temp(0,3);
    trans(1,3) = icp_temp(1,3);
    trans(2,3) = 0.0;
    trans(0,0) = 1.0;
    trans(1,1) = 1.0;
    trans(2,2) = 1.0;
    trans(3,3) = 1.0;
//    std::cout << icp_temp << std::endl<<std::endl;
    return trans;
}

void odometry::gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps)
{
    if(gps->status.status >0)
        gps_valid_ = 1;
    else
        gps_valid_ = 0;
}

void odometry::lidarCallback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    static double msg_time = 0.0;
    double temp_time1 = cloud->header.stamp.toSec() - msg_time;
    msg_time = cloud->header.stamp.toSec();

    double temp_time2 = ros::Time::now().toSec();

    PointCloudPtr pointCloud (new PointCloud);
    pcl::fromROSMsg(*cloud, *pointCloud);
    // 1. read the pointcloud and matching with the previous one
    static int flag = 0;
    if (flag ==0)
    {
        last_sweep_    = pointCloud;
        global_cloud_  = pointCloud;
        flag = 1;
        return;
    }
    current_sweep_ = pointCloud;

    tf::StampedTransform transform;
    try{
      listener_.lookupTransform("/world", "/vehicle",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    Eigen::Matrix3d temp_matrix;
    tf::Matrix3x3 temp_tf_matrix;
    Eigen::Vector3d temp_vector;
    tf::Vector3 temp_tf_vector;
    temp_tf_matrix.setRotation(transform.getRotation());
    temp_tf_vector = transform.getOrigin();
    tf::matrixTFToEigen(temp_tf_matrix, temp_matrix);
    tf::vectorTFToEigen(temp_tf_vector, temp_vector);

    Eigen::Matrix4f tf_trans;
    tf_trans.matrix() << temp_matrix(0,0), temp_matrix(0,1), temp_matrix(0,2), 0.0,
                               temp_matrix(1,0), temp_matrix(1,1), temp_matrix(1,2), 0.0,
                               temp_matrix(2,0), temp_matrix(2,1), temp_matrix(2,2), 0.0,
                               0, 0, 0, 1;

//    if (gps_valid_)
//    {
//        tf_trans(0,3) = temp_vector(0);
//        tf_trans(1,3) = temp_vector(1);
//        tf_trans(2,3) = temp_vector(2);
//        s2s_.transform = tf_trans;
//    }
//    else
    {
        Eigen::Affine3f a;
        a.matrix() = tf_trans;
        pcl::transformPointCloud(*current_sweep_, *current_sweep_, a);

        Eigen::Matrix4f icp_trans = regist_icp();
        temp_time2 = ros::Time::now().toSec() - temp_time2;
        double cost = temp_time2/temp_time1;

        icp_trans(0,3) = icp_trans(0,3)*cost;
        icp_trans(1,3) = icp_trans(1,3)*cost;
        s2s_.transform = s2s_.transform * icp_trans;
//        std::cout << s2s_.transform.matrix() << std::endl<<std::endl;
    }

    PointCloudPtr temp (new PointCloud);
//    pcl::transformPointCloud(*current_sweep_, *temp, s2s_.transform);

    *global_cloud_ = *current_sweep_;

    Eigen::Matrix4f trans = s2s_.transform.matrix();

    temp_matrix  <<      trans(0,0), trans(0,1), trans(0,2),
                         trans(1,0), trans(1,1), trans(1,2),
                         trans(2,0), trans(2,1), trans(2,2);
    tf::matrixEigenToTF(temp_matrix, temp_tf_matrix);
    temp_vector <<   trans(0,3), trans(1,3), trans(2,3);
    tf::vectorEigenToTF(temp_vector, temp_tf_vector);

//    std::cout << temp_vector << std::endl << std::endl;

    tf_t_.setOrigin(temp_tf_vector);
    tf_t_.setRotation(transform.getRotation());
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, cloud->header.stamp, "/world", "/lidar_odometry"));

    tf_q_.setRPY(0.0, 0.0, 0.0);
    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, cloud->header.stamp, "/world", "/lidar_odometry_map"));

    pcl::toROSMsg(*current_sweep_, pub_SweepCloud_);
    pcl::toROSMsg(*last_sweep_, pub_LastSweepCloud_);
    pcl::toROSMsg(*global_cloud_, pub_GlobaCloud_);
    pub_GlobaCloud_.header.frame_id = "lidar_odometry_map";
    pub_SweepCloud_.header.frame_id = "lidar_odometry_map";
    pub_LastSweepCloud_.header.frame_id = "lidar_odometry_map";

    publish();
    last_sweep_ = current_sweep_;
}
\
void odometry::publish()
{
    // TF for sweep to global map matching
    // 1. world->sweep * sweep->sweep * sweep->world

    pub_map_.publish(pub_GlobaCloud_);
    pub_sweep_.publish(pub_SweepCloud_);
    pub_sweep_last_.publish(pub_LastSweepCloud_);



}
