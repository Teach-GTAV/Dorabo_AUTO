#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

// GRID_MAP
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

// Msg
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

// OPENCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

// Filters
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf_conversions/tf_eigen.h>

// Kindr
#include <kindr/poses/Pose.hpp>
#include <kindr/phys_quant/PhysicalQuantities.hpp>
#include <kindr/rotations/Rotation.hpp>
#include <kindr/Core>
#include <kindr/common/common.hpp>
#include <kindr_ros/kindr_ros.hpp>
#include <kindr_ros/RosGeometryMsgRotation.hpp>

typedef Eigen::Array2i Index;


typedef pcl::PointXYZ PointT;
typedef pcl::PointXY PointPlane;
typedef pcl::PointNormal PointN;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointCloud<int> PointCloudIndex;
typedef pcl::PointCloud<PointT>::Ptr PointCloudPtr;
typedef pcl::PointCloud<PointT>::ConstPtr PointCloudConstPtr;
typedef pcl::VoxelGrid<PointT> VoxelGrid;

struct Parameter
{
    double    occupancy_min;
    double    occupancy_max;
    double    upper;
    double    lower;

    double distance;

    //! object param
    double    sigma;
    double    weight;
    double    radius;
};

namespace robot_center_mapping {

using namespace std;
using namespace grid_map;
using namespace tf;
using namespace cv;
using namespace pcl;

template<typename Scalar>
struct VarianceClampOperator
{
  VarianceClampOperator(const Scalar& minVariance, const Scalar& maxVariance)
      : minVariance_(minVariance),
        maxVariance_(maxVariance)
  {
  }
  const Scalar operator()(const Scalar& x) const
  {
    return x < minVariance_ ? minVariance_ : (x > maxVariance_ ? std::numeric_limits<float>::infinity() : x);
  }
  Scalar minVariance_, maxVariance_;
};

class Center_map{
public:
    /*!
     * Constructor.
     */
    Center_map (ros::NodeHandle nodehandle);

    /*!
     * Deconstructor.
     */
    virtual ~Center_map () {}

    /*!
     * Set the geometry of the elevation map. Clears all the data.
     * @param length the side lengths in x, and y-direction of the elevation map [m].
     * @param resolution the cell size in [m/cell].
     * @param postion the 2d position of the elevation map in the elevation map frame [m].
     * @return true if sucessful
     */
    void setGeometry(const Eigen::Array2d& length, const double& resoltuion, const Eigen::Vector2d& position);

    /*!
     * Clears all data of the elevation map (data and time).
     * @return true if successful.
     */
    bool clear();

    /*!
     * Callback method for the updates of the underlying map.
     * Updates the map from the vehicle frame.
     * @param in the plane, red is the plane and green is the object
     */
    void planeCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);


    void backObCb     (const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
    void frotObCb     (const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    /*!
     * Callback method for the updates of the underlying object.
     * Updates the map from the vehicle frame.
     * @param
     */
    void cfgCallback(const Parameter cfg);

    /*!
     * Callback method for the updates of the underlying object.
     * Updates the map from the vehicle frame.
     * @param
     */
    void output();

private:

    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful
     */
    bool readParameters();

    /*!
     * Set the frame id.
     * @param frameId: the frame id.
     */
    void setFrameId (const std::string& frameId);

    /*!
     * Updates the location of the map to follow the tracking point. Takes care
     * of the data handling the goes along with the relocalization.
     * @return true if successful.
     */
    bool updateMapLocation();

    bool addPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool addBackObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool addFrotObject(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    bool filterElevationMap();

    // Caculate the transformation
    bool cacuTransform();

    // Caculate the variance
    bool cacuVariance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

    //! ROS nodehandle
    ros::NodeHandle nodehandle_;

    //! Raw elevation map as grid map;
    grid_map::GridMap rawMap_;

    //! Fused elevation map as grid map;
    grid_map::GridMap fusedMap_;

    //! ROS publishers
    ros::Publisher MapRawPublisher_;
    ros::Publisher CostmapPublisher_;
    ros::Publisher OccupPublisher_;
    ros::Publisher RoadPublisher_;
    ros::Publisher ObjectPublisher_;
    ros::Publisher FusionPublisher_;

    //! ROS subscribers
    ros::Subscriber subPlane_;
    ros::Subscriber subFrotObject_;
    ros::Subscriber subBackObject_;

    //! TF listener.
    tf::TransformListener transformListener_;

    //! Point which the elevation map follows.
    kindr::Position3D trackPoint_;
    std::string trackPointFrameId_;

    //! Rotation from Base to Sensor frame (C_SB)
    kindr::RotationMatrixPD rotationBaseToSensor_;

    //! Translation from Base to Sensor in Base frame (B_r_BS)
    kindr::Position3D translationBaseToSensorInBaseFrame_;

    //! Rotation from (elevation) Map to Base frame (C_BM)
    kindr::RotationMatrixPD rotationMapToBase_;

    //! Translation from Map to Base in Map frame (M_r_MB)
    kindr::Position3D translationMapToBaseInMapFrame_;

    //! Transformation from Sensor to Map frame
    Eigen::Affine3d transformationSensorToMap_;

    Eigen::Matrix<double, 6, 6> robotPoseCovariance_;

    //! Mutex lock for raw map.
    boost::recursive_mutex rawMapMutex_;

    //! Parameters. Are set throuh the EvelationMapping class.
    double minVariance_;
    double maxVariance_;
    double mahalanobisDistanceThreshold_;
    double multiHeightNoise_;
    double minHorizontalVariance_;
    double maxHorizontalVariance_;
    double surfaceNormalEstimationRadius_;
    double mapLength_;
    double mapResolution_;

    bool   bridge_flag_;

    Eigen::Vector3d surfaceNormalPositiveAxis_;
    Eigen::VectorXf measurementVariance_;
    std::string strBackObject_;
    std::string strFrotObject_;
    std::string strPlaneTopic_;


    std::string pubRawMap_;
    std::string pubCostmap_;
    std::string pubRoad_;
    std::string pubObject_;
    std::string pubFusion_;
    std::string pubOccupancy_;
    std::string sensorFrameId_;
    std::string mapFrameId_;
    std::string robotBaseFrameId_;

    double filter_radius_;
    double block_circle_;
    double obstacleHeight_;

    ros::Time stamp_;
    Parameter cfg_;
};

}
