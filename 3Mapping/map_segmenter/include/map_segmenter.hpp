#include <ros/ros.h>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ugv_msgs/graph.h>

// OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

// MSG
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

// TF
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/exceptions.h>
#include <tf_conversions/tf_eigen.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

#include <dynamic_reconfigure/server.h>
#include <map_segmenter/map_segConfig.h>

#define DegToRad M_PI/180
#define RadToDeg 180/M_PI

#define GRAVITY 9.8035
#define EARTH_R 6367560

#define MINARCLEN 30
#define CLOSELINEMAXDIST 10

typedef pcl::PointXYZ    PointT;
typedef pcl::PointXYZRGB PointCT;
typedef pcl::PointCloud<PointT> Cloud;
typedef pcl::PointCloud<PointCT> ColorCloud;


namespace map_segmenter {

using namespace std;
using namespace cv;

struct Junction
{
    int     index;
    int     num;
    bool    junc;
    vector<int> nn_index;
};


struct Target
{
    int nID;
    int nType;
    double fLatitude;
    double fLongitude;
    float fAltitude;
};

struct Parameter
{
    int    resize;
    int    closing;
    int    openning;
    int    thinning;
    int    filter;
    int    filter_mode;

    int    binary;
    int    binary_type;

    int    width;
    int    grey;
};

struct Middle_point
{
    int pose;
    int val;
};

struct Edge
{
    int current_point;
    int new_point;
};

struct Pearls
{
    float dist;
    int   id;
};

class MapSeg{

public:

    Parameter param_;

    /*!
     * Constructor.
     */
    MapSeg(ros::NodeHandle nodehandle);

    /*!
     * Deconstructor.
     */
    virtual ~MapSeg(){}

    /*!
     * Callback method for the incoming img.
     */
    void mapCallback(const sensor_msgs::ImageConstPtr &img);

    /*!
     * Callback method for the vehicle pose
     */
    void poseCallback(const sensor_msgs::NavSatFixPtr &gps);

    void cfgCallback(Parameter &param);

    void globalPose_pub(void);

private:

    /*!
     * Read parameters.
     */
    bool readParameters();

    bool readMissionFile();

    //! ROS node
    ros::NodeHandle rosNode_;

    //! ROS publisher
    ros::Publisher pubPoint_;
    ros::Publisher PubRoadGraph_;

    //! ROS subscriber
    ros::Subscriber subPose_;

    //! Image transport node
    image_transport::ImageTransport imgNode_;

    //! Image transport subscrib
    image_transport::Subscriber subImg_;

    //! Image transport publisher
    image_transport::Publisher pubLine1_;
    image_transport::Publisher pubLine2_;
    image_transport::Publisher pubLine3_;
    image_transport::Publisher pubLine4_;
    image_transport::Publisher pubImgRoad_;

    //! Parameters
    string strSubImg_;
    string strSubPose_;
    string strPubImg_;
    string strPubOccupancy_;
    string strPubPoint_;
    string strMissionFile_;
    string strPubroadgraph_;

    //! Target
    vector<Target> targets_;
    int current_target_;
    geometry_msgs::PointStamped gps_target_;
    geometry_msgs::PointStamped gps_local_;
    geometry_msgs::PoseStamped mission_target_;
    geometry_msgs::PoseArray global_points_;

    tf::TransformListener transformListener_;
    ugv_msgs::graph RoadGraph_;

};

}
