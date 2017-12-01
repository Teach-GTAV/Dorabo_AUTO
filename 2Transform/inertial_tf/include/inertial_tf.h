#pragma once

#include <ros/ros.h>
#include <ros/console.h>

// TF
#include <tf/transform_broadcaster.h>
//#include <tf/LinearMath/QuadWord.h>
//#include <tf/LinearMath/Quaternion.h>

// MSG
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <ugv_msgs/NCOM.h>

#define DegToRad M_PI/180
#define RadToDeg 180/M_PI

#define GRAVITY 9.8035
#define EARTH_R 6367560

namespace inertial_tf {

class Transform{

public:
    /*!
     * Constructor.
     */
    Transform(ros::NodeHandle nodehandle);

    /*!
     * Destructor.
     */
    virtual ~Transform() {}

    /*!
     * Read the inertial_tf node parameters.
     */
    void readparameters();

    /*!
     * Initialize the message data.
     */
    void initializer();

    /*!
     * Callback method for the updated GNSS data.
     * @param msg: GNSS data.
     */
    void dataCallback(const ugv_msgs::NCOMConstPtr& msg);

    /*!
     * Publish the message data.
     * @param msg_imu_, msg_gps_, msg_pose_.
     */
    void publish();

private:

    //! ROS nodehandle
    ros::NodeHandle nodehandle_;

    //! ROS publisher
    ros::Publisher pub_Imu_;
    ros::Publisher pub_GPS_;
    ros::Publisher pub_Pos_;

    //! ROS subscribers
    ros::Subscriber sub_inertial_;

    //! sensor message
    sensor_msgs::Imu            msg_imu_;
    sensor_msgs::NavSatFix      msg_gps_;

    //! geometry message
    geometry_msgs::PoseStamped msg_pos_;

    //! TF
    tf::TransformBroadcaster tf_broadcaster_;
    tf::Transform tf_t_;
    tf::Quaternion tf_q_;

    //! parameters for initial GPS
    struct initial_gps{
        int flag;
        float latitude;
        float longitude;
        float altitude;
    }init_gps_;


};
}
