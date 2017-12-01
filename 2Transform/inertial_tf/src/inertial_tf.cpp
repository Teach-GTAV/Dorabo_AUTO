#include "inertial_tf.h"


using namespace inertial_tf;

Transform::Transform(ros::NodeHandle nodehandle)
    : nodehandle_(nodehandle)
{
    readparameters();
    initializer();

    pub_Imu_        = nodehandle_.advertise<sensor_msgs::Imu>("imu", 1);
    pub_GPS_        = nodehandle_.advertise<sensor_msgs::NavSatFix>("gps", 1);
    pub_Pos_        = nodehandle_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    sub_inertial_   = nodehandle_.subscribe("/inertial", 1, &Transform::dataCallback, this);
}

void Transform::readparameters()
{

}

void Transform::initializer()
{
    init_gps_.flag      = 0;
    init_gps_.altitude  = 0.0;
    init_gps_.latitude  = 0.0;
    init_gps_.longitude = 0.0;

    msg_pos_.pose.position.x = 0;
    msg_pos_.pose.position.y = 0;
    msg_pos_.pose.position.z = 0;

    msg_pos_.pose.orientation.x = 0;
    msg_pos_.pose.orientation.y = 0;
    msg_pos_.pose.orientation.z = 0;
}

void Transform::dataCallback(const ugv_msgs::NCOMConstPtr &msg)
{
    if (msg->navStatus != 4 ) // Not been located, publish old data.
    {
        publish();
        return;
    }

    // Transform IMU data into ROS coordinates;
    msg_imu_.header.frame_id        = "/world";
    msg_imu_.header.stamp           = msg->header.stamp;
    msg_imu_.orientation.x          = msg->roll * DegToRad;
    msg_imu_.orientation.y          = msg->pitch * DegToRad;
    msg_imu_.orientation.z          = msg->heading * DegToRad;
    msg_imu_.angular_velocity.x     = msg->angularRateX;
    msg_imu_.angular_velocity.y     = msg->angularRateY;
    msg_imu_.angular_velocity.z     = msg->angularRateZ;
    msg_imu_.linear_acceleration.x  = msg->velocityNorth;
    msg_imu_.linear_acceleration.y  = -msg->velocityEast;
    msg_imu_.linear_acceleration.z  = -msg->velocityDown;


    // Transform GPS data into ROS coordinates;
    msg_gps_.header.frame_id    = "/world";
    msg_gps_.header.stamp       = msg->header.stamp;
    msg_gps_.altitude           = msg->altitude;
    msg_gps_.latitude           = msg->latitude;
    msg_gps_.longitude          = msg->longitude;
    if (msg->gps_num >= 7 && msg->gps_num < 255)
        msg_gps_.status.status  = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    else
        msg_gps_.status.status  = sensor_msgs::NavSatStatus::STATUS_NO_FIX;

    // CAR pose
    msg_pos_.header.frame_id    = "vehicle";
    msg_pos_.header.stamp       = msg->header.stamp;

    if (init_gps_.flag == 0)
    {
        init_gps_.flag = 1;
        init_gps_.latitude      = msg->latitude;
        init_gps_.longitude     = msg->longitude;
        init_gps_.altitude      = msg->altitude;

        msg_pos_.pose.position.x    = 0;
        msg_pos_.pose.position.y    = 0;
        msg_pos_.pose.position.z    = 0;
    }
    else
    {
        msg_pos_.pose.position.x    =  (msg->latitude  - init_gps_.latitude ) * DegToRad * EARTH_R;
        msg_pos_.pose.position.y    = -(msg->longitude - init_gps_.longitude) * DegToRad * EARTH_R * cos(msg->latitude * DegToRad);
        msg_pos_.pose.position.z    =  (msg->altitude  - init_gps_.altitude );
    }

    msg_pos_.pose.orientation.x     =  msg->roll * DegToRad;
    msg_pos_.pose.orientation.y     = -msg->pitch * DegToRad;
    msg_pos_.pose.orientation.z     = -msg->heading * DegToRad;

    publish();
}

void Transform::publish()
{

    // TF for frame "world" to frame "vehicle".
    tf_t_.setOrigin(tf::Vector3(msg_pos_.pose.position.x, msg_pos_.pose.position.y, msg_pos_.pose.position.z));
    tf_q_.setRPY(msg_pos_.pose.orientation.x, msg_pos_.pose.orientation.y, msg_pos_.pose.orientation.z);
    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, msg_pos_.header.stamp, "/world", "/vehicle"));

    // TF for frame "world" to frame "cost map"
    tf_t_.setOrigin(tf::Vector3(msg_pos_.pose.position.x, msg_pos_.pose.position.y, msg_pos_.pose.position.z));
    tf_q_.setRPY(0.0, 0.0, msg_pos_.pose.orientation.z);
    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, msg_pos_.header.stamp, "/world", "/cost_map"));

    // TODO here.TF for frame "world" to frame "elevation map"
    tf_t_.setOrigin(tf::Vector3(msg_pos_.pose.position.x, msg_pos_.pose.position.y, msg_pos_.pose.position.z));
    tf_q_.setRPY(0.0, 0.0, 0.0);
    tf_t_.setRotation(tf_q_);
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_t_, msg_pos_.header.stamp, "/world", "/elevation_map"));

    pub_GPS_.publish(msg_gps_);
    pub_Imu_.publish(msg_imu_);
    pub_Pos_.publish(msg_pos_);
}
