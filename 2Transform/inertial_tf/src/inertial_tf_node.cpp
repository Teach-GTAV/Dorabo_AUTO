#include <ros/ros.h>
#include "inertial_tf.h"

int main(int argc, char** argv)
{
    ros::init (argc, argv, "inertial_tf");
    ros::NodeHandle node("~");

    inertial_tf::Transform transform(node);
    ros::spin();

    return 0;
}
