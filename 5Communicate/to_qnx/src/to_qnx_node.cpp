#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <dynamic_reconfigure/server.h>
#include <to_qnx/qnx_commandConfig.h>

struct Command
{
    double steering;
    double velocity;
    int mode;
    int reset;

    double steer_p;
    double steer_i;
    double steer_d;

    double accel_p;
    double accel_i;
    double accel_d;

    double current_velocity;
}send_to_qnx;

int steering;

char m_data[1000];

void AppendChecksum()
{
    int i=0;
    unsigned char checkSum = 0;
    for(i=1; i<strlen(m_data); i++)
    {
        checkSum ^= m_data[i];
    }
    sprintf(m_data, "%s*%02X\r\n", m_data, checkSum);
}

void CommandCallback(const geometry_msgs::TwistStampedPtr msg)
{

    steering = (int)(msg->twist.angular.z/0.25*2000);
}

void data_sum(void)
{
    ros::param::get("/to_qnx_command/vel",send_to_qnx.velocity);
    ros::param::get("/to_qnx_command/current_vel",send_to_qnx.current_velocity);
    sprintf(m_data, "$1,%d,%d,%0.3f,%d,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,",
            (int)send_to_qnx.reset ,(int)steering, (double)send_to_qnx.velocity, (int)send_to_qnx.mode,
            send_to_qnx.steer_p, send_to_qnx.steer_i, send_to_qnx.steer_d,
            send_to_qnx.accel_p, send_to_qnx.accel_i, send_to_qnx.accel_d,send_to_qnx.current_velocity);

    AppendChecksum();
}

void callback(to_qnx::qnx_commandConfig &config, uint32_t level)
{
    send_to_qnx.accel_p = config.accel_p;
    send_to_qnx.accel_i = config.accel_i;
    send_to_qnx.accel_d = config.accel_d;

    send_to_qnx.steer_p = config.steer_p;
    send_to_qnx.steer_i = config.steer_i;
    send_to_qnx.steer_d = config.steer_d;

    send_to_qnx.velocity = config.vel;
    send_to_qnx.mode     = config.mode;
    send_to_qnx.reset    = config.reset;

    send_to_qnx.current_velocity = config.current_vel;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "Command_to_QNX");
    ros::NodeHandle node;

    //------------------------Initial for the socket-------------------//
    int sock_fd;
    if ((sock_fd = socket(PF_INET, SOCK_DGRAM, 0)) < 0)
        ROS_INFO("Socket error");


    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port   = htons(8000);
    if (inet_pton(AF_INET, "192.168.1.12", &servaddr.sin_addr) <= 0)
    {
        ROS_INFO("IP error");
        return 1;
    }

    if (connect(sock_fd, (struct sockaddr *)& servaddr, sizeof(servaddr)) == -1)
    {
        ROS_INFO("connect error");
        return 1;
    }

    ros::Subscriber sub_command = node.subscribe("/cb_local_planner/Curvature2QNX", 1, &CommandCallback);

    dynamic_reconfigure::Server<to_qnx::qnx_commandConfig> server;
    dynamic_reconfigure::Server<to_qnx::qnx_commandConfig>::CallbackType f;

    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate loop(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop.sleep();
        data_sum();
        ROS_INFO("%s\n", m_data);
        write(sock_fd, m_data, strlen(m_data));
    }
    ros::spin();
    return 0;
}
