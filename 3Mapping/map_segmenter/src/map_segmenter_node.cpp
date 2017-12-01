#include <ros/ros.h>
#include "map_segmenter.hpp"

using namespace map_segmenter;

map_segmenter::Parameter param;

void Callback(map_segmenter::map_segConfig &config, uint32_t level)
{
    param.resize    = config.resize;
    param.closing   = config.closing;
    param.openning  = config.openning;
    param.thinning  = config.thinning;
    param.filter    = config.filter;
    param.filter_mode = config.filter_mode;

    param.grey      = config.grey;
    param.width     = config.width;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "map_segmenter");
    ros::NodeHandle nh("~");

    dynamic_reconfigure::Server<map_segmenter::map_segConfig> server;
    dynamic_reconfigure::Server<map_segmenter::map_segConfig>::CallbackType f;

    f = boost::bind(&Callback, _1, _2);
    server.setCallback(f);

    MapSeg seg(nh);

    while(ros::ok())
    {
        ros::spinOnce();
        seg.cfgCallback(param);
    }
    return 0;
}

