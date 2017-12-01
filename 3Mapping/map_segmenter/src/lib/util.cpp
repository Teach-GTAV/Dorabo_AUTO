#include "map_segmenter.hpp"

using namespace map_segmenter;
using namespace std;


bool MapSeg::readParameters()
{
    rosNode_.param("sub_image",         strSubImg_,         string("/center_mapping/costmap"));
    rosNode_.param("pub_image",         strPubImg_,         string("map_seg"));
    rosNode_.param("pub_point",         strPubPoint_,       string("TargetPoint"));
}

void MapSeg::cfgCallback(Parameter &param)
{
    param_ = param;
}
