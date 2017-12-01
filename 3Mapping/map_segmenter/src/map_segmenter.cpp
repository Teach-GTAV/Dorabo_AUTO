#include "map_segmenter.hpp"
#include "Morphology.h"
#include "superpixel.hpp"
#include "math_function.h"

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <geometry_msgs/PointStamped.h>

using namespace map_segmenter;
using namespace ai;

MapSeg::MapSeg(ros::NodeHandle nodehandle)
    : rosNode_(nodehandle),
      imgNode_(nodehandle)
{

    readParameters();

    pubPoint_         = rosNode_.advertise<geometry_msgs::PointStamped>(strPubPoint_,   1);

    pubImgRoad_       = imgNode_.advertise("map_raw", 1);
    pubLine1_         = imgNode_.advertise("line1", 1);
    pubLine2_         = imgNode_.advertise("line2", 1);
    pubLine3_         = imgNode_.advertise("line3", 1);

    subImg_           = imgNode_.subscribe(strSubImg_, 1, &MapSeg::mapCallback, this);
}

void MapSeg::mapCallback(const sensor_msgs::ImageConstPtr &img)
{

    //Raw data
    Mat img0 = cv_bridge::toCvShare(img, "bgr8")->image;

    switch (param_.filter_mode)
    {
    case 1:
        blur(img0, img0, Size(2*param_.filter+1, 2*param_.filter+1));
        break;
    case 2:
        GaussianBlur(img0, img0, Size(2*param_.filter+1, 2*param_.filter+1), 0, 0);
        break;
    case 3:
        medianBlur(img0, img0, 2*param_.filter+1);
        break;
    default:
        break;
    }

    //Resize
    Size size;
    size.width = img0.cols*param_.resize;
    size.height = img0.rows*param_.resize;
    resize(img0, img0, size, CV_INTER_NN);

    //Closing
    Mat element = getStructuringElement(MORPH_RECT, Size(2*param_.closing+1,2*param_.closing+1), Point(param_.closing, param_.closing));
    morphologyEx(img0, img0, MORPH_CLOSE, element);

    //Openning
    Mat element1 = getStructuringElement(MORPH_RECT, Size(2*param_.openning+1, 2*param_.openning+1), Point(param_.openning, param_.openning));
    morphologyEx(img0, img0, MORPH_OPEN, element1);

    //Resize back
    size.width = img0.cols/param_.resize;
    size.height = img0.rows/param_.resize;
    resize(img0, img0, size, CV_INTER_NN);

    //Skeletonization
    Mat img_grey;
    cvtColor(img0,img_grey,CV_BGR2GRAY);
    Mat bit_not = Mat::zeros(img0.size(), CV_8UC1);
    cv::bitwise_not(img_grey, bit_not);
    Mat img_skelet;

    for (size_t i = 0; i < bit_not.rows; i++)
    {
        for(size_t j = 0; j < param_.width; j++)
        {
            bit_not.at<unsigned char>(i, j) = param_.grey;
            bit_not.at<unsigned char>(i, bit_not.cols-j) = param_.grey;
        }
    }
    bit_not.copyTo(img_skelet);
    skeletonizaton(img_skelet);

    //Remove tiny contours, using flann kdtree to generate the trees and points;
    vector<vector<Point> > contours;
    vector<Vec4i>          hierarchy;
    cv::findContours(img_skelet, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
    RNG rng(12345);
    Mat drawing = cv::Mat::zeros(img_skelet.size(), CV_8UC3);
    vector<vector<Point> > contours_poly (contours.size());
    vector<Point2f> all_points;
    for (size_t i =0; i < contours.size(); i++)
    {
        if (cv::arcLength(contours[i], false) > MINARCLEN)
        {

            Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );

            vector<Point> poly;
            cv::approxPolyDP( contours[i], poly, 3, false);
            contours_poly.push_back(poly);
            vector<Point2f> f_poly;
            for (size_t j = 0; j < poly.size(); j++)
            {
                Point2f point;
                point.x = poly[j].x;
                point.y = poly[j].y;
                f_poly.push_back(point);\
                all_points.push_back(point);
            }
        }
    }

    Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
    Point2f target_point;
    double min_dis = 10000;
    for (size_t i = 0 ; i < all_points.size(); i++)
    {
        double distance = sqrt(pow(all_points[i].y - 0 , 2) +
                               pow(all_points[i].x - img_skelet.cols/2, 2));

        if (min_dis > distance)
        {
            min_dis = distance;
            target_point = all_points[i];
        }
    }
    circle(img0, Point2f(-(target_point.x-img_skelet.cols/2)/(target_point.y-img_skelet.rows/2)*img_skelet.rows/2+img_skelet.cols/2, 0), 2, color);
    circle(img0, Point2f(img_skelet.rows/2, img_skelet.cols/2), 2, color);

    geometry_msgs::PointStamped mission;
    mission.header.frame_id = "vehicle";
    mission.header.stamp = img->header.stamp;
    mission.point.x = img_skelet.rows/2*0.4;
    mission.point.y = (target_point.x-img_skelet.cols/2)/(target_point.y-img_skelet.rows/2)*img_skelet.rows/2*0.4;
    mission.point.z = 0;

    pubPoint_.publish(mission);


    //Publish image message.
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img0).toImageMsg();
    pubImgRoad_.publish(msg);

    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",   img_skelet).toImageMsg();
    pubLine1_.publish(msg);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8",    drawing).toImageMsg();
    pubLine2_.publish(msg);
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8",   bit_not).toImageMsg();
    pubLine3_.publish(msg);
}
