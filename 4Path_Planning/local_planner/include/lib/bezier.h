#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>


#define SAMPLE_SIZE  30
#define SAMPLE_DIFF  0.3
#define SAMPLE_SCALE 1
#define LINE_POINTS  10

#define CURVATURE_THRESHOLD 0.17

using namespace std;
using namespace geometry_msgs;
using namespace nav_msgs;

struct Traj
{
    nav_msgs::Path path;
    Point points[4];

    bool obstacle_fail;
    bool curve_fail;
    double curve_cost;
    double obstacle_cost;
    double variance_cost;
    double distance;


    double cost_sum;

    void init(void)
    {
        obstacle_fail = 0;
        obstacle_cost = 0.0;
        curve_cost = 0.0;
        variance_cost = 0.0;
        curve_fail = 0;
        distance = 0.0;
        path.poses.clear();
    }

    void traj_init(Point target)
    {
        points[0].x = 0;
        points[0].y = 0;

        points[1].x = 4;
        points[1].y = 0;

        points[2].x = target.x-5;
        points[2].y = target.y;

        points[3]   = target;
    }
};

vector<Traj> BezierGeneration(Point target);
void Interpolation(Traj &traj);
double curvaDetect(Traj traj, int i);
