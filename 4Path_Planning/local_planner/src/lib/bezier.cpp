#include "bezier.h"

using namespace std;
using namespace geometry_msgs;

vector<Traj> BezierGeneration(Point target)
{
    Traj traj;
    traj.init();
    vector<Traj> trajs;
    for(int i= -SAMPLE_SIZE/2; i < SAMPLE_SIZE/2; i++)
    {
        Point target_point = target;
        target_point.y = target.y + i*SAMPLE_DIFF;
        traj.traj_init(target_point);

        Interpolation(traj);
        traj.distance = fabs(i*1.0);
        trajs.push_back(traj);
    }
    return trajs;
}

void Interpolation(Traj &traj)
{

    traj.path.poses.clear();
    Point point;
    traj.distance   = 0.0;
    traj.curve_cost = 0.0;
    traj.curve_fail = 0;
    for(size_t i = 0; i < LINE_POINTS; i++)
    {
        double t = double(i)/LINE_POINTS;
        point.x = pow((1-t),3)*pow(t,0)*traj.points[0].x +
                pow((1-t),2)*pow(t,1)*traj.points[1].x*3 +
                pow((1-t),1)*pow(t,2)*traj.points[2].x*3 +
                pow((1-t),0)*pow(t,3)*traj.points[3].x;

        point.y = pow((1-t),3)*pow(t,0)*traj.points[0].y +
                pow((1-t),2)*pow(t,1)*traj.points[1].y*3 +
                pow((1-t),1)*pow(t,2)*traj.points[2].y*3 +
                pow((1-t),0)*pow(t,3)*traj.points[3].y;

        PoseStamped pose;
        pose.pose.position = point;
        traj.path.poses.push_back(pose);

        //! curvature cost ----------------------------------------------------------//
        double curve_cost = curvaDetect(traj, i);
        traj.curve_cost += fabs(curve_cost);

        if (fabs(curve_cost) >= CURVATURE_THRESHOLD)
        {
           traj.curve_fail = 1;
           break;
        }
    }

    traj.curve_cost /= LINE_POINTS;
//    std::cerr << "curve: "   <<traj.curve_cost << std::endl;
    return;
}

double curvaDetect(Traj traj, int i)
{
    double t = double(i)/LINE_POINTS;
    Point D_point;
    D_point.x =
            pow((1-t),2)*pow(t,0)*(traj.points[1].x-traj.points[0].x)*3 +
            pow((1-t),1)*pow(t,1)*(traj.points[2].x-traj.points[1].x)*6 +
            pow((1-t),0)*pow(t,2)*(traj.points[3].x-traj.points[2].x)*3;

    D_point.y =
            pow((1-t),2)*pow(t,0)*(traj.points[1].y-traj.points[0].y)*3 +
            pow((1-t),1)*pow(t,1)*(traj.points[2].y-traj.points[1].y)*6 +
            pow((1-t),0)*pow(t,2)*(traj.points[3].y-traj.points[2].y)*3;


    Point DD_point;
    DD_point.x =
            pow((1-t),1)*pow(t,0)*(traj.points[2].x-2*traj.points[1].x+traj.points[0].x)*6 +
            pow((1-t),0)*pow(t,1)*(traj.points[3].x-2*traj.points[2].x+traj.points[1].x)*6;


    DD_point.y =
            pow((1-t),1)*pow(t,0)*(traj.points[2].y-2*traj.points[1].y+traj.points[0].y)*6 +
            pow((1-t),0)*pow(t,1)*(traj.points[3].y-2*traj.points[2].y+traj.points[1].y)*6;


    double curvature =
            (D_point.x*DD_point.y - D_point.y*DD_point.x)/
            pow(pow(D_point.x,2)+pow(D_point.y,2), 1.5);

    return curvature;
}


