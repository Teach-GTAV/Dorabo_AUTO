#include <iostream>
#include <vector>
#include <cmath>
using namespace std;

struct LS_point
{
    int x;
    int y;
};

class LeastSquare{
    double a, b;
    vector<LS_point> point_;
public:

    LeastSquare(const vector<LS_point>& point)
    {
        point_ = point;
    }

    void Compute()
    {
        double t1=0, t2=0, t3=0, t4=0;
        for(int i=0; i<point_.size(); ++i)
        {
            t1 += point_[i].x*point_[i].x;
            t2 += point_[i].x;
            t3 += point_[i].x*point_[i].y;
            t4 += point_[i].y;
        }
        a = (t3*point_.size() - t2*t4) / (t1*point_.size() - t2*t2);
        b = (t1*t4 - t2*t3) / (t1*point_.size() - t2*t2);
    }

    float LS_error()
    {
        float error = 0.0;
        for(int i = 0; i < point_.size(); ++i)
        {
            error += pow((a*point_[i].x + b)-point_[i].y, 2);
        }
        return sqrt(error/point_.size());
    }

    int getY(int x)
    {
        return a*x + b;
    }

};
