#ifndef MATCHFORTIME_
#define MATCHFORTIME_
 
#include <ros/ros.h>
#include <string>
#include <vector>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <eigen3/Eigen/Dense>
#include "DataType.h"


class FusionCenter;

using namespace Eigen;
using namespace std;

class MatchForTime{

    public:
    MatchForTime(FusionCenter *pFsCenter);
    ~ MatchForTime(void);

    void MatchForLinear(vector<double*>& pose_match_utm, vector<double*>& vel_match_utm,  PoseResult &high_freq_match,double &lon0);
    void MatchForCircular(vector<double*>& pose_match_utm, vector<double*> vel_match_utm, PoseResult &high_freq_match,double &lon0);
    void MatchForStop(PoseResult &high_freq_match);
 
    private:
    FusionCenter *g_pfscenter; 

    private:
    void CalculateCircular(vector<double*>&pose_match_utm,Matrix3d &A,Vector3d &B,  Vector3d &X, double (&circle)[4]); //计算匀速圆周运动的圆心,半径,角速度  
    void setTime(UTC& hf_tm, UTC& lf_tm);

};

#endif
