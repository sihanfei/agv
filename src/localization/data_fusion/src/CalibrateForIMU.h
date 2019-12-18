#ifndef CALIBRATEFORIMU_H
#define CALIBRATEFORIMU_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <location_msgs/FusionDataInfo.h>
#include "DataType.h"

using namespace Eigen;
using namespace std;

class FusionCenter;
class CalibrateForIMU
{
    public:
    CalibrateForIMU(FusionCenter *pFsCenter);
    ~CalibrateForIMU(void);

    private:
    FusionCenter *g_pfscenter;

    private:
    void initForQ(void);    //  滤波参数初始化
    void initForNavigationParam(double (&LLH)[3], double (&VENU)[3], Vector3d &wn_in, double (&R_NM)[2]);
    void calculateFtSysParam(PoseResult &h_fm, MatrixXd &fusion_F, MatrixXd &fusion_G);     //计算系统方程的F矩阵
    void discreteForFusionFG(MatrixXd &fusion_F,double &delta_T, MatrixXd &fusion_Fai,
                            MatrixXd &fusion_G,MatrixXd &fusion_Q0,MatrixXd &fusion_Q);
    void calculatePoseConfidence(MatrixXd &P_2k, float &pose_confidence);       //计算位置置信度

    public:
    bool calculateFilter(PoseResult &high_freq_match, UTC &time,location_msgs::FusionDataInfo &g_fusion_data_calib);

    public:
    MatrixXd g_fusion_Q0;     //初始系统误差方差阵
    UTC g_last_tm;               //上一时间配准时刻
    MatrixXd g_calib_I;          //存储上一个传感器融合的信息阵；

};


#endif // !CALIBRATEFORIMU_H
