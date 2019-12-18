#ifndef FUSIONZUPTLIDAR_H
#define FUSIONZUPTLIDAR_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <location_msgs/FusionDataInfo.h>
#include "DataType.h"

using namespace Eigen;
using namespace std;

class FusionCenter;

class FusionZUPTLidar
{
    public:
    FusionZUPTLidar(FusionCenter *pFsCenter);
    ~FusionZUPTLidar(void);

    private:
    FusionCenter *g_pfscenter;

    private:
    void initForNavigationParam(double (&LLH)[3], double (&VENU)[3], Vector3d &wn_in, double (&R_NM)[2]);
    void initForPQ(void);    //  滤波参数初始化
    void initForR(void);        //滤波R方差初始化
    void initForX(void);        //状态参数X初始化
    void calculateFtSysParam(PoseResult &h_fm, PoseResult &l_fm,
                            MatrixXd &fusion_F,MatrixXd &fusion_G, Matrix3d &C_bn);     //计算系统方程的F,G矩阵
    void calculateFtMeaParam(PoseResult &h_fm,MatrixXd &fusion_H, Matrix3d &C_bn);     //计算测量方程的H矩阵
    void discreteForFusionFG(MatrixXd &fusion_F, MatrixXd &fusion_G, MatrixXd &fusion_Q0, double &delta_T,
                            MatrixXd &g_fusion_Fai, MatrixXd &g_fusion_Q); //一步转移矩阵和等效离散系统噪声方差阵的计算
    void calculateMeaZ(PoseResult &high_freq_match, PoseResult &low_freq_match, Matrix3d &C_bn, VectorXd &fusion_Z);        //计算测量方程测量参数Z
    void calculatePoseConfidence(VectorXd &fusion_Z, MatrixXd &fusion_H, MatrixXd &I_k, float &pose_confidence);       //计算位置置信度

    public:
    bool calculateFilter(PoseResult &high_freq_match, PoseResult &low_freq_match,UTC &time,
                        location_msgs::FusionDataInfo &g_fusion_data);
    
    private:
    MatrixXd g_fusion_P0;     //初始估计均方误差阵 顺序:组合导航姿态误差,速度误差,位置误差,低频传感器的速度误差,位置误差
    MatrixXd g_fusion_R;      //测量方程方差阵

    public:
    MatrixXd g_fusion_Q0;     //初始系统误差方差阵
    VectorXd g_fusion_X;         //当前状态参数
    VectorXd g_fusion_lX;        //上一状态参数
    MatrixXd g_fusion_lI;        //上一状态的信息矩阵
    UTC g_last_tm;               //上一时间配准时刻
};


#endif