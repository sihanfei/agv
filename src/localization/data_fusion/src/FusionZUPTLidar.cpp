#include "FusionZUPTLidar.h"
#include "FusionCenter.h"
#include "CoordinateSystem.h"
#include <vector>
#include <eigen3/Eigen/Sparse>
#include "GlobalVari.h"


FusionZUPTLidar::FusionZUPTLidar(FusionCenter *pFsCenter)
{
    g_pfscenter = pFsCenter;
    memset(&g_last_tm,0,sizeof(UTC));
    initForPQ();
    g_fusion_X.resize(15);
    g_fusion_X = VectorXd::Zero(15,1);
    g_fusion_lX.resize(15);
    g_fusion_lX = VectorXd::Zero(15,1);
    g_fusion_lI.resize(15,15);
    g_fusion_lI = MatrixXd ::Zero(15,15);
}

FusionZUPTLidar::~FusionZUPTLidar(void)
{

}

// **************
// 功能:初始化状态参量P,Q
// 输入:无
// 输出:无
// 无返回
// ***************
void FusionZUPTLidar::initForPQ(void)
{
    int count = 15;
    double P[count];
    //double P[15] = {0.01*M_PIl/180, 0.01*M_PIl/180, 0.01*M_PIl/180, 0.01, 0.01, 0.01, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1};
    P[0] = g_cfg_zuptlidar.P0.Fai_yaw*M_PIl/180;
    P[1] = g_cfg_zuptlidar.P0.Fai_pitch*M_PIl/180;
    P[2] = g_cfg_zuptlidar.P0.Fai_roll*M_PIl/180;
    P[3] = g_cfg_zuptlidar.P0.delta_ve;
    P[4] = g_cfg_zuptlidar.P0.delta_vn;
    P[5] = g_cfg_zuptlidar.P0.delta_vu;
    P[6] = g_cfg_zuptlidar.P0.delta_lan;
    P[7] = g_cfg_zuptlidar.P0.delta_lon;
    P[8] = g_cfg_zuptlidar.P0.delta_h;
    P[9] = g_cfg_zuptlidar.P0.delta_Lve;
    P[10] = g_cfg_zuptlidar.P0.delta_Lvn;
    P[11] = g_cfg_zuptlidar.P0.delta_Lvu;
    P[12] = g_cfg_zuptlidar.P0.delta_Llan;
    P[13] = g_cfg_zuptlidar.P0.delta_Llon;
    P[14] = g_cfg_zuptlidar.P0.delta_Lh;

    double Q[count];
    //double Q[15] = {0.1*M_PIl/180, 0.1*M_PIl/180, 0.1*M_PIl/180, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.5, 0.25, 0.25, 0.35, 0.35, 0.45};
    Q[0] = g_cfg_zuptlidar.q0.Fai_yaw*M_PIl/180;
    Q[1] = g_cfg_zuptlidar.q0.Fai_pitch*M_PIl/180;
    Q[2] = g_cfg_zuptlidar.q0.Fai_roll*M_PIl/180;
    Q[3] = g_cfg_zuptlidar.q0.delta_ve;
    Q[4] = g_cfg_zuptlidar.q0.delta_vn;
    Q[5] = g_cfg_zuptlidar.q0.delta_vu;
    Q[6] = g_cfg_zuptlidar.q0.delta_lan;
    Q[7] = g_cfg_zuptlidar.q0.delta_lon;
    Q[8] = g_cfg_zuptlidar.q0.delta_h;
    Q[10] = g_cfg_zuptlidar.q0.delta_Lve;
    Q[11] = g_cfg_zuptlidar.q0.delta_Lvn;
    Q[12] = g_cfg_zuptlidar.q0.delta_Lvu;
    Q[12] = g_cfg_zuptlidar.q0.delta_Llan;
    Q[13] = g_cfg_zuptlidar.q0.delta_Llon;
    Q[14] = g_cfg_zuptlidar.q0.delta_Lh;

    g_fusion_Q0.resize(count,count);
    g_fusion_Q0 = MatrixXd::Zero(count,count);
    g_fusion_P0.resize(count,count);
    g_fusion_P0 = MatrixXd::Zero(count,count);
    for(int i = 0; i < count; i++)
    {
        for(int j = 0; j < count; j++)
        {
            if (i==j)
            {
                g_fusion_P0(i,j) = P[i]*P[j];
                g_fusion_Q0(i,j) = Q[i]*Q[j];
            }
            else
            {
                g_fusion_P0(i,j) = 0.0;
                g_fusion_Q0(i,j) = 0.0;
            }
        }
    }
}

// **************
// 功能:初始化状态参量X
// 输入:无
// 输出:无
// 无返回
// ***************
void FusionZUPTLidar::initForX(void)
{
    // g_fusion_lX(0,0) = 0.005*M_PIl/180;       //delta_yaw
    // g_fusion_lX(1,0) = 0.003*M_PIl/180;     //delta_pitch
    // g_fusion_lX(2,0) = 0.003*M_PIl/180;     //delta_roll
    // g_fusion_lX(3,0) = 0.01;      //delta_ve
    // g_fusion_lX(4,0) = 0.01;      //delta_vn
    // g_fusion_lX(5,0) = 0.01;      //delta_vu
    // g_fusion_lX(6,0) = 0.5;      //delta_nav_lan
    // g_fusion_lX(7,0) = 0.35;      //delta_nav_lon
    // g_fusion_lX(8,0) = 0.02;      //delta_nav_h
    // g_fusion_lX(9,0) = 0.1;      //delta_lidar_ve
    // g_fusion_lX(10,0) = 0.1;     //delta_lidar_vn
    // g_fusion_lX(11,0) = 0.1;     //delta_lidar_vu
    // g_fusion_lX(12,0) = 0.5;      //delta_lidar_lan
    // g_fusion_lX(13,0) = 0.6;      //delta_lidar_lon
    // g_fusion_lX(14,0) = 0.03;      //delta_lidar_h

    g_fusion_lX(0,0) = g_cfg_zuptlidar.X0.Fai_yaw*M_PIl/180;       //delta_yaw
    g_fusion_lX(1,0) = g_cfg_zuptlidar.X0.Fai_pitch*M_PIl/180;     //delta_pitch
    g_fusion_lX(2,0) = g_cfg_zuptlidar.X0.Fai_roll*M_PIl/180;     //delta_roll
    g_fusion_lX(3,0) = g_cfg_zuptlidar.X0.delta_ve;      //delta_ve
    g_fusion_lX(4,0) = g_cfg_zuptlidar.X0.delta_vn;      //delta_vn
    g_fusion_lX(5,0) = g_cfg_zuptlidar.X0.delta_vu;      //delta_vu
    g_fusion_lX(6,0) = g_cfg_zuptlidar.X0.delta_lan;      //delta_nav_lan
    g_fusion_lX(7,0) = g_cfg_zuptlidar.X0.delta_lon;      //delta_nav_lon
    g_fusion_lX(8,0) = g_cfg_zuptlidar.X0.delta_h;      //delta_nav_h
    g_fusion_lX(9,0) = g_cfg_zuptlidar.X0.delta_Lve;      //delta_lidar_ve
    g_fusion_lX(10,0) = g_cfg_zuptlidar.X0.delta_Lvn;     //delta_lidar_vn
    g_fusion_lX(11,0) = g_cfg_zuptlidar.X0.delta_Lvu;     //delta_lidar_vu
    g_fusion_lX(12,0) = g_cfg_zuptlidar.X0.delta_Llan;      //delta_lidar_lan
    g_fusion_lX(13,0) = g_cfg_zuptlidar.X0.delta_Llon;      //delta_lidar_lon
    g_fusion_lX(14,0) = g_cfg_zuptlidar.X0.delta_Lh;      //delta_lidar_h
}

// **************
// 功能:初始化状态参量的测量方差R
// 输入:无
// 输出:无
// 无返回
// ***************
void FusionZUPTLidar::initForR(void)
{
    //测量方程方差阵R,需改,根据low_freq_type再加分支
    int count = 0;
    vector<double> v_R;
    if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop) 
    {
        count = 12;
        double R[count];
        //double R[12] = {0.9*M_PIl/180, 0.3*M_PIl/180, 0.3*M_PIl/180, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.2};
        R[0] = g_cfg_zuptlidar.ZSTOP_R.delta_yaw*M_PIl/180;
        R[1] = g_cfg_zuptlidar.ZSTOP_R.delta_pitch*M_PIl/180;
        R[2] = g_cfg_zuptlidar.ZSTOP_R.delta_roll*M_PIl/180;
        R[3] = g_cfg_zuptlidar.ZSTOP_R.delta_ve;
        R[4] = g_cfg_zuptlidar.ZSTOP_R.delta_vn;
        R[5] = g_cfg_zuptlidar.ZSTOP_R.delta_vu;
        R[6] = g_cfg_zuptlidar.ZSTOP_R.delta_Lve;
        R[7] = g_cfg_zuptlidar.ZSTOP_R.delta_Lvn;
        R[8] = g_cfg_zuptlidar.ZSTOP_R.delta_Lvu;
        R[9] = g_cfg_zuptlidar.ZSTOP_R.delta_Clan;
        R[10] = g_cfg_zuptlidar.ZSTOP_R.delta_Clon;
        R[11] = g_cfg_zuptlidar.ZSTOP_R.delta_Ch;
        for(int i = 0; i < count; i++)
        {
            v_R.push_back(R[i]);
        }
    }
    else if (g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
    {
        count = 11;
        double R[count];
        //double R[11] = {0.9*M_PIl/180, 0.3*M_PIl/180, 0.3*M_PIl/180, 0.01, 0.01, 0.01, 0.5, 0.6, 0.2, 0.01, 0.01};
        R[0] = g_cfg_zuptlidar.ZLIN_R.delta_yaw*M_PIl/180;
        R[1] = g_cfg_zuptlidar.ZLIN_R.delta_pitch*M_PIl/180;
        R[2] = g_cfg_zuptlidar.ZLIN_R.delta_roll*M_PIl/180;
        R[3] = g_cfg_zuptlidar.ZLIN_R.delta_Cve;
        R[4] = g_cfg_zuptlidar.ZLIN_R.delta_Cvn;
        R[5] = g_cfg_zuptlidar.ZLIN_R.delta_Cvu;
        R[6] = g_cfg_zuptlidar.ZLIN_R.delta_Clan;
        R[7] = g_cfg_zuptlidar.ZLIN_R.delta_Clon;
        R[8] = g_cfg_zuptlidar.ZLIN_R.delta_Ch;
        R[9] = g_cfg_zuptlidar.ZLIN_R.Bvx;
        R[10] = g_cfg_zuptlidar.ZLIN_R.Bvz;
        for(int i = 0; i < count; i++)
        {
            v_R.push_back(R[i]);
        }
    }
    g_fusion_R.resize(count,count);
    g_fusion_R = MatrixXd::Zero(count,count);
    for(int i = 0; i < count; i++)
    {
        for(int j = 0; j < count; j++)
        {
            if (i==j) 
            {
                g_fusion_R(i,j) = v_R[i]*v_R[j];
            }
            else
            {
                g_fusion_R(i,j) = 0.0;
            }
        }
    }
}

// **************
// 功能:计算系统方程(时间更新)与量测方程的矩阵参数
// 输入:h_fm 本帧高频传感器结果 l_fm 本帧低频传感器结果
// 输出:计算的fusion_F和fusion_H矩阵与fusion_G
// 无返回
// ***************
void FusionZUPTLidar::calculateFtSysParam(PoseResult &h_fm, PoseResult &l_fm,
                                MatrixXd &fusion_F, MatrixXd &fusion_G,Matrix3d &C_bn)
{
    double LLH[3],VENU[3],R_NM[2];
    Vector3d wn_in;
    wn_in = Vector3d::Zero(3,1);
    LLH[0] = h_fm.pos.lan;
    LLH[1] = h_fm.pos.lon;
    LLH[2] = h_fm.pos.h;
    VENU[0] = h_fm.vel.venu.vx;
    VENU[1] = h_fm.vel.venu.vy;
    VENU[2] = h_fm.vel.venu.vz;
    initForNavigationParam(LLH, VENU, wn_in, R_NM);
    int sys_count = 15;
    double Tao[6];
    //double Tao[6] = {5,5,5,10,10,10};
    Tao[0] = g_cfg_zuptlidar.Tao.delta_Lve;
    Tao[1] = g_cfg_zuptlidar.Tao.delta_Lvn;
    Tao[2] = g_cfg_zuptlidar.Tao.delta_Lvu;
    Tao[3] = g_cfg_zuptlidar.Tao.delta_Llan;
    Tao[4] = g_cfg_zuptlidar.Tao.delta_Llon;
    Tao[5] = g_cfg_zuptlidar.Tao.delta_Lh;
    int G[6] = {1,1,1,1,1,1};
    //fusion_F:
    fusion_F.resize(sys_count,sys_count);
    fusion_F.setZero(sys_count,sys_count);
    fusion_F(0,1) = w_ie*sin(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0]
                        +
                        h_fm.pos.h)*tan(h_fm.pos.lan*M_PI/180);
    fusion_F(0,2) = -(w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h));
    fusion_F(0,4) = -1/(R_NM[1] + h_fm.pos.h);
    fusion_F(0,8) = h_fm.vel.venu.vy/pow(R_NM[1] + h_fm.pos.h,2);

    fusion_F(1,0) = -fusion_F(0,1);
    fusion_F(1,2) = -h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(1,3) = 1/(R_NM[0] + h_fm.pos.h);
    fusion_F(1,6) = -w_ie*sin(h_fm.pos.lan*M_PI/180);
    fusion_F(1,8) = -h_fm.vel.venu.vx/pow((R_NM[0] + h_fm.pos.h),2);

    fusion_F(2,0) = -fusion_F(0,2);
    fusion_F(2,1) = -fusion_F(1,2);
    fusion_F(2,3) = tan(h_fm.pos.lan*M_PI/180)/(R_NM[0] + h_fm.pos.h);
    fusion_F(2,6) = w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)/pow(cos(h_fm.pos.lan*M_PI/180),2);
    fusion_F(2,8) = -h_fm.vel.venu.vx*tan(h_fm.pos.lan*M_PI/180)/pow(R_NM[0] + h_fm.pos.h,2);

    //h_fm的加速度为机体系
    fusion_F(3,1) = -C_bn(2,0)*h_fm.accel.ax - C_bn(2,1)*h_fm.accel.ay - C_bn(2,2)*h_fm.accel.az;   
    fusion_F(3,2) = C_bn(1,0)*h_fm.accel.ax + C_bn(1,1)*h_fm.accel.ay + C_bn(1,2)*h_fm.accel.az;
    fusion_F(3,3) = (h_fm.vel.venu.vy*tan(h_fm.pos.lan*M_PI/180)
                        -
                        h_fm.vel.venu.vz)/(R_NM[0] + h_fm.pos.h);
    fusion_F(3,4) = 2*w_ie*sin(h_fm.pos.lan*M_PI/180)
                        +
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)*tan(h_fm.pos.lan*M_PI/180);
    fusion_F(3,5) = -(2*w_ie*cos(h_fm.pos.lan*M_PI/180) + h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h));
    fusion_F(3,6) = 2*w_ie*(h_fm.vel.venu.vz*sin(h_fm.pos.lan*M_PI/180) 
                        +
                        h_fm.vel.venu.vy*cos(h_fm.pos.lan*M_PI/180))
                        + 
                        h_fm.vel.venu.vx*h_fm.vel.venu.vy/(R_NM[0] + h_fm.pos.h)
                        /
                        pow(cos(h_fm.pos.lan*M_PI/180),2); 
    fusion_F(3,8) = (h_fm.vel.venu.vx*h_fm.vel.venu.vz 
                        - 
                        h_fm.vel.venu.vx*h_fm.vel.venu.vy*tan(h_fm.pos.lan*M_PI/180))
                        /
                        pow(R_NM[0] + h_fm.pos.h,2);

    fusion_F(4,0) = -fusion_F(3,1);
    fusion_F(4,2) = -(C_bn(0,0)*h_fm.accel.ax + C_bn(0,1)*h_fm.accel.ay + C_bn(0,2)*h_fm.accel.az);
    fusion_F(4,3) = -2*(w_ie*sin(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)
                        * 
                        tan(h_fm.pos.lan*M_PI/180));
    fusion_F(4,4) = -h_fm.vel.venu.vz/(R_NM[1] + h_fm.pos.h);
    fusion_F(4,5) = -h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(4,6) = -(2*h_fm.vel.venu.vx*w_ie*cos(h_fm.pos.lan) 
                        + 
                        pow(h_fm.vel.venu.vx,2)/(R_NM[0] + h_fm.pos.h)
                        /
                        pow(cos(h_fm.pos.lan*M_PI/180),2));
    fusion_F(4,8) = h_fm.vel.venu.vy*h_fm.vel.venu.vz
                        /
                        pow(R_NM[1]+h_fm.pos.h,2) 
                        + 
                        pow(h_fm.vel.venu.vx,2)*tan(h_fm.pos.lan*M_PI/180)
                        /
                        pow(R_NM[0] + h_fm.pos.h,2);

    fusion_F(5,0) = -fusion_F(3,2);
    fusion_F(5,1) = -fusion_F(4,2);
    fusion_F(5,3) = 2*(w_ie*cos(h_fm.pos.lan*M_PI/180) 
                        + 
                        h_fm.vel.venu.vx
                        /
                        (R_NM[0] + h_fm.pos.h));
    fusion_F(5,4) = 2*h_fm.vel.venu.vy/(R_NM[1] + h_fm.pos.h);
    fusion_F(5,6) = -2*h_fm.vel.venu.vx*w_ie*sin(h_fm.pos.lan*M_PI/180);
    fusion_F(5,8) = -(pow(h_fm.vel.venu.vy,2)/pow(R_NM[1] + h_fm.pos.h,2) 
                        + 
                        pow(h_fm.vel.venu.vx,2)/pow(R_NM[0] + h_fm.pos.h,2));

    fusion_F(6,4) = 1/(R_NM[1] + h_fm.pos.h);
    fusion_F(6,8) = -h_fm.vel.venu.vy/pow(R_NM[1] + h_fm.pos.h,2);

    fusion_F(7,3) = 1/cos(h_fm.pos.lan*M_PI/180)/(R_NM[0] + h_fm.pos.h);
    fusion_F(7,6) = h_fm.vel.venu.vx/(R_NM[0] + h_fm.pos.h)*tan(h_fm.pos.lan*M_PI/180)/cos(h_fm.pos.lan*M_PI/180);
    fusion_F(7,8) = -h_fm.vel.venu.vx/cos(h_fm.pos.lan*M_PI/180)/pow(R_NM[0] + h_fm.pos.h,2);
    fusion_F(8,5) = 1;
    for(int i = 0; i < sys_count-9; i++)
    {
        fusion_F(i+9,i+9) = -1/Tao[i];
    }

    //fusion_G:
    fusion_G.resize(sys_count,sys_count);
    fusion_G.setZero(sys_count,sys_count);      //系统噪声矩阵
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            fusion_G(i,j) = -C_bn(i,j);
        } 
    }
    for (int i = 3; i < 6; i++)
    {
        for (int j = 3; j < 6; j++)
        {
            fusion_G(i,j) = C_bn(i-3,j-3);
        } 
    }
    for(int i = 0; i < sys_count-9; i++)
    {
        fusion_G(i+9,i+9) = 1;
    }
}

// **************
// 功能:计算测量方程H矩阵
// 输入:h_fm 组合导航数据
// 输出:fusion_H H矩阵,根据低频传感器的类型H矩阵不同
// 无返回
// ***************
void FusionZUPTLidar::calculateFtMeaParam(PoseResult &h_fm,MatrixXd &fusion_H,Matrix3d &C_bn)
{
    //根据传感器类型,行数大小要改
    if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop) {
        fusion_H.resize(12,15);
        fusion_H.setZero(12,15);
        fusion_H(0,0) = -sin(h_fm.att.pitch*M_PI/180)*sin(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(0,1) = -sin(h_fm.att.pitch*M_PI/180)*cos(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(0,2) = 1.0;

        fusion_H(1,0) = -cos(h_fm.att.yaw*M_PI/180);
        fusion_H(1,1) = sin(h_fm.att.yaw*M_PI/180);

        fusion_H(2,0) = -sin(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(2,1) = -cos(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);

        fusion_H(3,3) = 1.0;
        fusion_H(4,4) = 1.0;
        fusion_H(5,5) = 1.0;

        fusion_H(6,9) = 1.0;
        fusion_H(7,10) = 1.0;
        fusion_H(8,11) = 1.0;

        fusion_H(9,6) = 1.0;
        fusion_H(9,12) = -1.0;
        
        fusion_H(10,7) = 1.0;
        fusion_H(10,13) = -1.0;

        fusion_H(11,8) = 1.0;
        fusion_H(11,14) = -1.0;
    }
    else if (g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
    {
        fusion_H.resize(11,15);
        fusion_H.setZero(11,15); 
        fusion_H(0,0) = -sin(h_fm.att.pitch*M_PI/180)*sin(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(0,1) = -sin(h_fm.att.pitch*M_PI/180)*cos(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(0,2) = 1.0;

        fusion_H(1,0) = -cos(h_fm.att.yaw*M_PI/180);
        fusion_H(1,1) = sin(h_fm.att.yaw*M_PI/180);

        fusion_H(2,0) = -sin(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        fusion_H(2,1) = -cos(h_fm.att.yaw*M_PI/180)/cos(h_fm.att.pitch*M_PI/180);
        
        fusion_H(3,3) = 1.0;
        fusion_H(3,9) = -1.0;

        fusion_H(4,4) = 1.0;
        fusion_H(4,10) = -1.0;

        fusion_H(5,5) = 1.0;
        fusion_H(5,11) = -1.0;

        fusion_H(6,6) = 1.0;
        fusion_H(6,12) = -1.0;  

        fusion_H(7,7) = 1.0;
        fusion_H(7,13) = -1.0;

        fusion_H(8,8) = 1.0;
        fusion_H(8,14) = -1.0;

        fusion_H(9,0) = C_bn(2,0)*h_fm.vel.venu.vy - C_bn(1,0)*h_fm.vel.venu.vz;
        fusion_H(9,1) = C_bn(0,0)*h_fm.vel.venu.vz - C_bn(2,0)*h_fm.vel.venu.vx;
        fusion_H(9,2) = C_bn(1,0)*h_fm.vel.venu.vx - C_bn(0,0)*h_fm.vel.venu.vy;
        fusion_H(9,3) = C_bn(0,0);
        fusion_H(9,4) = C_bn(1,0);
        fusion_H(9,5) = C_bn(2,0);

        fusion_H(10,0) = C_bn(2,2)*h_fm.vel.venu.vy - C_bn(1,2)*h_fm.vel.venu.vz;
        fusion_H(10,1) = C_bn(0,2)*h_fm.vel.venu.vz - C_bn(2,2)*h_fm.vel.venu.vx;
        fusion_H(10,2) = C_bn(1,2)*h_fm.vel.venu.vx - C_bn(0,2)*h_fm.vel.venu.vy;
        fusion_H(10,3) = C_bn(0,2);
        fusion_H(10,4) = C_bn(1,2);
        fusion_H(10,5) = C_bn(2,2);

    }
}

// **************
// 功能:计算导航坐标系下(东北天)指令角速度wn_in及两个曲率半径
// 输入:LLH[3] 纬经高,VENU[3]东北天速度
// 输出:wn_in 导航坐标系下指令角速度  R_NM 两个曲率半径,R_NM[0] 沿卯酉圈的曲率半径 R_NM[1] 沿子午圈的曲率半径
// 无返回
// ***************
void FusionZUPTLidar::initForNavigationParam(double (&LLH)[3], double (&VENU)[3], Vector3d &wn_in, double (&R_NM)[2])
{
    
    double LLH1[3];
    for(int i = 0; i < 2; i++)
    {
        LLH1[i] = LLH[i]*M_PI/180;
    }
    R_NM[0] = Ra*(1 - 2*ff + 3*ff*pow(sin(LLH1[0]),2));
    R_NM[1] = Ra*(1 + ff*pow(sin(LLH1[0]),2));
    Vector3d wn_ie;     //导航坐标系下跟着地球转的角速度
    wn_ie(0,0) = 0.0;
    wn_ie(1,0) = w_ie*cos(LLH1[0]);
    wn_ie(2,0) = w_ie*sin(LLH1[0]);

    Vector3d wn_en;        //由于运载体运动而引起的相对地球的旋转角速度
    wn_en(0,0) = -VENU[1]/R_NM[1];
    wn_en(1,0) = VENU[0]/R_NM[0];
    wn_en(2,0) = VENU[0]/R_NM[0]*tan(LLH1[0]);

    wn_in = wn_ie + wn_en; 
}

// **************
// 功能:一步转移矩阵和等效离散系统噪声方差阵的计算
// 输入:g_fusion_F时间更新的传递矩阵 g_fusion_G系统噪声矩阵,时间配准时刻time
// 输出:离散后的一步转移矩阵g_fusion_Fai,g_fusion_Q离散化后的系统误差方差阵
// 无返回
// ***************
void FusionZUPTLidar::discreteForFusionFG(MatrixXd &fusion_F, MatrixXd &fusion_G,MatrixXd &fusion_Q0, double &delta_T, 
                                MatrixXd &g_fusion_Fai, MatrixXd &g_fusion_Q)
{
    MatrixXd Mat_I;
    MatrixXd Mat_tp1;
    MatrixXd Mat_tp2;
    int sys_count = 15;
    //一步转移矩阵
    Mat_I.resize(sys_count,sys_count); 
    Mat_I.setIdentity(sys_count,sys_count); 
    Mat_tp1.resize(sys_count,sys_count); 
    Mat_tp1 = delta_T*fusion_F;
    Mat_tp2.resize(sys_count,sys_count);
    Mat_tp2 = pow(delta_T,2)/2*fusion_F*fusion_F;
    g_fusion_Fai = Mat_I + Mat_tp1 + Mat_tp2;

    //系统噪声离散化
    MatrixXd M_tp1;
    M_tp1.resize(sys_count,sys_count);
    M_tp1.setZero(sys_count,sys_count);
    MatrixXd M_tp2;
    M_tp2.resize(sys_count,sys_count);
    M_tp2.setZero(sys_count,sys_count);
    M_tp1 = fusion_G*fusion_Q0*fusion_G.transpose();
    MatrixXd temp;
    temp.resize(sys_count,sys_count);
    temp.setZero(sys_count,sys_count);
    temp = fusion_F*M_tp1;
    M_tp2 = temp + temp.transpose();
    g_fusion_Q = M_tp1*delta_T + M_tp2*pow(delta_T,2)/2;

}

// **************
// 功能:计算测量方程的测量参数Z
// 输入:high_freq_match组合导航,low_freq_match低频传感器,C_bn姿态变换阵
// 输出:fusion_Z测量参数Z
// 无返回
// ***************
void FusionZUPTLidar::calculateMeaZ(PoseResult &high_freq_match, PoseResult &low_freq_match, Matrix3d &C_bn, 
                                VectorXd &fusion_Z)
{
    int count = 0;
    if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop) {
        count = 12;
        fusion_Z.resize(count,1);
        fusion_Z(0,0) = (high_freq_match.att.yaw - low_freq_match.att.yaw)*M_PIl/180;
        fusion_Z(1,0) = (high_freq_match.att.pitch - low_freq_match.att.pitch)*M_PIl/180;
        fusion_Z(2,0) = (high_freq_match.att.roll - low_freq_match.att.roll)*M_PIl/180;
        fusion_Z(3,0) = high_freq_match.vel.venu.vx;
        fusion_Z(4,0) = high_freq_match.vel.venu.vy;
        fusion_Z(5,0) = high_freq_match.vel.venu.vz;
        fusion_Z(6,0) = low_freq_match.vel.venu.vx;
        fusion_Z(7,0) = low_freq_match.vel.venu.vy;
        fusion_Z(8,0) = low_freq_match.vel.venu.vz;
        double LLH[3] = {high_freq_match.pos.lan,high_freq_match.pos.lon,high_freq_match.pos.h};
        double lan_lon_deg[2];
        lan_lon_deg[0] = high_freq_match.pos.lan - low_freq_match.pos.lan;
        lan_lon_deg[1] = high_freq_match.pos.lon - low_freq_match.pos.lon;
        double lan_lon_m[2] = {0,0};
        transForDegreetoMeter(lan_lon_deg,LLH,lan_lon_m);
        fusion_Z(9,0) = lan_lon_m[0];
        fusion_Z(10,0) = lan_lon_m[1];
        fusion_Z(11,0) = high_freq_match.pos.h - low_freq_match.pos.h;
    }
    else if (g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
    {
        count = 11;
        fusion_Z.resize(count,1);
        fusion_Z(0,0) = (high_freq_match.att.yaw - low_freq_match.att.yaw)*M_PIl/180;
        fusion_Z(1,0) = (high_freq_match.att.pitch - low_freq_match.att.pitch)*M_PIl/180;
        fusion_Z(2,0) = (high_freq_match.att.roll - low_freq_match.att.roll)*M_PIl/180;
        fusion_Z(3,0) = high_freq_match.vel.venu.vx - low_freq_match.vel.venu.vx;
        fusion_Z(4,0) = high_freq_match.vel.venu.vy - low_freq_match.vel.venu.vy;
        fusion_Z(5,0) = high_freq_match.vel.venu.vz - low_freq_match.vel.venu.vz;
        double LLH[3] = {high_freq_match.pos.lan,high_freq_match.pos.lon,high_freq_match.pos.h};
        double lan_lon_deg[2];
        lan_lon_deg[0] = high_freq_match.pos.lan - low_freq_match.pos.lan;
        lan_lon_deg[1] = high_freq_match.pos.lon - low_freq_match.pos.lon;
        double lan_lon_m[2] = {0,0};
        transForDegreetoMeter(lan_lon_deg,LLH,lan_lon_m);
        fusion_Z(6,0) = lan_lon_m[0];
        fusion_Z(7,0) = lan_lon_m[1];
        fusion_Z(8,0) = high_freq_match.pos.h - low_freq_match.pos.h;
        fusion_Z(9,0) = -C_bn(0,0)*high_freq_match.vel.venu.vx
                        - C_bn(1,0)*high_freq_match.vel.venu.vy
                        - C_bn(2,0)*high_freq_match.vel.venu.vz;
        fusion_Z(10,0) = -C_bn(0,2)*high_freq_match.vel.venu.vx
                        - C_bn(1,2)*high_freq_match.vel.venu.vy
                        - C_bn(2,2)*high_freq_match.vel.venu.vz;
    }
}

// **************
// 功能:计算位置置信度（只用经纬计算）
// 输入:fusion_Z　测量值 low_freq_match fusion_H,测量更新矩阵 X_k,当前时刻估计量 I_k,信息矩阵 
// 输出:pose_confidence 位置置信度
// 返回:无
// ***************
void FusionZUPTLidar::calculatePoseConfidence(VectorXd &fusion_Z, MatrixXd &fusion_H, MatrixXd &I_k, float &pose_confidence)
{
    int count_lan,count_lon = 0, count = 0;
    MatrixXd temp1;
    MatrixXd temp2;
    if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop)
    {
        count = 12;
        count_lan = 9;
        count_lon = 10;
        temp1.resize(count,15);
        temp1.setZero(count,15);
        temp2.resize(count,count);
        temp2.setZero(count,count);
    }
    else if (g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
    {
        count = 11;
        count_lan = 6;
        count_lon = 7;
        temp1.resize(count,15);
        temp1.setZero(count,15);
        temp2.resize(count,count);
        temp2.setZero(count,count);
    }

    temp1 = fusion_H*g_fusion_X;
    Vector2d temp_z;
    Vector2d temp_z2;
    temp_z(0,0) = fusion_Z(count_lan);
    temp_z(1,0) = fusion_Z(count_lon);
    temp_z2(0,0) = temp1(count_lan);
    temp_z2(1,0) = temp1(count_lon);
    Vector2d delta_z;
    delta_z = Vector2d::Zero();
    delta_z = temp_z - temp_z2;
    double meas = 1.0;
    meas = delta_z.transpose()*delta_z;

    MatrixXd tpP_k;
    tpP_k.resize(15,15);
    tpP_k.setZero(15,15);
    tpP_k = I_k.inverse();

    temp2 = fusion_H*tpP_k*fusion_H.transpose() + g_fusion_R;
    double theory = 1.0;
    theory = temp2(count_lan,count_lan) + temp2(count_lon,count_lon);
    if (meas <= theory)
        {
            pose_confidence = meas/theory;
        }
    else
    {
        pose_confidence = theory/meas;
    } 
}

// **************
// 功能:滤波计算
// 输入:high_freq_match 本帧高频传感器结果 low_freq_match 本帧低频传感器结果,time融合时刻
// 输出:融合结果g_fusion_data
// 返回:true
// ***************
bool FusionZUPTLidar::calculateFilter(PoseResult &high_freq_match, PoseResult &low_freq_match,UTC &time,
                            location_msgs::FusionDataInfo &g_fusion_data)
{
    if (g_pfscenter->g_lzkinit_flag == false ) 
    {
        initForX();
        initForPQ();
        g_fusion_lI = g_fusion_P0.inverse();  //信息矩阵
    }
    initForR();
    MatrixXd fusion_F;      //时间更新的传递矩阵
    MatrixXd   fusion_H;                //量测更新的矩阵
    MatrixXd  fusion_G;      //系统噪声矩阵
    int count_f=15;
    float atitude[3];
    atitude[0] = high_freq_match.att.yaw;
    atitude[1] = high_freq_match.att.pitch;
    atitude[2] = high_freq_match.att.roll;
    double quaternions[4];
    Matrix3d C_bn;
    transForAngletoQuaternions(atitude,quaternions);
    transForCoordBodytoNavigation(quaternions,C_bn);

    calculateFtSysParam(high_freq_match, low_freq_match,fusion_F,fusion_G,C_bn);
    calculateFtMeaParam(high_freq_match,fusion_H,C_bn);
    double delta_T;
    delta_T = time.hour*3600 + time.min*60 + time.sec + time.msec/1000
              - (g_last_tm.hour*3600 + g_last_tm.min*60 + g_last_tm.sec + g_last_tm.msec/1000);
    if (delta_T > 1) 
    {
        delta_T = 0.1;
    }
    
    MatrixXd fusion_Fai;      //离散化后的时间更新的传递矩阵
    MatrixXd fusion_Q;     //离散化后的系统误差方差阵
    discreteForFusionFG(fusion_F, fusion_G, g_fusion_Q0, delta_T,fusion_Fai,fusion_Q);

    MatrixXd X_2k;       //一步预测状态参量(从k-1至k)
    X_2k.resize(count_f,1);
    X_2k.setZero(count_f,1);
    X_2k = fusion_Fai*g_fusion_lX;

    MatrixXd R_inv;
    //滤波增益
    MatrixXd K_k;       //当前时刻的滤波增益
    int count_R;
    if (g_pfscenter->zupt_type == g_pfscenter->zupt_stop)
    {
        count_R = 12;
        R_inv.resize(count_R,count_R);
        R_inv = MatrixXd::Zero(count_R,count_R);
        K_k.resize(count_f,count_R);
        K_k.setZero(count_f,count_R);
    }
    else if (g_pfscenter->zupt_type == g_pfscenter->zupt_linear)
    {
        count_R = 11;
        R_inv.resize(count_R,count_R);
        R_inv = MatrixXd::Zero(count_R,count_R);
        K_k.resize(count_f,count_R);
        K_k.setZero(count_f,count_R);
    }
    //用于稀疏矩阵求逆
    typedef SparseMatrix<double,ColMajor,int> SpMatType;
    typedef Triplet<double> Tpt;
    //求fusion_Q的逆矩阵
    int Q_nonzero_num = fusion_Q.nonZeros();
    int Q_nonzero_r[Q_nonzero_num];
    int Q_nonzero_c[Q_nonzero_num];
    int Qcount_size = 0;
    int Qrows = fusion_Q.rows(),Qcols = fusion_Q.cols();
    for (int j = 0; j < Qcols; j++)
    {
        for (int i = 0; i < Qrows; i++)
        {
            if (fusion_Q(i,j) != 1e-8)
            {
                Q_nonzero_r[Qcount_size] = i;
                Q_nonzero_c[Qcount_size] = j;
                Qcount_size++;
            }   
        } 
    }
    vector<Tpt> Q_tripletlist;
    Q_tripletlist.reserve(Q_nonzero_num);   
    for (int i = 0; i < Q_nonzero_num; i++)
    {
        double val = fusion_Q(Q_nonzero_r[i],Q_nonzero_c[i]);
        Q_tripletlist.push_back(Tpt(Q_nonzero_r[i],Q_nonzero_c[i],val));
    }
    SpMatType Q_spA(Qrows,Qcols);
    Q_spA.setFromTriplets(Q_tripletlist.begin(),Q_tripletlist.end());
    Q_spA.makeCompressed(); //压缩优化矩阵
    vector<Tpt>().swap(Q_tripletlist);
    SparseQR<SpMatType,COLAMDOrdering<int> > Q_solver;
    Q_solver.compute(Q_spA);
    SpMatType Q_Sp_I(Qrows,Qcols);
    Q_Sp_I.setIdentity();
    MatrixXd Q_inv(Qrows,Qcols);
    Q_inv = Q_solver.solve(Q_Sp_I);
    R_inv = g_fusion_R.inverse();

    //计算信息矩阵
    MatrixXd I_2k;      //信息矩阵(从k-1至k)
    I_2k.resize(count_f,count_f);
    I_2k.setZero(count_f,count_f);
    MatrixXd I_k;       //k时刻信息矩阵
    I_k.resize(count_f,count_f);
    I_k.setZero(count_f,count_f);
    MatrixXd temp_1;
    temp_1.resize(count_f,count_f);
    temp_1 = MatrixXd::Zero(count_f,count_f);
    temp_1 = fusion_Fai.transpose()*Q_inv*fusion_Fai;
    MatrixXd temp_2;
    temp_2.resize(count_f,count_f);
    temp_2 = MatrixXd::Zero(count_f,count_f);
    temp_2 = g_fusion_lI + temp_1;
    MatrixXd temp_3;
    temp_3.resize(count_f,count_f);
    temp_3 = MatrixXd::Zero(count_f,count_f);
    temp_3 = Q_inv*fusion_Fai*temp_2.inverse();
    MatrixXd temp_4;
    temp_4.resize(count_f,count_f);
    temp_4 = MatrixXd::Zero(count_f,count_f);
    temp_4 = temp_3*fusion_Fai.transpose()*Q_inv;
    I_2k = Q_inv - temp_4;
    //信息矩阵
    I_k = I_2k + fusion_H.transpose()*R_inv*fusion_H;
    //求稀疏矩阵I_k的逆矩阵
    int nonzero_num = I_k.nonZeros();   //非零元素个数
    int nonzero_r[nonzero_num];
    int nonzero_c[nonzero_num];
    int count_size = 0;
    int rows = I_k.rows(),cols = I_k.cols();
    for (int j = 0; j < cols; j++)
    {
        for (int i = 0; i < rows; i++)
        {
            if (I_k(i,j) != 1e-8)
            {
                nonzero_r[count_size] = i;
                nonzero_c[count_size] = j;
                count_size++;
            }   
        } 
    }
    vector<Tpt> tripletlist;
    tripletlist.reserve(nonzero_num);   //为25个非零元素分配空间
    for (int i = 0; i < nonzero_num; i++)
    {
        double val = I_k(nonzero_r[i],nonzero_c[i]);
        tripletlist.push_back(Tpt(nonzero_r[i],nonzero_c[i],val));
    }
    SpMatType spA(rows,cols);
    spA.setFromTriplets(tripletlist.begin(),tripletlist.end());
    spA.makeCompressed(); //压缩优化矩阵
    vector<Tpt>().swap(tripletlist);
    SparseQR<SpMatType,COLAMDOrdering<int> > solver;
    solver.compute(spA);
    SpMatType Sp_I(rows,cols);
    Sp_I.setIdentity();
    SpMatType I_k_inv(rows,cols);
    I_k_inv = solver.solve(Sp_I);
    //滤波增益
    K_k = I_k.inverse()*fusion_H.transpose()*R_inv;

    //当前时刻状态参数
    VectorXd fusion_Z;      //  测量量
    calculateMeaZ(high_freq_match,low_freq_match,C_bn,fusion_Z);
    
    g_fusion_X = X_2k + K_k*(fusion_Z - fusion_H*X_2k);

    //计算置信度
    float pose_confidence = 0.0;
    calculatePoseConfidence(fusion_Z, fusion_H, I_k,pose_confidence);
    g_fusion_data.pose_confidence = pose_confidence;

    //融合结果
    //计算姿态角
    // double delta_Q[4];      //姿态误差角引起的误差四元素
    // delta_Q[0] = 1;
    // delta_Q[1] = g_fusion_X(0)/2;
    // delta_Q[2] = g_fusion_X(1)/2;
    // delta_Q[3] = g_fusion_X(2)/2;
    // double qua[4];
    // transForAngletoQuaternions(atitude, qua);
    // double Q_qua[4];          //补偿后的姿态四元数
    // Q_qua[0] = delta_Q[0]*qua[0] - delta_Q[1]*qua[1] - delta_Q[2]*qua[2] - delta_Q[3]*qua[3];
    // Q_qua[1] = delta_Q[0]*qua[1] + delta_Q[1]*qua[0] + delta_Q[2]*qua[3] - delta_Q[3]*qua[2];
    // Q_qua[2] = delta_Q[0]*qua[2] + delta_Q[2]*qua[0] + delta_Q[3]*qua[1] - delta_Q[1]*qua[3];
    // Q_qua[3] = delta_Q[0]*qua[3] + delta_Q[3]*qua[0] + delta_Q[1]*qua[2] - delta_Q[2]*qua[1];
    // Matrix3d C_bn1;
    // transForCoordBodytoNavigation(Q_qua, C_bn1);
    // float att[3]={0.0};       //补偿后的姿态角
    // transForCbntoAtitude(C_bn1, att);
    // g_fusion_data.yaw = att[0];
    // g_fusion_data.pitch = att[1];
    // g_fusion_data.roll = att[2];

    double qua[4];
    transForAngletoQuaternions(atitude, qua);
    Matrix3d C_bn0;
    transForCoordBodytoNavigation(qua, C_bn0); 
    Matrix3d degree_fai;
    degree_fai << 0, -g_fusion_X(2), g_fusion_X(1),
                 g_fusion_X(2), 0 , -g_fusion_X(0),
                 -g_fusion_X(1), g_fusion_X(0), 0;
    Matrix3d degtp;
    Matrix3d degI;
    degI.setIdentity();
    degtp = degI - degree_fai;
    Matrix3d C_bn1;
    C_bn1 = degtp.inverse()*C_bn0;
    float att[3];       //补偿后的姿态角
    transForCbntoAtitude(C_bn1, att); 
    g_fusion_data.yaw = att[0];
    g_fusion_data.pitch = att[1];
    g_fusion_data.roll = att[2];

    //计算补偿后的速度
    g_fusion_data.velocity.linear.x = high_freq_match.vel.venu.vx + g_fusion_X(3);
    g_fusion_data.velocity.linear.y = high_freq_match.vel.venu.vy + g_fusion_X(4);
    g_fusion_data.velocity.linear.z = high_freq_match.vel.venu.vz + g_fusion_X(5);

    //计算补偿后的位置
    double lan_lon_m[2] = {g_fusion_X(6),g_fusion_X(7)};
    double lan_lon_deg[2];
    double LLH_new[3] = {high_freq_match.pos.lan,high_freq_match.pos.lon,high_freq_match.pos.h};
    transForMetertoDegree(lan_lon_m,LLH_new,lan_lon_deg);
    g_fusion_data.pose_llh.x = high_freq_match.pos.lan + lan_lon_deg[0];
    g_fusion_data.pose_llh.y = high_freq_match.pos.lon + lan_lon_deg[1];
    g_fusion_data.pose_llh.z = high_freq_match.pos.h + g_fusion_X(8);

    //加速度
    g_fusion_data.accel.linear.x = high_freq_match.accel.ax;
    g_fusion_data.accel.linear.y = high_freq_match.accel.ay;
    g_fusion_data.accel.linear.z = high_freq_match.accel.az;

    //车体坐标系下角速度
    g_fusion_data.velocity.angular.x = high_freq_match.vel.wxyz.wx;
    g_fusion_data.velocity.angular.y = high_freq_match.vel.wxyz.wy;
    g_fusion_data.velocity.angular.z = high_freq_match.vel.wxyz.wz;

    //传感器融合状态
    g_fusion_data.fuse_state = 9;
    g_pfscenter->g_fusion_data_calib = g_fusion_data;
    
    //存储结果用于下次滤波
    g_last_tm = time;
    g_fusion_lI = I_k;
    g_fusion_lX = g_fusion_X;

    return true;
}