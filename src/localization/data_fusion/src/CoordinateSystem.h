#ifndef COORDINATESYSTEM_H
#define COORDINATESYSTEM_H

#include "FusionCenter.h"
#include "FusionFixedLidar.h"
#include "FusionLidar.h"
#include "FusionUWB.h"
#include "FusionZUPTFixedLidar.h"
#include "FusionZUPTLidar.h"
#include "FusionZUPTUWB.h"
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <sstream>
#include <string>
using namespace Eigen;

#define Ra (6378137.0)           //地球长半轴
#define Rb (6356752.3141)        //地球短半轴
#define ff (1.0 / 298.257223563) //扁率
#define w_ie (7.2921151467e-5)   //地球旋转角速度(弧度)
//港区原点坐标(需要实际测量)
#define e_sec2 ((Ra * Ra - Rb * Rb) / (Rb * Rb)) //椭球第二偏心率的平方 (c/b)^2 0.006739496742
#define ratio_k0 (1.0000)                        //中央子午线比例系数
//镇江港
// #define Har_e0 (465070.610)
// #define Har_n0 (3562440.741)
// #define Har_u0 (13.633)
// #define lontitude0  (120.0)               //UTM投影区域内的中央子午线经度(用于UTM转LLH)
//宁波港
#define Har_e0 (397356.105)
#define Har_n0 (3313796.613)
#define Har_u0 (17.366)
#define lontitude0 (123.0) // UTM投影区域内的中央子午线经度(用于UTM转LLH)

//由经纬高转到WGS-84
void transLLHtoWGS84(double llh_pos[3], double WGSpose[3]);
//由WGS-84转到经纬高
void transWGS84toLLH(double WGS[3], double llh_pos[3]);
//速度由东北天转到WGS-84
void transENUtoWGS84(double venu[3], double llh_pos[3], double WGSpose[3]);
//速度由84转到东北天
void transWGS84toENU(double WGSpose[3], double llh_pos[3], double venu[3]);
void transFusionLLHtoHarbourENU(location_msgs::FusionDataInfo &pose_in);
//将IMU位置,速度都转换为车辆中心点港区坐标
void transForAngletoQuaternions(float (&atitude)[3], double (&quaternions)[4]);
void transForCoordBodytoNavigation(double quaternions[4], Matrix3d &C_bn);
// n为东北天
void transForCbntoAtitude(Matrix3d &C_bn, float (&att)[3]);
// c从坐标变换矩阵得到姿态角,顺序yaw,pitch,roll
void transForMetertoDegree(double (&lan_lon_m)[2], double (&LLH)[3], double (&lan_lon_deg)[2]);
//将纬度，经度误差量lan_lonm（单位米，顺序纬度，经度）转换成纬度，经度误差量lan_lon_deg(单位：度，顺序纬度，经度)
void transForDegreetoMeter(double (&lan_lon_deg)[2], double (&LLH)[3], double (&lan_lon_m)[2]);
//将纬度，经度误差量lan_lon_deg(单位：度，顺序纬度，经度)转换成纬度，经度误差量lan_lonm（单位米，顺序纬度，经度）
double transForLLHtoUTM(double (&LLH)[3], double (&UTM_ENU)[3]);
//将经纬高转换UTM平面坐标,输入LLH为纬经高(deg),输出UTM_ENU经UTM转换过的东北天(m)
void transForUTMtoLLH(double (&UTM_ENU)[3], double (&LLH)[3], double &lon0);
//将UTM平面坐标转换经纬高,输入UTM_ENU东北天(m),输出LLH为纬经高(deg)
void transForAngletoCnb(double (&atitude)[3], Matrix3d &C_nb);
//计算Cnb,atitude顺序yaw,pitch,roll,输出Cnb矩阵
//车辆中心点SLAM坐标转为经纬高坐标,输入Har_ENU0为开始进行slam时的起点车辆中心港区坐标(m),slam_enu为车辆中心slam坐标东北天(m),当前车辆yaw角度
//输出LLH为定位天线的经纬高(deg)
void transForSLAMtoLLH(double (&Har_ENU0)[3], double (&slam_enu)[3], double &yaw, double &lon0, double (&LLH)[3]);

#endif