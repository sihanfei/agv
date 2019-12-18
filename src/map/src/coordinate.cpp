#include "coordinate.h"

#define Ra 6378137.0           //地球长半轴
#define Rb 6356752.314         //地球短半轴
#define FF 1.0 / 298.257223563 //扁率
//港区原点坐标(需要实际测量)
#define Har_lan0 22.67080000 * M_PI / 180 //  弧度
#define Har_lon0 114.33280000 * M_PI / 180
#define Har_h0 59.537175
#define ex2_ (2 - FF) * FF
#define N_ Ra / sqrt(1.0 - ex2_ * sin(Har_lan0) * sin(Har_lan0))
#define Har_wgsx0 (N_ + Har_h0) * cos(Har_lan0) * cos(Har_lon0)
#define Har_wgsy0 (N_ + Har_h0) * cos(Har_lan0) * sin(Har_lan0)
#define Har_wgsz0 (N_ * (1 - ex2_) + Har_h0) * sin(Har_lan0)

void transLLHtoWGS84(Vector3d &llh_pos, Vector3d &WGSpose)
{

  double lat_radian = llh_pos(0) * M_PI / 180; //维度弧度
  double lon_radian = llh_pos(1) * M_PI / 180; //经度弧度
  double hgt_       = llh_pos(2);              //  高度

  double ex2 = (2 - FF) * FF; // e（椭球偏心率）的平方
  double N   = Ra / sqrt(1.0 - ex2 * sin(lat_radian) * sin(lat_radian));

  WGSpose(0) = (N + hgt_) * cos(lat_radian) * cos(lon_radian); //  x
  WGSpose(1) = (N + hgt_) * cos(lat_radian) * sin(lat_radian); //  y
  WGSpose(2) = (N * (1 - ex2) + hgt_) * sin(lat_radian);       // z
}

void transFusionLLHtoHarbourENU(Vector3d &llh, Vector3d &henu)
{

  Vector3d wgspose;

  transLLHtoWGS84(llh, wgspose);

  Matrix3d trans_S; //港区转换坐标
  // trans_S.resize(3, 3);
  // trans_S = MatrixXd::Zero(3,3);
  trans_S(0, 0) = -sin(Har_lon0);
  trans_S(0, 1) = cos(Har_lon0);
  trans_S(0, 2) = 0.0;
  trans_S(1, 0) = -sin(Har_lan0) * cos(Har_lon0);
  trans_S(1, 1) = -sin(Har_lan0) * sin(Har_lon0);
  trans_S(1, 2) = cos(Har_lan0);
  trans_S(2, 0) = cos(Har_lan0) * cos(Har_lon0);
  trans_S(2, 1) = cos(Har_lan0) * sin(Har_lon0);
  trans_S(2, 2) = sin(Har_lan0);
  // trans_S << -sin(Har_lon0), cos(Har_lon0), 0.0,
  //             -sin(Har_lan0)*cos(Har_lon0), -sin(Har_lan0)*sin(Har_lon0), cos(Har_lan0),
  //             cos(Har_lan0)*cos(Har_lon0), cos(Har_lan0)*sin(Har_lon0), sin(Har_lan0);
  Vector3d delta_wgs;
  // delta_wgs.resize(3);
  delta_wgs = Vector3d::Zero();
  delta_wgs(0, 0) = wgspose[0] - Har_wgsx0;
  delta_wgs(1, 0) = wgspose[1] - Har_wgsy0;
  delta_wgs(2, 0) = wgspose[2] - Har_wgsz0;
  // IMU港区坐标
  Vector3d delta_enu;
  // delta_enu.resize(3);
  delta_enu = Vector3d::Zero();
  delta_enu = trans_S * delta_wgs;

  //车辆中心点港区坐标
  Vector2d delta_center_IMU; //车辆IMU与中心点偏差
  // delta_center_IMU.resize(2);
  delta_center_IMU = Vector2d::Zero();
  delta_center_IMU(0, 0) = -0.027;  //车体坐标系下x轴向偏差
  delta_center_IMU(1, 0) = -6.5227; //车体坐标系下y轴偏差
  Matrix2d trans_IMU_center;
  // trans_IMU_center.resize(2, 2);
  trans_IMU_center = Matrix2d::Zero(2, 2);
  float yaw        = 0;
  trans_IMU_center(0, 0) = cos(yaw);
  trans_IMU_center(0, 1) = sin(yaw);
  trans_IMU_center(1, 0) = -sin(yaw);
  trans_IMU_center(1, 1) = cos(yaw);
  // trans_IMU_center << cos(yaw), sin(yaw),
  //                     -sin(yaw),cos(yaw);
  Vector2d delta_xyz; //港区坐标系下车辆IMU与中心点的偏差
  // delta_xyz.resize(2);
  delta_xyz = Vector2d::Zero();
  delta_xyz = trans_IMU_center * delta_center_IMU;
  Vector3d delta_tp;
  // delta_tp.resize(3);
  delta_tp = Vector3d::Zero();
  delta_tp(0, 0) = delta_xyz(0);
  delta_tp(1, 0) = delta_xyz(1);
  delta_tp(2, 0) = 0.0;
  // delta_xyz(2) = 0.0;
  // vel_center_enu.resize(3);
  henu = Vector3d::Zero();
  henu(0, 0) = delta_enu(0) + delta_tp(0);
  henu(1, 0) = delta_enu(1) + delta_tp(1);
  henu(2, 0) = delta_enu(2) + delta_tp(2);
}