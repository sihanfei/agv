#ifndef COORDINATE_H
#define COORDINATE_H

#include <eigen3/Eigen/Dense>

using namespace Eigen;

//由经纬高转到WGS-84
void transLLHtoWGS84(Vector3d &llh_pos, Vector3d &WGSpose);
//将车辆位置转换为港区坐标
//@param:输入:llh:车辆的经纬高坐标(顺序:纬经高)
//@param:输出:henu:车辆在港区下的坐标
void transFusionLLHtoHarbourENU(Vector3d &llh, Vector3d &henu);

#endif
