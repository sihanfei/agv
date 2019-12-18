#ifndef CARTESIAN_FRENET_CONVERTER_H_
#define CARTESIAN_FRENET_CONVERTER_H_

#include "common/utils.h"
#include "common/frenet_point.h"
#include "common/cartesian_point.h"
#include "common/reference_line.h"

namespace planning{

//#define M_PI 3.14159265358979323846 //pi

//~CartesianFrenetConverter();
//实现笛卡尔坐标系到frenet坐标系的转换
bool cartesianToFrenet(const CartesianPoint p0, const ReferenceLine& ref_line, FrenetPoint& frenet_point);
//实现frenet坐标系到笛卡尔坐标系的转换
bool frenetToCartesian(const FrenetPoint slp0, const ReferenceLine& ref_line, CartesianPoint& cartesian_point);
//ReferenceLine rp0_test;
//ReferenceLine vrp;

//求两点间距离的平方
double funcDistanceSquare(const double x, const double y, const double r_x, const double r_y);
//车身朝向角限制在(-pi,pi)之间
double normalizeAngle(const double angle);
//角度插值
double slerp(const double a0, const double a1, const double k);
//向量法计算投影点到投影线上一点的长度占总长度的比例
double calcRatio(const CartesianPoint& p0, const ReferenceLinePoint& p1, const ReferenceLinePoint &p2);
//各参数线性插值
ReferenceLinePoint linearInterpolation(const ReferenceLinePoint& p1,const ReferenceLinePoint& p2,const CartesianPoint& p0);
ReferenceLinePoint linearInterpolation(const ReferenceLinePoint &p1, const ReferenceLinePoint &p2, const FrenetPoint &p0);

//计算转换到frenet坐标系的输出(l,l',l'',s',s'')
double calcL(const CartesianPoint &p0, const ReferenceLinePoint& rp0);
double calcDl(const CartesianPoint &p0, const ReferenceLinePoint& rp0,const double l, const double tan_delta_theta);
double calcDdl(const CartesianPoint &p0, const ReferenceLinePoint &rp0,
               const double l, const double dl_ds,
               const double tan_delta_theta, const double cos_delta_theta);
double calcDs(const CartesianPoint &p0, const ReferenceLinePoint &rp0,
              const double l, const double cos_delta_theta);
double calcDds(const CartesianPoint &p0, const ReferenceLinePoint &rp0,
               const double l, const double dl_ds, const double ds_dt,
               const double tan_delta_theta, const double cos_delta_theta);
//计算并返回(l,l',l'',s',s'')
FrenetPoint calcFrenetPoint(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double delta_theta);

} // namespace bjoy_decision

#endif // CARTESIAN_FRENET_CONVERTER_H_
