#include "math/cartesian_frenet_converter.h"

namespace planning
{

//笛卡尔坐标系到frenet坐标系转换
bool cartesianToFrenet(const CartesianPoint p0, const ReferenceLine &ref_line, FrenetPoint &frenet_point)
{
  const int num = ref_line.getReferenceLinePointsSize();

  if (num < 2) //传来参考线点小于2个，不满足转换条件
    return false;

  double d_min = funcDistanceSquare(p0.getX(), p0.getY(), ref_line.getReferenceLinePointByIndex(0).getX(),
                                    ref_line.getReferenceLinePointByIndex(0).getY());

  uint32_t index_min = 0;

  for (uint32_t i = 1; i < num; i++)
  {
    double d_temp = funcDistanceSquare(p0.getX(), p0.getY(), ref_line.getReferenceLinePointByIndex(i).getX(),
                                       ref_line.getReferenceLinePointByIndex(i).getY());
    if (d_temp < d_min)
    {
      d_min     = d_temp;
      index_min = i;
    }
  }

  uint32_t index_start = (index_min == 0 ? index_min : index_min - 1);
  uint32_t index_end   = (index_min + 1 == num ? index_min : index_min + 1);

  if (index_start != index_min && index_end != index_min)
  {
    const double ratio1 = calcRatio(p0, ref_line.getReferenceLinePointByIndex(index_start),
                                    ref_line.getReferenceLinePointByIndex(index_min));
    const double ratio2 = calcRatio(p0, ref_line.getReferenceLinePointByIndex(index_min),
                                    ref_line.getReferenceLinePointByIndex(index_end));
    index_end   = (ratio1 >= 0.0 && ratio1 < 1.0 ? index_min : index_end);
    index_start = (ratio2 >= 0.0 && ratio2 <= 1.0 ? index_min : index_start);

    // ROS_INFO("ratio1=%f,ratio2=%f,index_end=%d,index_start=%d",ratio1,ratio2,index_end,index_start);
  }

  //  const ReferenceLinePoint reference_point_start = ref_line.getReferenceLinePointByIndex(index_start);
  //  const ReferenceLinePoint reference_point_end = ref_line.getReferenceLinePointByIndex(index_end);

  //  const double ratio_test = calcRatio(p0, reference_point_start, reference_point_end);
  //  if(ratio_test < 0 || ratio_test > 1)
  //  {
  //    ROS_ERROR("There are no subpoints on this reference line...");
  //    return false;//当前点在找到的参考线两点间无插值点
  //  }

  //  const ReferenceLinePoint rp0 = linearInterpolation(reference_point_start, reference_point_end, p0);

  ReferenceLinePoint rp0;
  if (index_end == index_start)
  {
    rp0 = ref_line.getReferenceLinePointByIndex(index_start);
  }
  else
  {
    const ReferenceLinePoint reference_point_start = ref_line.getReferenceLinePointByIndex(index_start);
    const ReferenceLinePoint reference_point_end   = ref_line.getReferenceLinePointByIndex(index_end);

    // ROS_INFO("Transform:reference_point_start(s=%f,x=%f,y=%f,theta=%f,kappa=%f,dkappa=%f),reference_point_end(s=%f,x=%"
    //          "f,y=%f,theta=%f,kappa=%f,dkappa=%f)",
    //          reference_point_start.getS(), reference_point_start.getX(), reference_point_start.getY(),
    //          reference_point_start.getTheta(), reference_point_start.getKappa(), reference_point_start.getdKappa(),
    //          reference_point_end.getS(), reference_point_end.getX(), reference_point_end.getY(),
    //          reference_point_end.getTheta(), reference_point_end.getKappa(), reference_point_end.getdKappa());

    // ROS_INFO("Transform:p0(x=%f,y=%f,theta=%f,vel=%f,acc=%f,kappa=%f)", p0.getX(), p0.getY(), p0.getTheta(),
    //          p0.getVel(), p0.getAcc(), p0.getKappa());

    const double ratio_test = calcRatio(p0, reference_point_start, reference_point_end);


    // 若插值区间不合理，则动态调整修改插值区间，20190725，LZC
    bool trans_flag = false;
    uint32_t i_max = 0;

    if (ratio_test < 0.0)
    {
      ROS_WARN("Transform:Ratio < 0,look forward 2 reference line.");
      if (index_start - 2 > 0)
      {
        i_max = 2;
      }
      else
      {
        i_max = 2 - index_start;
      }

      for (uint32_t j = 1; j < i_max; j++)
      {
        // 插值区间往前推
        const ReferenceLinePoint reference_point_start = ref_line.getReferenceLinePointByIndex(index_start - j);

        const double ratio_test_temp = calcRatio(p0, reference_point_start, reference_point_end);

        if (ratio_test_temp < 0.0)
        {
          continue;
        }
        else
        {
          trans_flag = true;
          break;
        }
      }

      if (trans_flag == false)
      {
        ROS_WARN("Transform:There are no subpoints on this reference line.");
        return false; //当前点在找到的参考线两点间无插值点
      }
    }
    else if (ratio_test > 1.0)
    {
      ROS_WARN("Transform:Ratio > 1,look after 2 reference line.");
      if (index_end + 2 < num)
      {
        i_max = 2;
      }
      else
      {
        i_max = num - index_end;
      }

      for (int j = 1; j < i_max; j++)
      {
        // 插值区间往后推
        const ReferenceLinePoint reference_point_end = ref_line.getReferenceLinePointByIndex(index_end + j);

        const double ratio_test_temp = calcRatio(p0, reference_point_start, reference_point_end);

        if (ratio_test_temp > 1.0)
        {
          continue;
        }
        else
        {
          trans_flag = true;
          break;
        }
      }

      if (trans_flag == false)
      {
        ROS_WARN("Transform:There are no subpoints on this reference line.");
        return false; //当前点在找到的参考线两点间无插值点
      }
    }


//    if (ratio_test < 0.0 || ratio_test > 1.0)
//    {
//      // std::cout<<"111111111"<<std::endl;
//      // ROS_ERROR("Transform:There are no subpoints on this reference line.");
//      ROS_WARN("Transform:There are no subpoints on this reference line.");
//      return false; //当前点在找到的参考线两点间无插值点
//    }
    // std::cout<<"2222222222"<<std::endl;

    rp0 = linearInterpolation(reference_point_start, reference_point_end, p0);
  }

  // rp0_test = rp0;

  // ROS_INFO("Test:p0 theta=%f,rp0 theta=%f", p0.getTheta(), rp0.getTheta());
  const double delta_theta = p0.getTheta() - rp0.getTheta();
  if (fabs(delta_theta) >= 0.5 * PI && fabs(fabs(delta_theta) - 2 * PI) >= 0.5 * PI)
  {
    // ROS_ERROR("Transform:The gap between target pose and ref line is too large");
    ROS_WARN("Transform:The gap between target pose and ref line is too large");
    return false;
  }
  frenet_point = calcFrenetPoint(p0, rp0, delta_theta);
  // frenet_point.t = p0.t;

  return true;
}

// frenet坐标系到笛卡尔坐标系转换
bool frenetToCartesian(const FrenetPoint slp0, const ReferenceLine &ref_line, CartesianPoint &cartesian_point)
{
  const int num = ref_line.getReferenceLinePointsSize();
  //参考线点数量小于2个
  if (num < 2)
  {
    return false;
  }

  double ds_min       = pow((ref_line.getReferenceLinePointByIndex(0).getS() - slp0.getS()), 2);
  double ds_min_index = 0.0;
  for (int i = 1; i < num; i++)
  {
    double ds_temp = pow((ref_line.getReferenceLinePointByIndex(i).getS() - slp0.getS()), 2);
    if (ds_temp < ds_min)
    {
      ds_min       = ds_temp;
      ds_min_index = i;
    }
  }

  uint32_t index_start                           = (ds_min_index == 0 ? ds_min_index : ds_min_index - 1);
  uint32_t index_end                             = (ds_min_index + 1 == num ? ds_min_index : ds_min_index + 1);
  const ReferenceLinePoint reference_point_start = ref_line.getReferenceLinePointByIndex(index_start);
  const ReferenceLinePoint reference_point_end   = ref_line.getReferenceLinePointByIndex(index_end);

  if (slp0.getS() < reference_point_start.getS() ||
      slp0.getS() > reference_point_end.getS()) //当前点在找到的参考线两点间无插值点
    return false;

  const ReferenceLinePoint rp0 = linearInterpolation(reference_point_start, reference_point_end, slp0);

  // const ReferenceLinePoint rp0 = ref_line.ref_line_points[ds_min_index];
  const double cos_theta_r = std::cos(rp0.getTheta());
  const double sin_theta_r = std::sin(rp0.getTheta());
  //计算转换到cartesian坐标系的x,y
  const double cartesian_x = rp0.getX() - sin_theta_r * -slp0.getL();
  const double cartesian_y = rp0.getY() + cos_theta_r * -slp0.getL();

  const double part_tan_delta_theta = 1.0 - rp0.getKappa() * -slp0.getL();
  const double tan_delta_theta      = slp0.getdL() / part_tan_delta_theta;

  const double delta_theta     = std::atan2(slp0.getdL(), part_tan_delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);

  //计算theta
  const double cartesian_theta = normalizeAngle(delta_theta + rp0.getTheta());

  const double dkdl = rp0.getdKappa() * -slp0.getL() + rp0.getKappa() * slp0.getdL();
  //计算k
  const double cartesian_k =
      ((slp0.getddL() + dkdl * tan_delta_theta) * cos_delta_theta * cos_delta_theta / part_tan_delta_theta +
       rp0.getKappa()) *
      cos_delta_theta / part_tan_delta_theta;
  const double l_dot = slp0.getdL() * slp0.getdS();
  //计算v
  const double cartesian_v =
      std::sqrt(part_tan_delta_theta * part_tan_delta_theta * slp0.getdS() * slp0.getdS() + l_dot * l_dot);

  const double delta_theta_prime = part_tan_delta_theta * cartesian_k / cos_delta_theta - rp0.getKappa();
  //计算a
  const double cartesian_a = slp0.getddS() * part_tan_delta_theta / cos_delta_theta +
                             slp0.getdS() * slp0.getdS() / cos_delta_theta * (delta_theta_prime * slp0.getdL() - dkdl);

  // CartesianPoint(double x,double y,double theta,double kappa,double vel,double acc,double relative_time)
  // cartesian_point =
  // CartesianPoint(cartesian_x,cartesian_y,cartesian_theta,cartesian_k,cartesian_v,cartesian_a,slp0.t);

  cartesian_point.setX(cartesian_x);
  cartesian_point.setY(cartesian_y);
  cartesian_point.setTheta(cartesian_theta);
  cartesian_point.setKappa(cartesian_k);
  cartesian_point.setVel(cartesian_v);
  cartesian_point.setAcc(cartesian_a);
  cartesian_point.setRelativeTime(slp0.getRelativeTime());

  return true;
}

double funcDistanceSquare(const double x, const double y, const double r_x, const double r_y)
{
  double dx = x - r_x;
  double dy = y - r_y;

  return dx * dx + dy * dy;
}

double normalizeAngle(const double angle)
{
  // double a = std::fmod(angle + PI, PI * 2.0);
  double a = std::fmod(angle, PI * 2.0);
  if (a <= 0.0)
    a += (2.0 * PI);

  // return a - PI;
  return a;
}

double slerp(const double a0, const double a1, const double k)
{
  const double a0_n = normalizeAngle(a0);
  const double a1_n = normalizeAngle(a1);
  double d          = a1_n - a0_n;
  if (d > PI)
    d = d - 2 * PI;
  else if (d < -PI)
    d = d + 2 * PI;

  const double a = a0_n + d * k;
  return normalizeAngle(a);
}

double calcRatio(const CartesianPoint &p0, const ReferenceLinePoint &p1, const ReferenceLinePoint &p2)
{
  const double x0 = p0.getX();
  const double y0 = p0.getY();
  const double x1 = p1.getX();
  const double y1 = p1.getY();
  const double x2 = p2.getX();
  const double y2 = p2.getY();

  const double dx             = x2 - x1;
  const double dy             = y2 - y1;
  const double sum_of_squares = dx * dx + dy * dy;
  if (sum_of_squares < EPSILON)
  {
    ROS_ERROR("Transform:points are too close");
    return -1.0;
  }

  const double length_all        = sqrt(sum_of_squares);
  const double length_projection = ((x0 - x1) * (x2 - x1) + (y0 - y1) * (y2 - y1)) / length_all;
  const double length_ratio      = length_projection / length_all;

  return length_ratio;
}

ReferenceLinePoint linearInterpolation(const ReferenceLinePoint &p1, const ReferenceLinePoint &p2,
                                       const CartesianPoint &p0)
{
  const double x1 = p1.getX();
  const double y1 = p1.getY();
  const double x2 = p2.getX();
  const double y2 = p2.getY();

  double ratio = calcRatio(p0, p1, p2);

  //  if (ratio<0 || ratio>1)
  //    ROS_ERROR("There are no subpoints on this reference line...");

  //插值得到投影点的其他信息：y,theta,曲率，曲率一阶导，s

  // ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa);
  ReferenceLinePoint rp0(p1.getS() + ratio * (p2.getS() - p1.getS()), x1 + ratio * (x2 - x1), y1 + ratio * (y2 - y1),
                         slerp(p1.getTheta(), p2.getTheta(), ratio),
                         p1.getKappa() + ratio * (p2.getKappa() - p1.getKappa()),
                         p1.getdKappa() + ratio * (p2.getdKappa() - p1.getdKappa()));

  return rp0;
}

ReferenceLinePoint linearInterpolation(const ReferenceLinePoint &p1, const ReferenceLinePoint &p2,
                                       const FrenetPoint &p0)
{
  const double s1 = p1.getS();
  const double s2 = p2.getS();
  const double s0 = p0.getS();

  double ratio = 0.0;
  if (std::fabs(s2 - s1) < EPSILON)
  {
    ROS_ERROR("Transform:ReferencePoints are too close.");
    return p1;
  }
  else
  {
    ratio = (s0 - s1) / (s2 - s1);
  }

  // ReferenceLinePoint(double s,double x,double y,double theta,double kappa,double dkappa);
  ReferenceLinePoint rp0(s0, p1.getX() + ratio * (p2.getX() - p1.getX()), p1.getY() + ratio * (p2.getY() - p1.getY()),
                         p1.getTheta() + ratio * (p2.getTheta() - p1.getTheta()),
                         p1.getKappa() + ratio * (p2.getKappa() - p1.getKappa()),
                         p1.getdKappa() + ratio * (p2.getdKappa() - p1.getdKappa()));

  return rp0;
}

double calcL(const CartesianPoint &p0, const ReferenceLinePoint &rp0)
{
  const double dx         = p0.getX() - rp0.getX();
  const double dy         = p0.getY() - rp0.getY();
  const double cos_rtheta = std::cos(rp0.getTheta());
  const double sin_rtheta = std::sin(rp0.getTheta());
  const double l_sign     = dy * cos_rtheta - dx * sin_rtheta;
  const double l          = copysign(std::sqrt(dx * dx + dy * dy), l_sign);

  return l;
}

double calcDl(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double l, const double tan_delta_theta)
{
  const double dl_ds = (1 - rp0.getKappa() * l) * tan_delta_theta;

  return dl_ds;
}

double calcDdl(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double l, const double dl_ds,
               const double tan_delta_theta, const double cos_delta_theta)
{
  const double kr     = rp0.getKappa();
  const double dkr    = rp0.getdKappa();
  const double kx     = p0.getKappa();
  const double ddl_ds = -(dkr * l + kr * dl_ds) * tan_delta_theta +
                        (1 - kr * l) * (kx * (1 - kr * l) / cos_delta_theta - kr) / (cos_delta_theta * cos_delta_theta);

  return ddl_ds;
}

double calcDs(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double l, const double cos_delta_theta)
{
  const double v     = p0.getVel();
  const double kr    = rp0.getKappa();
  const double ds_dt = v * cos_delta_theta / (1 - kr * l);

  return ds_dt;
}

double calcDds(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double l, const double dl_ds,
               const double ds_dt, const double tan_delta_theta, const double cos_delta_theta)
{
  const double kx  = p0.getKappa();
  const double kr  = rp0.getKappa();
  const double dkr = rp0.getdKappa();
  const double ax  = p0.getAcc();
  const double dds_dt_part =
      (1 - kr * l) * tan_delta_theta * (kx * (1 - kr * l) / cos_delta_theta - kr) + dkr * l + kr * dl_ds;
  const double dds_dt = cos_delta_theta / (1 - kr * l) * (ax - ds_dt * ds_dt * dds_dt_part / cos_delta_theta);

  return dds_dt;
}

FrenetPoint calcFrenetPoint(const CartesianPoint &p0, const ReferenceLinePoint &rp0, const double delta_theta)
{
  // const double delta_theta = p0.getTheta() - rp0.getTheta();
  const double tan_delta_theta = std::tan(delta_theta);
  const double cos_delta_theta = std::cos(delta_theta);
  //转换到frenet坐标系的输出(l,l',l'',s',s'')

  const double l             = -calcL(p0, rp0);
  const double dl_ds         = calcDl(p0, rp0, l, tan_delta_theta);
  const double ddl_ds        = calcDdl(p0, rp0, l, dl_ds, tan_delta_theta, cos_delta_theta);
  const double ds_dt         = calcDs(p0, rp0, l, cos_delta_theta);
  const double dds_dt        = calcDds(p0, rp0, l, dl_ds, ds_dt, tan_delta_theta, cos_delta_theta);
  const double relative_time = p0.getRelativeTime();

  FrenetPoint frenet_point(rp0.getS(), ds_dt, dds_dt, l, dl_ds, ddl_ds, relative_time);

  return frenet_point;
}

} // namespace bjoy_decision
