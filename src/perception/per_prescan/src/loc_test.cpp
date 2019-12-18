#include "location_interpolation.h"
#include "ros/ros.h"

using namespace std;
using namespace superg_agv::drivers;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "loc_test");
  ros::NodeHandle n;

  //帧 1帧75包 //包 1包12块 //块 1块2组  //组 16个点数据  //通道 每个激光器一通道
  pcl::PointCloud<pcl::PointXYZI> frame_pcl_points;

  std::vector<double> bag_points_time_vec;
  std::vector<LocationMathInfo> bag_points_location_shift_ver;
  std::vector<LocationMathInfo> frame_points_location_shift_ver;
  std::vector<std::vector<double> > frame_points_shift_ver;
  std::vector<std::vector<double> > bag_points_shift_ver;

  LocationMathInfo first_bag_location_info;
  LocationMathInfo cur_location_info;
  LocationMathInfo last_location_info;
  LocationMathInfo cur_location_coefficient;

  double bag_start_time;
  double bag_point_duration;
  double bag_time_delay;
  int bag_point_amount;

  updateLineLocationCoefficient(last_location_info, cur_location_info, cur_location_coefficient);

  mathLidarPointsTimeVec(bag_start_time, bag_point_amount, bag_point_duration, bag_points_time_vec);
  mathPointsShift(first_bag_location_info, cur_location_info, cur_location_coefficient, bag_points_time_vec,
                  bag_points_shift_ver);
  addBagPointsShiftVer2Frame(frame_points_shift_ver, bag_points_shift_ver);

  doLidarPointsCorrect(frame_pcl_points, frame_points_shift_ver);

  return 0;
}
