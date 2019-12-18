#include "get_ref_line.h"
#include "ros/ros.h"

using namespace std;
using namespace superg_agv::map;

#define MAP_REF_POINT_CONNECT_PATH "/home/wrll/work/superg_agv/src/routing/map_new/data/ref_line.json"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ref_get");
  ros::NodeHandle n;

  GetRefLine get_ref_line;

  std::map<int, RefLine> ref_line_map;
  std::map<int, std::vector<RefPoints> > ref_points_map;
  std::map<int, AppendixAttribute> appendix_attribute_map;
  std::vector<RefPoints> ref_points_vec;

  // std::vector< BoundrayLine > boundray_line_vec;
  // std::vector< BoundrayCircle > boundray_circle_vec;
  std::map<int, BoundrayLine> boundray_line_map;
  std::map<int, BoundrayCircle> boundray_circle_map;
  std::vector<BoundrayPoint> boundray_points_vec;

  std::string home_path = getenv("HOME");
  std::string workplace_path = "/work/superg_agv/src/routing/map_new/data";
  std::string ref_line_file_name = home_path + workplace_path + "/ref_line.json";
  std::string ref_line_appendix_file_name = home_path + workplace_path + "/appendix_belongs.json";
  std::string ref_point_file_name = home_path + workplace_path + "/ref_points.json";
  std::string appendix_attribute_name = home_path + workplace_path + "/appendix_attribute.json";
  std::string pysical_circle_name = home_path + workplace_path + "/physical_circle.json";
  std::string pysical_line_name = home_path + workplace_path + "/physical_line.json";
  std::string pysical_points_name = home_path + workplace_path + "/physical_points.json";

  get_ref_line.setRefLineInit();
  get_ref_line.getRefLineFromFile2Map(ref_line_file_name, ref_line_map);
  get_ref_line.getAppendixFromFile2RefLine(ref_line_appendix_file_name, ref_line_map);
  get_ref_line.getRefPointFromFile2Map(ref_point_file_name, ref_line_map, ref_points_map, ref_points_vec);
  get_ref_line.getAppendixAttributeFromFile2Map(appendix_attribute_name, appendix_attribute_map);
  get_ref_line.getBoundrayCircleFromFile2Map(pysical_circle_name, boundray_circle_map);
  get_ref_line.getBoundrayLineFromFile2Map(pysical_line_name, boundray_line_map);
  get_ref_line.getBoundrayPointsFromFile2Ver(pysical_points_name, boundray_points_vec);

  // RefPoints ref_point_temp;
  // ref_point_temp.point.x = -16.0416;
  // ref_point_temp.point.y = 49.3437;
  // ref_point_temp.point.z = 0.0;
  // ref_point_temp.theta   = 5.88;

  // get_ref_line.findLineFromRefPointVecWithXYZHeading(ref_point_temp, ref_points_vec);

  return 0;
}
