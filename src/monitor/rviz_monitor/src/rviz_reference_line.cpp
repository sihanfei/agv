#include "rviz_reference_line.h" //我们自定义类（class）的头文件
//#include "glog_helper.h"
namespace superg_agv
{
namespace monitor
{
void SplitString(const std::string &s, std::vector< std::string > &v, const std::string &c)
{
  std::string::size_type pos1, pos2;
  pos2 = s.find(c);
  pos1 = 0;
  std::string str;
  while (std::string::npos != pos2)
  {
    str = s.substr(pos1, pos2 - pos1);
    if (str != "")
      v.push_back(str);

    pos1 = pos2 + c.size();
    pos2 = s.find(c, pos1);
  }
  if (pos1 != s.length())
  {
    str = s.substr(pos1);
    if (str != "")
      v.push_back(str);
  }
}
// 不得不通过节点句柄指针进入构造函数再有构造函数去构建subscriber
RvizReferenceLineClass::RvizReferenceLineClass(ros::NodeHandle &nodehandle, ros::NodeHandle &private_nh,
                                               string refline_file_name, string refpoint_file_name, string topic_name)
    : nh_(nodehandle) // 构造函数
{
  ROS_INFO("in class constructor of RvizReferenceLineClass");
  // GLogHelper gh(( char * )"RvizReferenceLineClass");
  // SUPERG_INFO << "in class constructor of RvizReferenceLineClass";

  initializePublishers(topic_name);

  isDataOk_ = initializeReferenceLineData(refline_file_name, refpoint_file_name);

  srv_ = boost::make_shared< dynamic_reconfigure::Server< rviz_monitor::myconfigConfig > >(private_nh);

  dynamic_reconfigure::Server< rviz_monitor::myconfigConfig >::CallbackType f;

  f = boost::bind(&RvizReferenceLineClass::reflineColorDynamicReconfigcallback, this, _1, _2);
  srv_->setCallback(f);

  lane_id_ = 0;
  // val_to_remember_ = 0.0; //初始化储存数据的变量的值
}
void RvizReferenceLineClass::reflineColorDynamicReconfigcallback(rviz_monitor::myconfigConfig &config, uint32_t level)
{
  lane_id_ = config.route_id;
  visualization_msgs::MarkerArray markerArray_title;
  visualization_msgs::Marker title_start, title_end;

  title_start.header.frame_id = title_end.header.frame_id = "/odom";
  title_start.header.stamp = title_end.header.stamp = ros::Time::now();

  title_start.ns = "refline_title_start";
  title_end.ns   = "refline_title_end";

  title_start.action = title_end.action = visualization_msgs::Marker::DELETEALL;

  markerArray_title.markers.emplace_back(title_start);
  markerArray_title.markers.emplace_back(title_end);

  reference_title_pub_.publish(markerArray_title);

  ROS_INFO("refline_configConfig: %d", config.route_id);
}
//以下是一个成员协助函数，读取参考线文件
bool RvizReferenceLineClass::initializeReferenceLineData(string refline_file_name, string refpoint_file_name)
{
  ROS_INFO("RvizReferenceLineClass Initializing ReferenceLineData");
  //获取该计算机系统HOME文件夹路径
  std::string home_path      = getenv("HOME");
  std::string workplace_path = "/work/superg_agv/src/routing/map_new/data/";
  //获取文件路径
  std::string ref_point_file_name = home_path + workplace_path + refpoint_file_name;
  std::string ref_line_file_name  = home_path + workplace_path + refline_file_name;

  //  SUPERG_WARN << "RvizReferenceLineClass:" << ref_point_file_name;

  //读取文件
  getRefLineFromFile2Map(ref_line_file_name, ref_line_map);
  getRefPointFromFile2Map(ref_point_file_name, ref_line_map, ref_points_map, ref_points_vec);

  return true;
}
void RvizReferenceLineClass::getRefLineFromFile2Map(std::__cxx11::string &file_name,
                                                    std::map< int, RefLine > &ref_line_map)
{
  ifstream infile;
  infile.open(file_name); //将文件流对象与文件连接起来
  if (!infile)
  {
    cout << "open file fail!" << endl;
  }
  std::__cxx11::string str;
  std::stringstream ss;
  int get_line_count    = 0;
  int add_refline_count = 0;
  while (getline(infile, str))
  {
    RefLine ref_line_temp;
    ss.clear();
    ss << str.c_str();
    ss2RefLine(ss, ref_line_temp);
    //查找重复
    std::map< int, RefLine >::iterator it = ref_line_map.find(ref_line_temp.ref_line_id);
    if (it == ref_line_map.end())
    {
      ref_line_map.insert(std::pair< int, RefLine >(ref_line_temp.ref_line_id, ref_line_temp));
      ++add_refline_count;
      // cout << "Add " << get_line_count << " line " << ref_line_temp.ref_line_id << " in ref line map, now map size is
      // "
      //      << ref_line_map.size() << endl;
      // ROS_INFO("add %d %s state in map", p_state_->id, AgvStateStr[p_state_->id].c_str());
    }
    else
    {
      // cout << "This " << get_line_count << " line " << ref_line_temp.ref_line_id
      //      << " is during in ref line map, now map size is " << ref_line_map.size() << endl;
      // ROS_INFO("The map had same state as %d %s", p_state_->id, AgvStateStr[p_state_->id].c_str());
    }
    ++get_line_count;
  }
  infile.close();
  cout << "get total " << get_line_count << " line from file, and add " << add_refline_count << "in ref map." << endl;
}

void RvizReferenceLineClass::ss2RefLine(std::stringstream &line_ss, RefLine &out_ref_line)
{
  char ss_c_temp;
  line_ss >> out_ref_line.ref_line_id;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.speed_min;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.speed_max;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.high_max;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.cuv_min;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.line_count;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.gradient;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.line_direction;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.total_length;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_begin.x;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_begin.y;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_begin.z;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_end.x;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_end.y;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.ref_end.z;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.material;
  line_ss >> ss_c_temp;
  line_ss >> out_ref_line.line_area_value;
  line_ss >> ss_c_temp;
  // cout << "LINE:"
  //      << " id:" << out_ref_line.ref_line_id << " smin:" << out_ref_line.speed_min
  //      << " smax:" << out_ref_line.speed_max << " lc:" << out_ref_line.line_count
  //      << " ld:" << out_ref_line.line_direction << " gd:" << out_ref_line.gradient
  //      << " tl:" << out_ref_line.total_length << " bx:" << out_ref_line.ref_begin.x
  //      << " by:" << out_ref_line.ref_begin.y << " bz:" << out_ref_line.ref_begin.z << " mal:" <<
  //      out_ref_line.material
  //      << " lav:" << out_ref_line.line_area_value << endl;
}
void RvizReferenceLineClass::getRefPointFromFile2Map(std::__cxx11::string &file_name,
                                                     std::map< int, RefLine > &ref_line_map,
                                                     std::map< int, std::vector< RefPoints > > &ref_points_map,
                                                     std::vector< RefPoints > &ref_points_vec)
{
  ifstream infile;
  infile.open(file_name); //将文件流对象与文件连接起来
  if (!infile)
  {
    cout << "open file fail!" << endl;
  }
  std::__cxx11::string str;
  std::stringstream ss;
  std::vector< RefPoints > ref_point_vec_temp;
  int ref_line_key   = 0;
  int get_line_count = 0;
  while (getline(infile, str))
  {
    RefPoints ref_point_temp;
    ss.clear();
    ss << str.c_str();
    ss2RefPoint(ss, ref_point_temp);
    ref_point_temp.ref_point_id = get_line_count;
    if (ref_line_key != ref_point_temp.ref_line_id)
    {
      std::map< int, RefLine >::iterator it = ref_line_map.find(ref_point_temp.ref_line_id);
      if (it == ref_line_map.end())
      {
        // SUPERG_WARN << "This ref_points ref line id " << ref_point_temp.ref_line_id << " can not find duing
        // RefLineVec";

        ref_point_vec_temp.clear();
        // break;
      }
      else
      {
        if (ref_line_key != 0)
        {
          // SUPERG_WARN << "This ref_points ref line id " << ref_line_key << " is duing RefLineVec, add "
          //             << ref_point_vec_temp.size() << "point.";
          ref_points_map.insert(std::pair< int, std::vector< RefPoints > >(ref_line_key, ref_point_vec_temp));
        }
        ref_point_vec_temp.clear();
        ref_line_key = ref_point_temp.ref_line_id;
      }
    }
    ref_point_vec_temp.push_back(ref_point_temp);
    ref_points_vec.push_back(ref_point_temp);
    ++get_line_count;
  }
  if (!ref_point_vec_temp.empty())
  {
    // SUPERG_WARN << "This ref_points ref line id " << ref_line_key << " is duing RefLineVec, add "
    //             << ref_point_vec_temp.size() << "point.";
    ref_points_map.insert(std::pair< int, std::vector< RefPoints > >(ref_line_key, ref_point_vec_temp));
    ref_point_vec_temp.clear();
  }
  infile.close();
  //  SUPERG_WARN << "This ref_points map size is " << ref_points_map.size();
  // SUPERG_WARN << "This ref_points vec size is " << ref_points_vec.size();
}

void RvizReferenceLineClass::ss2RefPoint(std::stringstream &point_ss, RefPoints &out_ref_point)
{
  char ss_c_temp;
  point_ss >> out_ref_point.ref_line_id;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.point.x;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.point.y;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.point.z;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.line_count;
  point_ss >> ss_c_temp;
  for (size_t i = 0; i < out_ref_point.line_count; i++)
  {
    LineWidth line_width_temp;
    point_ss >> line_width_temp.left;
    point_ss >> ss_c_temp;
    point_ss >> line_width_temp.right;
    point_ss >> ss_c_temp;
    out_ref_point.line_width.push_back(line_width_temp);
  }
  point_ss >> out_ref_point.kappa;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.dappa;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.d_from_line_begin;
  point_ss >> ss_c_temp;
  point_ss >> out_ref_point.theta;
  point_ss >> ss_c_temp;
}
//构建 消息发布者
void RvizReferenceLineClass::initializePublishers(string topic_name)
{
  ROS_INFO("RvizReferenceLineClass Initializing Publishers");
  std::stringstream ss_topic_name1, ss_topic_name2;
  ss_topic_name1 << topic_name << "_point";
  ss_topic_name2 << topic_name << "_title";
  reference_point_pub_ = nh_.advertise< visualization_msgs::MarkerArray >(ss_topic_name1.str(), 10);

  reference_title_pub_ = nh_.advertise< visualization_msgs::MarkerArray >(ss_topic_name2.str(), 10);
}

int RvizReferenceLineClass::split(char dst[][80], char *str, const char *spl)
{
  int n        = 0;
  char *result = NULL;
  result       = strtok(str, spl);
  while (result != NULL)
  {
    strcpy(dst[n++], result);
    result = strtok(NULL, spl);
  }
  return n;
}

void RvizReferenceLineClass::referenceLinePub(string point_color)
{
  if (!isDataOk_)
  {
    return;
  }
  // SUPERG_WARN << "RvizReferenceLineClass:referenceLinePub";

  visualization_msgs::MarkerArray markerArray_point;
  visualization_msgs::Marker points;
  points.header.frame_id    = "/odom";
  points.header.stamp       = ros::Time::now();
  points.ns                 = "refline_point";
  points.action             = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id                 = 0;
  points.type               = visualization_msgs::Marker::POINTS;
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.3;
  points.scale.y = 0.3;

  std::vector< std::string > v;
  SplitString(point_color, v, ",");
  // Points color
  points.color.r = atof(v[0].c_str());
  points.color.g = atof(v[1].c_str());
  points.color.b = atof(v[2].c_str());
  points.color.a = 1.0;

  visualization_msgs::MarkerArray markerArray_title;

  // std::map< int, std::vector< RefPoints > > ref_points_map
  std::map< int, std::vector< RefPoints > >::iterator ref_iter;
  for (ref_iter = ref_points_map.begin(); ref_iter != ref_points_map.end(); ref_iter++)
  {
    // SUPERG_WARN << "ref_iter->first = " << ref_iter->first;
    visualization_msgs::Marker title_start, title_end;

    title_start.header.frame_id = title_end.header.frame_id = "/odom";
    title_start.header.stamp = title_end.header.stamp = ros::Time::now();

    std::stringstream ss;

    title_start.ns = "refline_title_start";
    title_end.ns   = "refline_title_end";

    title_start.action = title_end.action = visualization_msgs::Marker::ADD;
    title_start.pose.orientation.w = title_end.pose.orientation.w = 1.0;

    title_start.id = ref_iter->first;
    title_end.id   = ref_iter->first;

    title_start.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    title_end.type   = visualization_msgs::Marker::TEXT_VIEW_FACING;

    title_start.scale.z = 3;

    title_start.color.r = 1.0;
    title_start.color.g = 0.0;
    title_start.color.b = 0.0;
    title_start.color.a = 1.0;

    title_end.scale.z = 3;

    title_end.color.r = 0.0;
    title_end.color.g = 0.0;
    title_end.color.b = 1.0;
    title_end.color.a = 1.0;

    // SUPERG_WARN << "second.size() = " << ref_iter->second.size();
    for (size_t i = 0; i < ref_iter->second.size(); i++)
    {

      geometry_msgs::Point p;
      p.x = ref_iter->second[i].point.x * GRID_SIZE + OFFSET_X;
      p.y = ref_iter->second[i].point.y * GRID_SIZE + OFFSET_Y;
      p.z = 0;

      // SUPERG_WARN << "p.x = " << p.x << " p.y = " << p.y;
      // ref_points.points.emplace_back(p);

      if (lane_id_ == 0 || lane_id_ == ref_iter->first)
      {
        points.points.emplace_back(p);

        if (i == 0)
        {
          ss << "Route " << ref_iter->first << " start";
          title_start.text = ss.str();
          ss.str("");
          title_start.pose.position.x = ref_iter->second[i].point.x * GRID_SIZE + OFFSET_X + 1;
          title_start.pose.position.y = ref_iter->second[i].point.y * GRID_SIZE + OFFSET_Y;
          title_start.pose.position.z = 0;
          markerArray_title.markers.emplace_back(title_start);
        }
        // i = i + 4;
        if (i == ref_iter->second.size() - 1)
        {
          ss << "Route " << ref_iter->first << " end";
          title_end.text = ss.str();
          ss.str("");
          title_end.pose.position.x = ref_iter->second[i - 4].point.x * GRID_SIZE + OFFSET_X + 1;
          title_end.pose.position.y = ref_iter->second[i - 4].point.y * GRID_SIZE + OFFSET_Y;
          title_end.pose.position.z = 3;
          markerArray_title.markers.emplace_back(title_end);
        }
      }
    }
  }
  markerArray_point.markers.emplace_back(points);

  reference_point_pub_.publish(markerArray_point);
  reference_title_pub_.publish(markerArray_title);
}

} // namespace superg_agv
} // namespace monitor