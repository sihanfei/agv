#ifndef MAP_INCLUDE_GET_REF_LINE_H_
#define MAP_INCLUDE_GET_REF_LINE_H_

#include "ros/ros.h"

#include "monitor_common_utils.h"
#include <fstream>
#include <iostream>
#include <sstream>

#include <algorithm>
#include <map>
#include <math.h>
#include <string>
#include <vector>

using namespace std;

namespace superg_agv
{
namespace monitor
{
class GetRefLine
{
public:
  GetRefLine()
  {
  }
  virtual ~GetRefLine()
  {
  }

  void setRefLineInit()
  {
    // if (!ref_line_map.empty())
    // {
    //   ref_line_map.clear();
    // }
    // if (!ref_points_map.empty())
    // {
    //   ref_points_map.clear();
    // }
    // if (!appendix_attribute_map.empty())
    // {
    //   appendix_attribute_map.clear();
    // }
    // if (!ref_points_vec.empty())
    // {
    //   ref_points_vec.clear();
    // }
  }

  void getRefLineFromFile2Map(std::__cxx11::string &file_name, std::map< int, RefLine > &ref_line_map)
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
        ROS_DEBUG_STREAM("Add " << get_line_count << " line " << ref_line_temp.ref_line_id
                                << " in ref line map, now map size is " << ref_line_map.size());
        // ROS_INFO("add %d %s state in map", p_state_->id, AgvStateStr[p_state_->id].c_str());
      }
      else
      {
        ROS_DEBUG_STREAM("This " << get_line_count << " line " << ref_line_temp.ref_line_id
                                 << " is during in ref line map, now map size is " << ref_line_map.size());
        // ROS_INFO("The map had same state as %d %s", p_state_->id, AgvStateStr[p_state_->id].c_str());
      }
      ++get_line_count;
    }
    infile.close();
    ROS_DEBUG_STREAM("get total " << get_line_count << " line from file, and add " << add_refline_count
                                  << "in ref map.");
  }

  void ss2RefLine(std::stringstream &line_ss, RefLine &out_ref_line)
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

  void getAppendixFromFile2RefLine(std::__cxx11::string &file_name, std::map< int, RefLine > &ref_line_map)
  {
    ifstream infile;
    infile.open(file_name); //将文件流对象与文件连接起来
    if (!infile)
    {
      cout << "open file fail!" << endl;
    }
    std::__cxx11::string str;
    std::stringstream ss;
    while (getline(infile, str))
    {
      int ref_line_id_temp;
      ss.clear();
      ss << str.c_str();
      ss >> ref_line_id_temp;
      char ss_c_temp;
      ss >> ss_c_temp;
      std::map< int, RefLine >::iterator it = ref_line_map.find(ref_line_id_temp);
      if (it == ref_line_map.end())
      {
        cout << "This appendix_id ref line id " << ref_line_id_temp << " can not find duing RefLineVec" << endl;
        //       break;
      }
      else
      {
        cout << "This appendix_id ref line id " << ref_line_id_temp << " is duing RefLineVec" << endl;
        int appendix_id_temp;
        while (ss >> appendix_id_temp)
        {
          it->second.appendix_id.push_back(appendix_id_temp);
          ss >> ss_c_temp;
        }
      }
    }
  }

  void getRefPointFromFile2Map(std::__cxx11::string &file_name, std::map< int, RefLine > &ref_line_map,
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
          ROS_DEBUG_STREAM("This ref_points ref line id " << ref_point_temp.ref_line_id
                                                          << " can not find duing RefLineVec");

          ref_point_vec_temp.clear();
          // break;
        }
        else
        {
          if (ref_line_key != 0)
          {
            ROS_DEBUG_STREAM("This ref_points ref line id " << ref_line_key << " is duing RefLineVec, add "
                                                            << ref_point_vec_temp.size() << "point.");
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
      ROS_DEBUG_STREAM("This ref_points ref line id " << ref_line_key << " is duing RefLineVec, add "
                                                      << ref_point_vec_temp.size() << "point.");
      ref_points_map.insert(std::pair< int, std::vector< RefPoints > >(ref_line_key, ref_point_vec_temp));
      ref_point_vec_temp.clear();
    }
    infile.close();
    ROS_DEBUG_STREAM("This ref_points map size is " << ref_points_map.size());
    ROS_DEBUG_STREAM("This ref_points vec size is " << ref_points_vec.size());
  }

  double changeAngle(double theta_, double kappa_, double dappa_)
  {
    //镇江港地图错误补丁0822
    double angle_temp_ = theta_;
    //转弯角度判断 针对0821s镇江港地图修订
    if (kappa_ == 0 && dappa_ == 0)
    {
      angle_temp_ = angle_temp_;
    }
    else
    {
      angle_temp_ = 2 * M_PI - angle_temp_;
      if (angle_temp_ < 0)
      {
        angle_temp_ = angle_temp_ + 2 * M_PI;
      }
    }
    return angle_temp_;
  }

  void ss2RefPoint(std::stringstream &point_ss, RefPoints &out_ref_point)
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
    //镇江港地图错误补丁0822 ,修订后注释掉
    // out_ref_point.theta = changeAngle(out_ref_point.theta, out_ref_point.kappa, out_ref_point.dappa);
    point_ss >> ss_c_temp;
  }

  void getAppendixAttributeFromFile2Map(std::__cxx11::string &file_name,
                                        std::map< int, AppendixAttribute > &appendix_attribute_map)
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
      AppendixAttribute appendix_attribute_temp;
      ss.clear();
      ss << str.c_str();
      ss2AppendixAttribute(ss, appendix_attribute_temp);
      //查找重复
      std::map< int, AppendixAttribute >::iterator it =
          appendix_attribute_map.find(appendix_attribute_temp.appendix_id);
      if (it == appendix_attribute_map.end())
      {
        appendix_attribute_map.insert(
            std::pair< int, AppendixAttribute >(appendix_attribute_temp.appendix_id, appendix_attribute_temp));
        cout << "Add appendix attribute ID " << appendix_attribute_temp.appendix_id << " into AA map" << endl;
      }
      else
      {
        cout << "appendix attribute ID " << appendix_attribute_temp.appendix_id << " is already during map" << endl;
      }
    }
    infile.close();
  }

  void ss2AppendixAttribute(std::stringstream &aa_ss, AppendixAttribute &out_appendix_attribute)
  {
    char ss_c_temp;
    aa_ss >> out_appendix_attribute.appendix_id;
    aa_ss >> ss_c_temp;
    aa_ss >> out_appendix_attribute.appendix_type;
    aa_ss >> ss_c_temp;
    aa_ss >> out_appendix_attribute.corner_count;
    aa_ss >> ss_c_temp;
    for (size_t i = 0; i < out_appendix_attribute.corner_count; i++)
    {
      Point3 corner_point_temp;
      aa_ss >> corner_point_temp.x;
      aa_ss >> ss_c_temp;
      aa_ss >> corner_point_temp.y;
      aa_ss >> ss_c_temp;
      aa_ss >> corner_point_temp.z;
      aa_ss >> ss_c_temp;
      out_appendix_attribute.corner_point.push_back(corner_point_temp);
    }
  }

  void showALLRefPointsFromMap(std::map< int, std::vector< RefPoints > > &ref_points_map)
  {
    int loop_index = 0;
    for (std::map< int, std::vector< RefPoints > >::iterator it_loop_first = ref_points_map.begin();
         it_loop_first != ref_points_map.end(); it_loop_first++)
    {
      for (std::vector< RefPoints >::iterator it_loop_second = it_loop_first->second.begin();
           it_loop_second != it_loop_first->second.end(); it_loop_second++)
      {
        cout << "The " << loop_index << " at line " << it_loop_first->first << " point id："
             << it_loop_second->ref_point_id << " (" << it_loop_second->point.x << ", " << it_loop_second->point.y
             << ", " << it_loop_second->point.z << " ) heading" << it_loop_second->theta * 180 / M_PI << endl;
        ++loop_index;
      }
    }
  }

  void showALLRefPoints(std::vector< RefPoints > &ref_points_vec)
  {
    int loop_index = 0;
    for (std::vector< RefPoints >::iterator it_loop = ref_points_vec.begin(); it_loop != ref_points_vec.end();
         it_loop++)
    {
      cout << "The " << loop_index << " at line " << it_loop->ref_line_id << " (" << it_loop->point.x << ", "
           << it_loop->point.y << ", " << it_loop->point.z << " )" << endl;
      ++loop_index;
    }
  }

  int findXYZFromRefPointVecTriangle(const RefPoints point_in, std::map< int, RefLine > &ref_line_map_caches,
                                     std::map< int, std::vector< RefPoints > > &ref_points_map_caches, int mode)
  {
    // if (mode == 1)
    // {
    //   cout << "Use min side length dif" << endl;
    // }
    // else
    // {
    //   cout << "Use min hight" << endl;
    // }

    int ref_line_id_temp = 0;
    std::map< int, RefLine >::iterator it_refline;
    std::map< double, int > line_distance_map;
    std::map< double, int > curve_distance_map;
    int loop_index;
    for (it_refline = ref_line_map_caches.begin(); it_refline != ref_line_map_caches.end(); it_refline++)
    {
      std::map< int, std::vector< RefPoints > >::iterator it_refpoint = ref_points_map_caches.find(it_refline->first);
      if (it_refpoint == ref_points_map_caches.end())
      {
      }
      else
      {
        //判断是否弧线或者直线
        int ref_line_point_size_ = static_cast< int >(ref_points_map_caches.size());
        int judge_index          = max(2, ref_line_point_size_ / 2);
        double l1                = sqrt(mathPointDistanceSquare(point_in.point, it_refline->second.ref_begin));
        double l2                = sqrt(mathPointDistanceSquare(point_in.point, it_refline->second.ref_end));
        double l3     = sqrt(mathPointDistanceSquare(it_refline->second.ref_begin, it_refline->second.ref_end));
        double p      = (l1 + l2 + l3) / 2;
        double hieght = sqrt(p * (p - l1) * (p - l2) * (p - l3)) / l3;
        double dl     = l1 + l2 - l3;
        if (it_refpoint->second.at(judge_index).kappa == 0 && it_refpoint->second.at(judge_index).dappa == 0)
        {
          //直线
          double line_width = maxWidthFromRefPoint(it_refpoint->second.at(judge_index));
          if (hieght < line_width)
          {
            if (mode == 1)
            {
              line_distance_map.insert(std::pair< double, int >(dl, it_refline->first));
            }
            else
            {

              line_distance_map.insert(std::pair< double, int >(hieght, it_refline->first));
            }
          }
        }
        else
        {
          //曲线 遍历
          if (mode == 1)
          {
            curve_distance_map.insert(std::pair< double, int >(dl, it_refline->first));
          }
          else
          {
            curve_distance_map.insert(std::pair< double, int >(hieght, it_refline->first));
          }
        }
      }
      ++loop_index;
    }

    if (!line_distance_map.empty()) //结果不为空
    {
      ref_line_id_temp = line_distance_map.begin()->second;
      ROS_DEBUG_STREAM("The point is during " << ref_line_id_temp << " ref line distance is "
                                              << line_distance_map.begin()->first);
    }
    else if (!curve_distance_map.empty())
    {
      ref_line_id_temp = minDistanceFromCurveDistanceMap(point_in, curve_distance_map, ref_points_map_caches);
      ROS_DEBUG_STREAM("The point is during " << ref_line_id_temp << " ref curve");
    }
    else
    {
      ref_line_id_temp = 0;
    }
    return ref_line_id_temp;
  }

  int minDistanceFromCurveDistanceMap(const RefPoints point_in, std::map< double, int > &cdm_,
                                      std::map< int, std::vector< RefPoints > > &ref_points_map_caches)
  {
    double distance_ = 100;
    struct DistanceRefPoint distance_ref_point;
    distance_ref_point.ref_line_id = 0;
    std::map< double, int >::iterator it_cdm;
    for (it_cdm = cdm_.begin(); it_cdm != cdm_.end(); it_cdm++)
    {
      std::map< int, std::vector< RefPoints > >::iterator it_refpoint = ref_points_map_caches.find(it_cdm->second);
      DistanceRefPoint distance_ref_point_temp;
      double distance_temp =
          minDistanceFromRefPointVec(distance_, point_in, it_refpoint->second, distance_ref_point_temp);
      if (distance_ > distance_temp)
      {
        distance_          = distance_temp;
        distance_ref_point = distance_ref_point_temp;
      }
    }
    return distance_ref_point.ref_line_id;
  }

  double minDistanceFromRefPointVec(const double distance_in, const RefPoints point_in, std::vector< RefPoints > &rpv_,
                                    DistanceRefPoint &ret)
  {
    double distance_ = distance_in;
    std::vector< RefPoints >::iterator it_rpv;
    for (it_rpv = rpv_.begin(); it_rpv != rpv_.end(); it_rpv++)
    {
      double line_width    = maxWidthFromRefPoint(*it_rpv);
      double distance_temp = mathPointDistanceSquare(point_in.point, it_rpv->point);
      if (distance_temp < line_width)
      {
        if (distance_ > distance_temp)
        {
          distance_        = distance_temp;
          ret.ref_line_id  = it_rpv->ref_line_id;
          ret.ref_point    = *it_rpv;
          ret.min_distance = distance_;
        }
      }
    }
    return distance_;
  }

  double maxWidthFromRefPoint(const RefPoints point_)
  {
    double width_ = 0.0;
    for (size_t i = 0; i < point_.line_count; i++)
    {
      double max_width = max(abs(point_.line_width.at(i).left), abs(point_.line_width.at(i).right));
      if (width_ < max_width)
      {
        width_ = max_width;
      }
    }
    return width_;
  }

  int findLineFromRefPointVecWithXYZHeading(RefPoints &point_in, std::vector< RefPoints > &ref_points_vec_caches)
  {
    int ref_line_id_temp = 0;
    std::vector< RefPoints > find_points_caches;
    std::map< double, RefPoints > find_points_map_caches;
    std::map< double, FindPoint > find_points_plus_map_caches;
    std::map< double, FindPoint > find_radian_point_caches;
    std::vector< RefPoints >::iterator it_b    = ref_points_vec_caches.begin();
    std::vector< RefPoints >::iterator it_e    = ref_points_vec_caches.end();
    std::vector< RefPoints >::iterator it_loop = it_b;

    double max_distance = 1000.0;

    // cout << "Will go to find (" << point_in.point.x << ", " << point_in.point.y << ", " << point_in.point.z << " )"
    //      << endl;
    int loop_index = 0;
    // sleep(1);
    while (ros::ok())
    {
      it_b       = it_loop;
      it_loop    = std::find_if(it_b, it_e, PointFindFun(point_in));
      loop_index = it_loop - ref_points_vec_caches.begin();
      if (it_loop == it_e)
      {
        // cout << "We had find at the END" << endl;
        break;
      }
      else
      {
        FindPoint find_point_temp;
        find_point_temp.distance = mathPointDistanceSquare(point_in.point, it_loop->point);
        if (find_point_temp.distance < 0.0001)
        {
          find_point_temp.distance = 0.0001;
        }

        // int relative_space_position = isPointOnLineLeft(point_in.theta, it_loop->theta);
        // if (radian_diff < M_PI / 3)
        // {
        if (find_point_temp.distance < max_distance)
        {
          find_point_temp.radian_diff = mathPointRadianDifference(point_in.theta, it_loop->theta);
          find_point_temp.ref_point   = *it_loop;
          if (find_point_temp.radian_diff < 0.0001)
          {
            find_point_temp.radian_diff = 0.0001;
          }
          find_points_plus_map_caches.insert(std::pair< double, FindPoint >(find_point_temp.distance, find_point_temp));
        }
        // find_points_map_caches.insert(std::pair< double, RefPoints >(find_point_temp.distance, *it_loop));
        // find_points_caches.push_back(*it_loop);
        // cout << "We had find " << loop_index << " answer: (" << it_loop->point.x << ", " << it_loop->point.y << ", "
        //      << it_loop->point.z << " ) during " << it_loop->ref_line_id << endl;
        // cout << "On line " << find_point_temp.ref_point.ref_line_id << " Target angele: " << point_in.theta << " "
        //      << point_in.theta * 180 / M_PI << " Find angele: " << find_point_temp.ref_point.theta << " "
        //      << find_point_temp.ref_point.theta * 180 / M_PI << " diffradian: " << find_point_temp.radian_diff << " "
        //      << find_point_temp.radian_diff * 180 / M_PI << " distance:" << find_point_temp.distance << endl;

        // cout << "Find vec caches had " << find_points_caches.size() << " Points" << endl;
        // cout << "Find map caches had " << find_points_map_caches.size() << " Points" << endl;
        // }

        ++it_loop;
        if (it_loop == it_e)
        {
          // cout << "We had find at the END" << endl;
          break;
        }
      }
    }

    if (!find_points_plus_map_caches.empty()) //结果不为空
    {
      std::map< double, FindPoint >::iterator it_loop_min;
      for (it_loop_min = find_points_plus_map_caches.begin(); it_loop_min != find_points_plus_map_caches.end();
           it_loop_min++)
      {
        // double judge_par = 1 / ((1 / (it_loop_min->second.distance-max_distance ) )+
        // (it_loop_min->second.radian_diff));
        double judge_par = ((it_loop_min->second.distance) / 4) + (it_loop_min->second.radian_diff * 4 / M_PI);
        // find_radian_point_caches.insert(
        //     std::pair< double, FindPoint >(it_loop_min->second.radian_diff, it_loop_min->second));
        find_radian_point_caches.insert(std::pair< double, FindPoint >(judge_par, it_loop_min->second));
      }

      // for (it_loop_min = find_radian_point_caches.begin(); it_loop_min != find_radian_point_caches.end();
      // it_loop_min++)
      // {
      //   ROS_INFO("laneID %d (%lf,%lf) dis:%lf ang:%lf judge_par:%lf", it_loop_min->second.ref_point.ref_line_id,
      //            it_loop_min->second.ref_point.point.x, it_loop_min->second.ref_point.point.y,
      //            it_loop_min->second.distance, it_loop_min->second.radian_diff, it_loop_min->first);
      // }

      ref_line_id_temp = find_radian_point_caches.begin()->second.ref_point.ref_line_id;
      point_in         = find_radian_point_caches.begin()->second.ref_point;

      // cout << "The most closest is (" << find_radian_point_caches.begin()->second.ref_point.point.x << " , "
      //      << find_radian_point_caches.begin()->second.ref_point.point.y << " , "
      //      << find_radian_point_caches.begin()->second.ref_point.point.z
      //      << ") ref_l_heading is: " << find_radian_point_caches.begin()->second.ref_point.theta
      //      << " distance is: " << find_radian_point_caches.begin()->second.distance
      //      << " diff radian is: " << find_radian_point_caches.begin()->second.radian_diff
      //      << " ref_l_id is: " << ref_line_id_temp << " size: " << find_points_plus_map_caches.size() << endl;
    }
    else //结果为空
    {
      ref_line_id_temp = 0;
    }

    return ref_line_id_temp;
  }

  int findXYZFromRefPointVecTraversal(RefPoints &point_in, std::vector< RefPoints > &ref_points_vec_caches)
  {
    int ref_line_id_temp = 0;
    std::vector< RefPoints > find_points_caches;
    std::map< double, RefPoints > find_points_map_caches;
    std::vector< RefPoints >::iterator it_b    = ref_points_vec_caches.begin();
    std::vector< RefPoints >::iterator it_e    = ref_points_vec_caches.end();
    std::vector< RefPoints >::iterator it_loop = it_b;

    // cout << "Will go to find (" << point_in.point.x << ", " << point_in.point.y << ", " << point_in.point.z << " )"
    //      << endl;
    int loop_index = 0;
    // sleep(1);
    while (ros::ok())
    {
      it_b       = it_loop;
      it_loop    = std::find_if(it_b, it_e, PointFindFun(point_in));
      loop_index = it_loop - ref_points_vec_caches.begin();
      if (it_loop == it_e)
      {
        // cout << "We had find at the END" << endl;
        break;
      }
      else
      {
        double distance_ = mathPointDistanceSquare(point_in.point, it_loop->point);
        find_points_map_caches.insert(std::pair< double, RefPoints >(distance_, *it_loop));
        find_points_caches.push_back(*it_loop);
        // cout << "We had find " << loop_index << " answer: (" << it_loop->point.x << ", " << it_loop->point.y << ", "
        //      << it_loop->point.z << " ) during " << it_loop->ref_line_id << endl;
        // cout << "Find vec caches had " << find_points_caches.size() << " Points" << endl;
        // cout << "Find map caches had " << find_points_map_caches.size() << " Points" << endl;
        ++it_loop;
        if (it_loop == it_e)
        {
          // cout << "We had find at the END" << endl;
          break;
        }
      }
    }

    if (!find_points_caches.empty()) //结果不为空
    {
      // cout << "We had find " << find_points_caches.size() << " answers" << endl;
    }
    else //结果为空
    {
      // point_out 赋空
    }

    if (!find_points_map_caches.empty()) //结果不为空
    {
      ref_line_id_temp = find_points_map_caches.begin()->second.ref_line_id;
      ROS_DEBUG_STREAM("The most closest is (" << find_points_map_caches.begin()->second.point.x << " , "
                                               << find_points_map_caches.begin()->second.point.y << " , "
                                               << find_points_map_caches.begin()->second.point.z
                                               << ") distance is: " << find_points_map_caches.begin()->first
                                               << " ref line id is: " << ref_line_id_temp);

      point_in = find_points_map_caches.begin()->second;
    }
    else //结果为空
    {
      ref_line_id_temp = 0;
    }

    return ref_line_id_temp;
  }

  double mathPointDistanceSquare(const Point3 p1, const Point3 p2)
  {
    const double dx = p1.x - p2.x;
    const double dy = p1.y - p2.y;
    return dx * dx + dy * dy;
  }

  double mathPointRadianDifference(const double &theta_in, const double &theta_ref)
  {
    double radian_temp = abs(theta_in - theta_ref);
    if (radian_temp < M_PI)
    {
      return radian_temp;
    }
    else
    {
      return (2 * M_PI - radian_temp);
    }
  }

  int isPointOnLineLeft(const double &theta_in, const double &theta_ref)
  {
    double radian_temp = theta_in - theta_ref;
    if (radian_temp < 0)
    {
      radian_temp = radian_temp + 2 * M_PI;
    }

    if (radian_temp == M_PI || radian_temp == 0)
    {
      return 0;
    }
    else if (radian_temp > M_PI)
    {
      return 1;
    }
    else
    {
      return -1;
    }
  }

public:
  struct PointFindFun
  {
    PointFindFun(RefPoints task_point) : aim_point(task_point)
    {
    }
    bool operator()(const std::vector< RefPoints >::value_type &point_caches_)
    {
      double distance_ = (aim_point.point.x - point_caches_.point.x) * (aim_point.point.x - point_caches_.point.x) +
                         (aim_point.point.y - point_caches_.point.y) * (aim_point.point.y - point_caches_.point.y);
      double width_ = 0.0;
      for (size_t i = 0; i < point_caches_.line_count; i++)
      {
        double max_width = max(abs(point_caches_.line_width.at(i).left), abs(point_caches_.line_width.at(i).right));
        if (width_ < max_width)
        {
          width_ = max_width;
        }
      }
      return (distance_ < (width_ * width_));
    }

    RefPoints aim_point;
  };
};

} // namespace map
} // namespace superg_agv
#endif