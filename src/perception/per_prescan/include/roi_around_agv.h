#ifndef ROI_AROUND_AGV_H_
#define ROI_AROUND_AGV_H_

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <stdio.h>

using namespace cv;
namespace superg_agv
{
namespace roi_agv
{

enum MaxROISize
{
  ROI_HEIGHT = 220,
  ROI_WIDTH  = 30
};

struct ROIRow
{
  uint64_t left;
  uint64_t right;
};

struct ScaleStr
{
  double height;
  double width;
};

struct RoiPoint
{
  int y; //负数为agv中心点后 正数为agv中心点前
  int x; //负数为agv中心点左 正数为agv中心点右
};

struct MapStruct
{
  cv::Mat map;
  std::vector< vector< Point > > contour;
  double scale;
  double extra_map_boundry;
  Point base_point;
  Point max_point;
  Point min_point;
  Point centre_point;
  Point total_size;
  int rotation_tip;
};

class RoiMap
{
public:
  void setCentrePoint(Point &p_)
  {
    centre_point.x = p_.x;
    centre_point.y = p_.y;
  }

  void setBasePoint(Point &p_)
  {
    base_point.x = p_.x;
    base_point.y = p_.y;
  }

  void setMinPoint(Point &p_)
  {
    min_point.x = p_.x;
    min_point.y = p_.y;
  }

  void setMaxPoint(Point &p_)
  {
    max_point.x = p_.x;
    max_point.y = p_.y;
  }

  void setMapScale(double &scale_)
  {
    scale = scale_;
  }

  void setRotationTip(int &rotation_tip_)
  {
    rotation_tip = rotation_tip_;
  }

  void setExtraMapBoundry(double &extra_map_boundry_)
  {
    extra_map_boundry = extra_map_boundry_;
  }

  void setTotalPoint(Point &p_)
  {
    total_size.x = p_.x * 2 + 1;
    total_size.y = p_.y * 2 + 1;
  }

  Point getCentrePoint()
  {
    return centre_point;
  }

  Point getMinPoint()
  {
    return min_point;
  }

  Point getMaxPoint()
  {
    return max_point;
  }

  double getMapScale()
  {
    return scale;
  }

  double getExtraMapBoundry()
  {
    return extra_map_boundry;
  }

  int getRotationTip()
  {
    return rotation_tip;
  }

  void initZerosMap()
  {
    map = Mat::zeros(cvSize(total_size.x, total_size.y), CV_8UC1);
  }

  void initOnesMap()
  {
    map = Mat::ones(cvSize(total_size.x, total_size.y), CV_8UC1);
  }

public:
  cv::Mat map;
  std::vector< vector< Point > > contour;
  double scale;
  double extra_map_boundry;
  Point base_point;
  Point max_point;
  Point min_point;
  Point centre_point;
  Point total_size;
  int rotation_tip;
};

class BuildRoiSpace
{
public:
  BuildRoiSpace(ros::NodeHandle nh_)
  {
    ns                = nh_;
    total_width       = 100;
    total_height      = 100;
    max_width         = 0;
    min_width         = 0;
    max_height        = 0;
    min_height        = 0;
    extra_map_boundry = 50;
    image_transport::ImageTransport it(ns);
    pic_pub_ = it.advertise("camera/image", 1);
    // sendPic();
  }
  virtual ~BuildRoiSpace()
  {
  }

  void setMapScale(double scale_temp)
  {
    map_scale = scale_temp;
  }

  void setMaxAndMinWidthHeight(double max_width_x, double min_width_x, double max_width_y, double min_width_y)
  {
    max_width  = getIntFromDouble((max_width_x + extra_map_boundry) * map_scale);
    min_width  = getIntFromDouble((min_width_x - extra_map_boundry) * map_scale);
    max_height = getIntFromDouble((max_width_y + extra_map_boundry) * map_scale);
    min_height = getIntFromDouble((min_width_y - extra_map_boundry) * map_scale);
  }

  void showMaxMinWidthHeight()
  {
    std::cout << "max_width  is: " << max_width << std::endl;
    std::cout << "min_width  is: " << min_width << std::endl;
    std::cout << "max_height is: " << max_height << std::endl;
    std::cout << "min_height is: " << min_height << std::endl;
  }

  void setMapSize()
  {
    total_width  = max_width - min_width;
    total_height = max_height - min_height;
  }

  Point getMaxXY()
  {
    Point p_temp;
    p_temp.x = total_width;
    p_temp.y = total_height;
    return p_temp;
  }

  Point getBaseXY()
  {
    Point p_temp;
    p_temp.x = min_width;
    p_temp.y = min_height;
    return p_temp;
  }

  void initMap()
  {
    ROS_INFO("map size w:%d h:%d", total_width, total_height);
    // base_map = Mat::ones(cvSize(total_width, total_height), CV_8UC1, Scalar::all(1));
    initBaseMap();
    initMaskMap();
    initLaneMap();
    initObsMap();
  }

  void initBaseMap()
  {
    base_map = Mat::ones(cvSize(total_width, total_height), CV_8UC1);
  }

  void initMaskMap()
  {
    mask_map = Mat::zeros(base_map.size(), CV_8UC1);
  }

  void initObsMap()
  {
    obs_map = Mat::zeros(base_map.size(), CV_8UC1);
  }

  void initLaneMap()
  {
    lane_map = Mat::zeros(base_map.size(), CV_8UC1);
  }

  int setContours(vector< Point > &pts, int c_index) //顺序添加...
  {
    // ROS_INFO("setContours %d", c_index);
    switch (c_index)
    {
    case 0:
      if (pts.size() > 2)
      {
        lane_contour.push_back(pts);
        useContoursMask(mask_map, c_index);
        return 1;
      }
      else
      {
        return -1;
      }
      break;
    case 1:
      if (pts.size() > 2)
      {
        agv_contour.push_back(pts);
        useContoursMask(mask_map, c_index);
        return 1;
      }
      else
      {
        return -1;
      }
      break;
    case 2:
      if (pts.size() > 2)
      {
        agv_contour.push_back(pts);
        useContoursMask(mask_map, c_index);
        return 1;
      }
      else
      {
        return -1;
      }
      break;
    case 3:
      if (pts.size() > 2)
      {
        contour.push_back(pts);
        useObsMask(c_index);
        // cv::Mat obs_temp = Mat::zeros(base_map.size(), CV_8UC1);
        // useContoursMask(obs_map, c_index, (contour.size() - 1));
        // obs_map = obs_map + obs_temp;
        return 1;
      }
      else
      {
        return -1;
      }
      break;
    default:
      return -1;
    }
  }

  void useObsMask(int c_index)
  {
    useContoursMask(obs_map, c_index);
  }

  void clearLaneContour()
  {
    lane_contour.clear();
  }

  void clearAgvContour()
  {
    agv_contour.clear();
  }

  void clearContour()
  {
    contour.clear();
  }

  void useLaneContoursMask(cv::Mat &mask_)
  {
    drawContours(mask_, lane_contour, -1, Scalar::all(255), -1);
    // drawContours(mask_, contour, -1, Scalar::all(255), -1);
  }

  void useContoursMask(cv::Mat &mask_, int c_index)
  {
    switch (c_index)
    {
    case 0:
      drawContours(mask_, lane_contour, -1, Scalar::all(255), -1);
      // drawContours(mask_, lane_contour, -1, Scalar::all(255), 1);
      break;
    case 1:
      drawContours(mask_, agv_contour, -1, Scalar::all(255), -1);
      break;
    case 2:
      drawContours(mask_, agv_contour, -1, Scalar::all(255), -1);
      break;
    case 3:
      drawContours(mask_, contour, -1, Scalar::all(255), 1);
      break;
    default:
      break;
    }
  }

  void useContoursMask(cv::Mat &mask_, int c_index, int c_class)
  {
    switch (c_index)
    {
    case 0:
      drawContours(mask_, lane_contour, c_class, Scalar::all(255), -1);
      break;
    case 1:
      drawContours(mask_, agv_contour, c_class, Scalar::all(255), -1);
      break;
    case 2:
      drawContours(mask_, agv_contour, c_class, Scalar::all(255), -1);
      break;
    case 3:
      drawContours(mask_, contour, c_class, Scalar::all(255), 1);
      break;
    default:
      break;
    }
  }

  int getIntFromDouble(double double_temp)
  {
    if (double_temp < 0)
    {
      return floor(double_temp);
    }
    else
    {
      return ceil(double_temp);
    }
  }

  void refreshROIMapFromLane()
  {
    base_map.copyTo(roi_map, lane_map);
  }

  void refreshROIMap()
  {
    base_map.copyTo(roi_map, mask_map);
  }

  void showAllMap(int show_index)
  {
    switch (show_index)
    {
    case 0:
      imshow("mask_map", mask_map);
      imshow("lane_map", lane_map);
      imshow("roi_map", roi_map);
      imshow("obs_map", obs_map);
      break;
    case 1:
      imshow("mask_map", mask_map);
      break;
    case 2:
      imshow("base_map", base_map);
      break;
    case 3:
      imshow("roi_map", roi_map);
      break;
    case 4:
      imshow("obs_map", obs_map);
      break;
    default:
      break;
    }
    waitKey(1);
  }

  void coutROIMap()
  {
    std::cout << "base_map is: \n" << base_map << std::endl;
    std::cout << "mask_map is: \n" << mask_map << std::endl;
    std::cout << "roi_map is: \n" << roi_map << std::endl;
  }

  void mathObsAndROI()
  {
    roi_map = roi_map.mul(obs_map);
  }

  void sendPic()
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mask_map).toImageMsg(); //图像格式转换
    pic_pub_.publish(msg);
  }

public:
  std::vector< ROIRow > roi_vec;
  double map_scale;
  double extra_map_boundry;
  int max_width;
  int min_width;
  int max_height;
  int min_height;
  int total_width;
  int total_height;
  ScaleStr roi_scale;
  cv::Mat base_map;
  cv::Mat mask_map;
  cv::Mat lane_map;
  cv::Mat roi_map;
  cv::Mat obs_map;
  std::vector< vector< Point > > contour;
  std::vector< vector< Point > > lane_contour;
  std::vector< vector< Point > > agv_contour;

  ros::NodeHandle ns;
  image_transport::Publisher pic_pub_;
};
}
} // roi_agv

#endif