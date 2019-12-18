#ifndef GET_REF_MAP_H_
#define GET_REF_MAP_H_

#include "ros/ros.h"

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
namespace roi_agv
{

struct AreaRange
{
  double min_x;
  double min_y;
  double max_x;
  double max_y;
};

class GetRefMapAreaRange
{
public:
  void setAreaFilePath(std::string workplace_path, std::string file_name)
  {
    std::string home_path = getenv("HOME");
    area_file_path        = home_path + workplace_path + file_name;
  }

  void getAreaRangeFromFile(AreaRange &area_range)
  {
    ifstream infile;
    infile.open(area_file_path); //将文件流对象与文件连接起来
    if (!infile)
    {
      cout << "open file fail!" << endl;
    }
    std::__cxx11::string str;
    std::stringstream ss;
    int get_line_count = 0;
    while (getline(infile, str))
    {
      ss.clear();
      ss << str.c_str();

      char ss_c_temp;
      if (get_line_count == 0)
      {
        ss >> area_range.min_x;
        ss >> ss_c_temp;
        ss >> area_range.max_x;
      }

      if (get_line_count == 1)
      {
        ss >> area_range.min_y;
        ss >> ss_c_temp;
        ss >> area_range.max_y;
      }

      ++get_line_count;
    }
    infile.close();
  }

public:
  std::string area_file_path;
};
}
}
#endif
