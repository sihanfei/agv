#ifndef _EXCEPTION_MONITOR_H_
#define _EXCEPTION_MONITOR_H_

#include "ros/ros.h"
#include "ros/console.h"

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>


namespace monitor{


class ExceptionMonitor
{
  public:
    ExceptionMonitor();
    ~ExceptionMonitor() = default;
    //存入传来的故障
    void addException(std::vector<std::string> &message_codes);
    //故障排序并返回最严重故障
    std::string selectException();

  private:
    //故障列表
    std::unordered_map<std::string, int> exception_map_;
    //传来的故障集合
    std::vector<std::pair<std::string, int>> exception_add_pairs_;
    //进行排序处理的集合
    std::vector<std::pair<std::string, int>> exception_select_pairs_;
    //载入故障列表
    bool loadExceptionList();

};




} // namespace monitor



#endif
