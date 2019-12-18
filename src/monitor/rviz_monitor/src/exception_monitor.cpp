
#include "exception_monitor.h"

namespace monitor{

ExceptionMonitor::ExceptionMonitor()
{
  if(loadExceptionList())
  {
    ROS_INFO_STREAM("Successfully loaded the exception list...");
    //std::cout << "Successfully loaded the exception list..." << std::endl;
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load exception list...");
    //std::cout << "Failed to load exception list..." << std::endl;
  }
}

//载入故障列表
bool ExceptionMonitor::loadExceptionList()
{
  //打开故障列表文件并检测是否成功
  std::ifstream exception_list_file;
  //改一下路径~~
  exception_list_file.open("/home/wency/exception_catkin/src/exception/data/exception_list_csv.csv", std::ios::in);
  if(!exception_list_file.is_open())
  {
    ROS_ERROR_STREAM("Error opening file.");
    //std::cout << "Error opening file." << std::endl;
    return false;
  }

  //截止到末尾行逐行读取，以“，”分隔
  while(!exception_list_file.eof())
  {
    std::string line, line_orig;
    while(getline(exception_list_file, line_orig))
    {
      line = "";
      for (char a:line_orig)//过滤非法字符
      {
        if((a >= 65 && a <= 122) || (a >= 48 && a <= 57) || a == 44)
        //if(a >= 0 && a <= 127)
        {
          line += a;
        }
      }

      std::istringstream sin(line);
      std::string info;
      std::vector<std::string> infos;
      while(getline(sin, info, ',')){
        infos.emplace_back(info);
      }

      std::stringstream s_exception_level;
      int exception_level;
      std::string exception_id = infos[0];//故障id
      s_exception_level << infos[1];
      s_exception_level >> exception_level;//等级

      std::pair<std::string, int> exception_pair(exception_id, exception_level);
      exception_map_.insert(exception_pair);//存入
    }
  }
  //打印存入的id
  for(std::unordered_map<std::string, int>::iterator it=exception_map_.begin(); it!=exception_map_.end();it++)
  {
    ROS_DEBUG_STREAM("Load exception id : " << it->first);
    //std::cout << "Load exception id : " << it->first << std::endl;
  }

  return true;
}

//存入传来的故障 参数：故障ids[]
void ExceptionMonitor::addException(std::vector<std::string> &message_codes)
{
  //空检查
  if(message_codes.empty())
  {
  	return;
  }

  int exception_level;
  std::pair<std::string, int> exception_pair;
  for(int i=0; i<message_codes.size(); i++)
  {
    if(exception_map_.find(message_codes[i]) == exception_map_.end())
    {
      ROS_WARN_STREAM("Failed to search id : " <<  message_codes[i]);
      //std::cout << "Failed to search id : " << message_codes[i] << std::endl;
    }
    else
    {
      //获得相应等级
      exception_level = exception_map_.at(message_codes[i]);
      exception_pair = make_pair(message_codes[i], exception_level);
      //重复检测
      std::vector<std::pair<std::string, int>>::iterator it_pair = find(exception_add_pairs_.begin(), exception_add_pairs_.end(), exception_pair);
      if(it_pair == exception_add_pairs_.end())
      {
        //存入
        ROS_DEBUG_STREAM("Add exception id : " << exception_pair.first);
        //std::cout << "Add exception id : " << exception_pair.first << std::endl;
        exception_add_pairs_.emplace_back(exception_pair);
      }
    }

  }

  return;
}

//排序 返回最高等级id
std::string ExceptionMonitor::selectException()
{
  //检查 是否为空
  if(exception_add_pairs_.size() == 0)
  {
  	return "0";
  }

  //储存好的故障拿出来，原数组清空
  exception_select_pairs_.assign(exception_add_pairs_.begin(), exception_add_pairs_.end());

  exception_add_pairs_.clear();
  for(auto& exception_select_pair : exception_select_pairs_)
  {
    ROS_DEBUG_STREAM("Select exception id : " << exception_select_pair.first);
    //std::cout << "select id : " << exception_select_pair.first << std::endl;
  }

  //按等级排序
  std::sort(exception_select_pairs_.begin(), exception_select_pairs_.end(),
  	[](const std::pair<std::string, int> &pair0, const std::pair<std::string, int> &pair1){return pair0.second > pair1.second;});

  return exception_select_pairs_[0].first;
}


} // namespace monitor
