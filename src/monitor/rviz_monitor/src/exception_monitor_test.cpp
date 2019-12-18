
#include "exception_monitor.h"

using namespace monitor;

/*
改一下exception_monitor.cpp里面ExceptionMonitor::loadExceptionList()中exception_list_file.open后面文件的路径
exception_list_csv.csv是用来模拟测试的文件
*/

int main(int argc, char** argv)
{
  //对象在monitor节点开启时创建一次就可以了
  ExceptionMonitor exception_monitor;

  //模拟传来的故障id[]
  std::vector<std::string> message_codes1 = {"W03011001", "E03011002","E03011002"};
  std::vector<std::string> message_codes2 = {"F03011003","aaaaaa"};
  std::vector<std::string> message_codes3 = {};


  //传来的故障id[]这里接收,传来的故障在载入的列表里并没有记载的话会被筛掉并警告
  exception_monitor.addException(message_codes1);
  exception_monitor.addException(message_codes2);
  exception_monitor.addException(message_codes3);


  //返回挑选出的id，没有故障返回"0"
  std::cout << "final id : " << exception_monitor.selectException() << std::endl;

  return 1;
}
