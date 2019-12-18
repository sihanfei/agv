#include "ros/ros.h"
#include "std_msgs/String.h"

#include <chrono>
#include <iostream>
#include <sstream>

#include <boost/thread.hpp>

#include <fstream>
#include <ios>
#include <iostream>
#include <vector>

using namespace std;
using namespace chrono;

void func1(const int &id)
{
  cout << "func1 id : " << id << endl;
}

struct MyThread
{
  void operator()(const int &id)
  {
    cout << "MyThread id : " << id << endl;
  }

  void func1(const int &id)
  {
    cout << "MyThread::func1 id : " << id << endl;
  }
};

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "test");

  ros::NodeHandle n;

  //普通函数
  boost::thread t1(func1, 11);
  t1.join();

  //函数对象
  MyThread myThread;
  boost::thread t2(myThread, 22);
  t2.join();

  //成员函数
  boost::thread t3(&MyThread::func1, myThread, 33);
  t3.join();

  //临时对象
  boost::thread t4(MyThread(), 44);
  t4.join();

  //对象引用
  boost::thread t5(boost::ref(myThread), 55);
  t5.join();

  /////////////////////////////////////////////////////////
  ros::Time t01 = ros::Time::now();
  // ifstream fin("/home/linh/work/agvdata/rs1-192.168.2.109-7109-20190821-110000.csv", std::ios::binary);
  ifstream fin("/home/linh/work/agvdata/rs1-192.168.2.109-7109-20190821-110000.csv");

  // std::ifstream t("file.txt");
  // std::stringstream buffer;
  // buffer << t.rdbuf();
  // std::string contents(buffer.str());

  stringstream buffer;
  buffer << fin.rdbuf();
  fin.close();
  // cout << buf << endl;
  // vector< char > buf(static_cast< unsigned int >(fin.seekg(0, std::ios::end).tellg()));
  // fin.seekg(0, std::ios::beg).read(&buf[0], static_cast< std::streamsize >(buf.size()));

  string line;
  while (getline(buffer, line))
  {
    cout << line << endl;
  }

  ros::Time t02   = ros::Time::now();
  ros::Duration d = t02 - t01;

  cout << d.toSec() << " ns" << endl;

  /////////////////////////////////
  ros::Rate loop_rate(1);

  int count = 0;
  while (ros::ok())
  {
    // auto t0 = system_clock::now();

    // // cout << "*" << endl;
    // auto t1 = system_clock::now();

    // auto d1 = duration_cast< nanoseconds >(t1 - t0);

    // cout << d1.count() << "ns" << endl;

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}