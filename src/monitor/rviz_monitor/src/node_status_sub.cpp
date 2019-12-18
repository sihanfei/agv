#include <ros/ros.h>

#include <iostream>

#include "common_functions.h"
// #include <sys/types.h>

#include <thread>

#include <list>
#include <map>

#include <yaml-cpp/yaml.h>

#include <sys/syscall.h>
#include <unistd.h> //getpid()

#include <vector>

#include "common_msgs/KeyValue.h"
#include "status_msgs/NodeStatus.h"
#include "status_msgs/SafetyStatus.h"

#include "status_msgs/ADStatus.h"

// #include "glog_helper.h"

using namespace std;

#define NODE_NAME "node_status_sub"

#define CONTI_YAML_FILE_PATH "/work/superg_agv/src/monitor/rviz_monitor/cfg"
#define CONTI_YAML_FILE_NAME "/node_list.yaml"

int count_node = 0;
map< string, status_msgs::SafetyStatus > g_map_node_status;
map< string, status_msgs::SafetyStatus > g_map_node_err;
map< string, status_msgs::SafetyStatus > g_map_hardware_err;

void nodeStatusCallback(const status_msgs::NodeStatus::ConstPtr &msg)
{
  count_node++;
  ros::Time t1 = ros::Time::now();
  // ROS_INFO("PID[%d] state_num[%d]", msg->node_pid, ( int )msg->status.size());
  map< string, status_msgs::SafetyStatus >::iterator it1;
  for (size_t i = 0; i < msg->status.size(); i++)
  {
    //判断状态信息等级
    if (msg->status[i].message_code.substr(0, 1) == "I" || msg->status[i].message_code.substr(0, 1) == "W")
    {
      g_map_node_status[msg->status[i].message_code] = msg->status[i];
    }
    else
    {
      if (msg->status[i].hardware_id == 0)
      {
        g_map_node_err[msg->status[i].message_code] = msg->status[i];
      }
      else
      {
        g_map_hardware_err[msg->status[i].message_code] = msg->status[i];
      }
    }
  }
  ros::Time t2    = ros::Time::now();
  ros::Duration d = t2 - t1;
  ROS_INFO("%s NSec = %ld", msg->node_name.c_str(), d.toNSec()); // d.toSec());
}

// adstatus
status_msgs::ADStatus g_ad_status_;
int main(int argc, char *argv[])
{
  ROS_INFO("ROS node is start, name is [%s], file name is %s", NODE_NAME, argv[0]);
  ros::init(argc, argv, NODE_NAME); // node name
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/node/node_status", 100, nodeStatusCallback);

  ros::Publisher ad_status_pub = n.advertise< status_msgs::ADStatus >("/monitor/ad_status", 1);

  map< string, status_msgs::SafetyStatus >::iterator status_it_;

  ////////////////////glog
  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory(NODE_NAME);
  ///////////////////////////////////////////////
  g_ad_status_.Agv_Status      = 3;
  g_ad_status_.Node_Status     = 1;
  g_ad_status_.Hardware_Status = 1;

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////
  ros::Rate loop_rate(10);

  int icount = 0;
  while (ros::ok())
  {
    ros::Time t1 = ros::Time::now();
    g_map_node_status.clear();
    g_map_node_err.clear();
    g_map_hardware_err.clear();

    g_ad_status_.Node_Error_Num = 0;
    g_ad_status_.Node_Error_Code.clear();

    g_ad_status_.Hardware_Error_Num = 0;
    g_ad_status_.Hardware_Error_Code.clear();

    count_node = 0;
    //触发回调
    ros::spinOnce();
    //消息头
    g_ad_status_.header.stamp = ros::Time::now();
    //获取Agv_Status
    status_it_ = g_map_node_status.find("I04011000"); //控制提供的常规状态编码
    if (status_it_ != g_map_node_status.end())
    {
      for (size_t i = 0; i < status_it_->second.values.size(); i++)
      {
        if (status_it_->second.values[i].key == "Agv_Status")
        {
          g_ad_status_.Agv_Status = stoi(status_it_->second.values[i].value);
        }
      }
    }

    //获取节点错误信息
    if (g_map_node_err.size() > 0)
    {
      g_ad_status_.Node_Status    = 0;
      g_ad_status_.Node_Error_Num = g_map_node_err.size();
      map< string, status_msgs::SafetyStatus >::iterator node_err_Iter;
      for (node_err_Iter = g_map_node_err.begin(); node_err_Iter != g_map_node_err.end(); node_err_Iter++)
        g_ad_status_.Node_Error_Code.push_back(node_err_Iter->second.message_code);
    }
    else
    {
      g_ad_status_.Node_Status = 1;
    }
    //获取硬件错误信息
    if (g_map_hardware_err.size() > 0)
    {
      g_ad_status_.Hardware_Status    = 0;
      g_ad_status_.Hardware_Error_Num = g_map_hardware_err.size();
      map< string, status_msgs::SafetyStatus >::iterator hardware_err_Iter;
      for (hardware_err_Iter = g_map_hardware_err.begin(); hardware_err_Iter != g_map_hardware_err.end();
           hardware_err_Iter++)
        g_ad_status_.Hardware_Error_Code.push_back(hardware_err_Iter->second.message_code);
    }
    else
    {
      g_ad_status_.Hardware_Status = 1;
    }
    //获取决策任务信息 planning_task_ID ,planning_task_status
    status_it_ = g_map_node_status.find("");
    if (status_it_ != g_map_node_status.end())
    {
      for (size_t i = 0; i < status_it_->second.values.size(); i++)
      {
        if (status_it_->second.values[i].key == "planning_task_ID")
        {
          g_ad_status_.planning_task_ID = stoi(status_it_->second.values[i].value);
        }
        if (status_it_->second.values[i].key == "planning_task_status")
        {
          g_ad_status_.planning_task_status = stoi(status_it_->second.values[i].value);
        }
      }
    }

    //获取业务任务信息 operation_task_ID , operation_task_status
    status_it_ = g_map_node_status.find("");
    if (status_it_ != g_map_node_status.end())
    {
      for (size_t i = 0; i < status_it_->second.values.size(); i++)
      {
        if (status_it_->second.values[i].key == "operation_task_ID")
        {
          g_ad_status_.operation_task_ID = stoi(status_it_->second.values[i].value);
        }
        if (status_it_->second.values[i].key == "operation_task_status")
        {
          g_ad_status_.operation_task_status = stoi(status_it_->second.values[i].value);
        }
      }
    }

    //获取故障处理任务信息 exception_task_ID , exception_task_status
    status_it_ = g_map_node_status.find("");
    if (status_it_ != g_map_node_status.end())
    {
      for (size_t i = 0; i < status_it_->second.values.size(); i++)
      {
        if (status_it_->second.values[i].key == "exception_task_ID")
        {
          g_ad_status_.exception_task_ID = stoi(status_it_->second.values[i].value);
        }
        if (status_it_->second.values[i].key == "exception_task_status")
        {
          g_ad_status_.exception_task_status = stoi(status_it_->second.values[i].value);
        }
      }
    }

    //获取VC驱动状态 ad_enbale_status
    status_it_ = g_map_node_status.find("");
    if (status_it_ != g_map_node_status.end())
    {
      for (size_t i = 0; i < status_it_->second.values.size(); i++)
      {
        if (status_it_->second.values[i].key == "ad_enbale_status")
        {
          g_ad_status_.ad_enbale_status = stoi(status_it_->second.values[i].value);
        }
      }
    }

    ad_status_pub.publish(g_ad_status_);
    ros::Time t2    = ros::Time::now();
    ros::Duration d = t2 - t1;
    ROS_INFO("[%d]node  [%ld]NSec", count_node, d.toNSec()); // d.toSec());

    loop_rate.sleep();
    icount++;
  }
  // ros::spin();
  return 0;
}