#include "hmi_msgs/FsmControVcuDriver.h"
#include "hmi_msgs/VMSControlAD.h"
#include "ros/ros.h"
#include "status_msgs/ADStatus.h"
#include "std_msgs/String.h"
#include <iostream>

using namespace std;

using std::__cxx11::string;

u_int8_t planning_task_ID  = 0; //任务状态
int planning_task_status   = 0; //任务状态
u_int8_t operation_task_ID = 0; //任务状态
int operation_task_status  = 0; //任务状态
u_int8_t exception_task_ID = 0; //任务状态
int exception_task_status  = 0; //任务状态
u_int8_t cur_task_ID       = -1;
int ad_enbale_status       = 0;

int planning_tip  = 0;
int operation_tip = 0;
int exception_tip = 0;

void callback1(const ros::TimerEvent &)
{
  // sleep(10);
  ROS_INFO("Callback 1 triggered");
}

void callback2(const ros::TimerEvent &)
{
  ROS_INFO("Callback 2 triggered");
}

int get_char(int ch)
{
  fd_set rfds;
  struct timeval tv;
  // int ch = 0;

  FD_ZERO(&rfds);
  FD_SET(0, &rfds);
  tv.tv_sec  = 1;
  tv.tv_usec = 0; //设置等待超时时间

  //检测键盘是否有输入
  if (select(1, &rfds, NULL, NULL, &tv) > 0)
  {
    ch = getchar() - 48;
  }

  return ch;
}

void recvPlanningCallback(const hmi_msgs::VMSControlAD::ConstPtr &msg)
{
  if (msg->task_ID > 0)
  {
    planning_task_ID = msg->task_ID;
    cur_task_ID      = msg->task_ID;
  }
  planning_tip = 1;
}

void recvExceptionCallback(const hmi_msgs::VMSControlAD::ConstPtr &msg)
{
  if (msg->task_ID > 0)
  {
    exception_task_ID = msg->task_ID;
    cur_task_ID       = msg->task_ID;
  }
  operation_tip = 1;
}

void recvOperationCallback(const hmi_msgs::VMSControlAD::ConstPtr &msg)
{
  if (msg->task_ID > 0)
  {
    operation_task_ID = msg->task_ID;
    cur_task_ID       = msg->task_ID;
  }
  exception_tip = 1;
}

void recvControlCallback(const hmi_msgs::FsmControVcuDriver::ConstPtr &msg)
{
  ad_enbale_status = msg->is_AD_status;
}

int main(int argc, char *argv[])
{

  ros::init(argc, argv, "ad_test_pub");

  ros::NodeHandle n;

  int agv_status        = 0;
  int node_status       = 0;
  int hardware_status   = 0;
  int state_tip         = 0;
  int state_task_status = 0;
  int tip_              = 0;

  if (argc == 5)
  {
    agv_status      = atoi(argv[1]);
    node_status     = atoi(argv[2]);
    hardware_status = atoi(argv[3]);
    tip_            = atoi(argv[4]);
    // state_task_status = atoi(argv[5]);
  }
  else
  {
    agv_status        = 0;
    node_status       = 0;
    hardware_status   = 0;
    state_tip         = 0;
    state_task_status = 0;
    tip_              = 0;
  }

  ros::Publisher ad_pub = n.advertise< status_msgs::ADStatus >("status_msgs", 1, true);

  ros::Subscriber planning_sub  = n.subscribe("/fsm/to_planning_order", 10, recvPlanningCallback);
  ros::Subscriber exception_sub = n.subscribe("/fsm/to_exception_order", 10, recvExceptionCallback);
  ros::Subscriber operation_sub = n.subscribe("/fsm/to_operation_order", 10, recvOperationCallback);
  ros::Subscriber control_sub   = n.subscribe("/fsm/fsm_control_vcu_driver", 10, recvControlCallback);

  ros::Rate loop_rate(1);

  int count = 0;

  std::cout << "please input state_tip" << std::endl;
  std::cin >> state_tip;

  switch (state_tip)
  {
  case 1: // planning_task_status
    std::cout << "please input status" << std::endl;
    std::cin >> planning_task_status;
    operation_task_status = 0;
    exception_task_status = 0;
    break;
  case 2: // operation_task_status
    std::cout << "please input status" << std::endl;
    std::cin >> operation_task_status;
    planning_task_status  = 0;
    exception_task_status = 0;
    break;
  case 3: // operation_task_status
    std::cout << "please input status" << std::endl;
    std::cin >> exception_task_status;
    planning_task_status  = 0;
    operation_task_status = 0;
    break;
  default:
    break;
    // planning_task_status  = 0;
    // operation_task_status = 0;
    // exception_task_status = 0;
  }
  ROS_INFO("state_tip %d planning %d operation %d excetion %d", state_tip, planning_task_status, operation_task_status,
           exception_task_status);

  while (ros::ok())
  {

    loop_rate.sleep();
    if (tip_ > 0)
    {
      planning_task_ID  = 0;
      operation_task_ID = 0;
      exception_task_ID = 0;
    }

    status_msgs::ADStatus ad_status_temp;
    ad_status_temp.Agv_Status            = agv_status;
    ad_status_temp.Node_Status           = node_status;           //整体状态
    ad_status_temp.Hardware_Status       = hardware_status;       //整体状态
    ad_status_temp.planning_task_ID      = planning_task_ID;      //任务状态
    ad_status_temp.planning_task_status  = planning_task_status;  //任务状态
    ad_status_temp.operation_task_ID     = operation_task_ID;     //任务状态
    ad_status_temp.operation_task_status = operation_task_status; //任务状态
    ad_status_temp.exception_task_ID     = exception_task_ID;     //任务状态
    ad_status_temp.exception_task_status = exception_task_status; //任务状态
    ad_status_temp.ad_enbale_status      = ad_enbale_status;      // ad模式
    ad_status_temp.Node_Error_Num        = 1;
    ad_status_temp.Hardware_Error_Num    = 1;

    std::__cxx11::string temp_ = "11";

    ad_status_temp.Node_Error_Code.push_back(temp_);
    ad_status_temp.Hardware_Error_Code.push_back(temp_);

    ad_pub.publish(ad_status_temp);

    ROS_INFO("pub agv %d node %d hardware %d task_id %u planning %u %d exception %u %d operation %u %d",
             ad_status_temp.Agv_Status, ad_status_temp.Node_Status, ad_status_temp.Hardware_Status, cur_task_ID,
             planning_task_ID, ad_status_temp.planning_task_status, exception_task_ID,
             ad_status_temp.exception_task_status, operation_task_ID, ad_status_temp.operation_task_status);

    ros::spinOnce();
    ++count;
  }

  // ros::Timer timer1 = n.createTimer(ros::Duration(1.0), callback1);
  // ros::Timer timer2 = n.createTimer(ros::Duration(0.2), callback2);

  // while (ros::ok())
  // {
  //   ros::spinOnce();
  // }
  //
  // ros::spin();

  ROS_INFO("shutting down!\n");
  ros::shutdown();

  return 0;
}
