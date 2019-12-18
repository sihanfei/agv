#ifndef FSM_INCLUDE_STATE_BASE_MACHINE_H_
#define FSM_INCLUDE_STATE_BASE_MACHINE_H_

#include "ros/ros.h"
#include "state_define.h"

#include "hmi_msgs/FsmControVcuDriver.h"
#include "hmi_msgs/VMSControlAD.h"
#include "status_msgs/ADStatus.h"
#include "std_msgs/String.h"

#include <map>
#include <stdio.h>
#include <stdlib.h>
// #include <string>
#include <vector>

namespace superg_agv
{
namespace fsm
{
class StateBase;
// enum AgvStateID;
// enum AgvTransition;
// class StateMachine;

class StateMachine
{
public:
  StateMachine(ros::NodeHandle &nh_);
  virtual ~StateMachine();
  void stateInit();
  int translateState(int state_id_);
  void addState(StateBase *p_state_);
  void addTranslate(const int &stn_, const StateStatus &state_status_);
  void update();
  void showAllState();
  void recvAdStatusCallback(const status_msgs::ADStatus::ConstPtr &msg);
  void recvVmsControlCallback(const hmi_msgs::VMSControlAD::ConstPtr &msg);
  int getCurState();
  int getLateState();
  int vcuControlPub();
  void pushNodeEnableValue(const u_int8_t &fsm_value_, const u_int8_t &planning_value_,
                           const u_int8_t &operation_value_, const u_int8_t &exception_value_,
                           const u_int8_t &ad_value_);
  int getNodeEnableValue();
  int getAdEnableValue();
  int taskControlPub(const int &fsm_control_, int state_);
  int taskControlRePub(const int &recieve_task_num_);
  int fsmStatusPub();
  int addTaskCache(const hmi_msgs::VMSControlAD &task_temp_, const int &rtn_);
  int updateTaskCache();
  int updateCurCacheTaskResendTimes(const int &recieve_task_num_);
  bool getTaskSendStatus(const int &recieve_task_num_);
  int getCurTaskID();
  int getCurTaskSendID();
  bool getCurTaskSendStatus();
  int getCurTaskStatus();
  TaskStatus getCurTask();
  int taskTimerPlus();
  void curHmiControlValueInit();
  void showAllTaskPubCache();
  void showAllTaskCache();
  void showAllStateTranslate();
  int updateCurCacheTaskResult(const int &recieve_task_num_);
  void clearAllTaskCache();
  void timerCb(const ros::TimerEvent &event);
  void getNodeInfo();
  void doStateMachineSpinOnce();
  void curAdStatusValueInit();

public:
  std::map< int, StateBase * > m_state_cache;
  //定义上一个状态   //机器初始化时，没有上一个状态
  int m_prvious_state_id;
  StateBase *p_prvious_state;
  //定义当前状态
  int m_current_state_id;
  StateBase *p_current_state;
  //状态切换顺序 记录系统切换过程中的state顺序
  std::map< int, StateStatus > m_state_translate;

  //任务计数器
  int task_timer;

  AdStatusValue cur_ad_status;
  HmiControlValue cur_hmi_control_value;

  // ros 句柄
  ros::NodeHandle ns;
  // ros 收
  ros::Subscriber monitor_sub_;
  ros::Subscriber vms_info_sub_;
  // ros 发
  ros::Publisher planning_pub_;
  ros::Publisher exception_pub_;
  ros::Publisher operation_pub_;
  ros::Publisher vcu_control_pub_;
  ros::Publisher fsm_status_pub_;
  // ros 定时器
  ros::Timer machine_timer;
  double timer_duration;
  //状态机控制权
  VcuControlNodeStatus node_enable_status;
  std::map< int, hmi_msgs::VMSControlAD > m_task_pub_cache;
  std::vector< TaskStatus > m_task_cache;
  std::string node_name;
  u_int64_t node_pid;
};

class StateBase
{
public:
  StateBase(int id_, StateMachine *fsm_)
  {
  }
  virtual int doStateLoop()
  {
  }
  virtual void onEnter()
  {
  }
  virtual void onStay()
  {
  }
  virtual void onExit()
  {
  }
  virtual int doAdStatusJudge(const AdStatusValue &cur_ad_status_)
  {
  }
  virtual int doAdTaskStatusJudge()
  {
  }
  virtual int doHmiControlJudge(const HmiControlValue &cur_hmi_control_value_)
  {
  }

  virtual bool isTaskControlPubOK()
  {
  }

  virtual int isRepetitionAdStatus(const AdStatusValue &ad_status_a, const AdStatusValue &ad_status_b)
  {
    if (ad_status_a.agv_status != ad_status_b.agv_status)
    {
      return 0;
    }
    if (ad_status_a.node_Status != ad_status_b.node_Status)
    {
      return 0;
    }
    if (ad_status_a.hardware_Status != ad_status_b.hardware_Status)
    {
      return 0;
    }
    if (ad_status_a.planning_task_id != ad_status_b.planning_task_id)
    {
      return 0;
    }
    if (ad_status_a.planning_task_status != ad_status_b.planning_task_status)
    {
      return 0;
    }
    if (ad_status_a.operation_task_id != ad_status_b.operation_task_id)
    {
      return 0;
    }
    if (ad_status_a.operation_task_status != ad_status_b.operation_task_status)
    {
      return 0;
    }
    if (ad_status_a.exception_task_id != ad_status_b.exception_task_id)
    {
      return 0;
    }
    if (ad_status_a.exception_task_status != ad_status_b.exception_task_status)
    {
      return 0;
    }
    if (ad_status_a.ad_enbale_status != ad_status_b.ad_enbale_status)
    {
      return 0;
    }
    if (ad_status_a.node_error_Num != ad_status_b.node_error_Num)
    {
      return 0;
    }
    else
    {
      if (ad_status_a.node_error_Num > 0)
      {
        if (0 == compareStringVector(ad_status_a.node_error_code, ad_status_b.node_error_code))
        {
          return 0;
        }
      }
    }
    if (ad_status_a.hardware_error_num != ad_status_b.hardware_error_num)
    {
      return 0;
    }
    else
    {
      if (ad_status_a.hardware_error_num > 0)
      {
        if (0 == compareStringVector(ad_status_a.hardware_error_code, ad_status_b.hardware_error_code))
        {
          return 0;
        }
      }
    }
    return 1;
  }

  virtual int compareStringVector(const std::vector< std::string > &sv_a, const std::vector< std::string > &sv_b)
  {
    int count_a = static_cast< int >(sv_a.size());
    int count_b = static_cast< int >(sv_b.size());
    if (count_a != count_b)
    {
      return 0;
    }
    for (size_t i = 0; i < count_a; i++)
    {
      if (find(sv_b.begin(), sv_b.end(), sv_a.at(i)) == sv_b.end())
      {
        return 0;
      }
    }
    return 1;
  }

  virtual int taskReciever2State(const int &cur_reciecer_)
  {
    switch (cur_reciecer_)
    {
    case 2:
      return PLANNING;
    case 3:
      return EXCEPTION;
    case 4:
      return OPERATION;
    default:
      return NULL_STATE_ID;
    }
  }

  virtual void enableNode(const int &node_value_, const u_int8_t &ad_value_)
  {
    switch (node_value_)
    {
    case 1000:
      p_machine->pushNodeEnableValue(1, 0, 0, 0, ad_value_);
      break;
    case 1100:
      p_machine->pushNodeEnableValue(1, 1, 0, 0, ad_value_);
      break;
    case 1010:
      p_machine->pushNodeEnableValue(1, 0, 1, 0, ad_value_);
      break;
    case 1110:
      p_machine->pushNodeEnableValue(1, 1, 1, 0, ad_value_);
      break;
    case 1001:
      p_machine->pushNodeEnableValue(1, 0, 0, 1, ad_value_);
      break;
    default:
      p_machine->pushNodeEnableValue(1, 0, 0, 0, 0);
    }
  }
  virtual int changeNodeEnableValue(const int &cur_value_, const int &next_state_)
  {
    int next_value_ = 1000;
    if (next_state_ == PLANNING) // PLANNING 不修改 OPERATION 权限
    {
      return ((cur_value_ / 1000) * 1000 + ((cur_value_ / 10) % 10) * 10 + 100);
    }
    if (next_state_ == OPERATION) // OPERATION 不修改 PLANNING 权限
    {
      return ((cur_value_ / 100) * 100 + 10);
    }
    if (next_state_ == EXCEPTION)
    {
      return ((cur_value_ / 1000) * 1000 + 1);
    }
    return next_value_;
  }

  virtual u_int8_t nodeAdEnable()
  {
    return 1;
  }

  virtual u_int8_t nodeAdDisable()
  {
    return 0;
  }

  virtual int fsmControlNull()
  {
    return 0;
  }

  virtual int fsmControlOperation()
  {
    return 1;
  }

  virtual int fsmControlCancel()
  {
    return 2;
  }
  virtual int fsmControlCancelAll()
  {
    return 3;
  }

  virtual bool isAdStatusOK(const AdStatusValue &cur_ad_status_)
  {
    if (cur_ad_status_.agv_status == 2 && cur_ad_status_.node_Status == 1 && cur_ad_status_.hardware_Status == 1)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  virtual bool isRemoteState(const AdStatusValue &cur_ad_status_)
  {
    if (cur_ad_status_.agv_status != 2)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  virtual bool isExceptionState(const AdStatusValue &cur_ad_status_)
  {
    if (cur_ad_status_.node_Status == 0 || cur_ad_status_.hardware_Status == 0)
    {
      return 1;
    }
    else
    {
      return 0;
    }
  }

  virtual ~StateBase()
  {
  }

public:
  int id;
  int status_;
  int vms_status_;
  int ad_status_tip_;
  int hmi_status_tip_;
  int time_reset_tip_;

  StateMachine *p_machine;

  StateStatus cur_state_status;

  int node_enable_value_;
  u_int8_t node_enable_status_;
  int node_fsm_control_;
  TaskStatus m_cur_task_status;
  int state_status_pub_tip;
};

} // namespace fsm
} // namespace superg_agv

#endif