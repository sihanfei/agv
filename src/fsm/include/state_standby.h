#ifndef FSM_INCLUDE_STATE_STANDBY_H_
#define FSM_INCLUDE_STATE_STANDBY_H_

#include "ros/ros.h"
#include "state_base_machine.h"
#include "state_define.h"

namespace superg_agv
{
namespace fsm
{

class StandbyState : public StateBase
{
public:
  StandbyState(int id_, StateMachine *fsm_) : StateBase(id, p_machine)
  {
    this->id                   = id_;
    this->p_machine            = fsm_;
    this->status_              = 0;
    this->vms_status_          = 0;
    time_reset_tip_            = 0;
    state_status_pub_tip         = 0;
    cur_state_status.time_in   = ros::Time::now();
    cur_state_status.time_out  = ros::Time::now();
    cur_state_status.cur_state = STANDBY;
    ad_status_tip_             = cur_state_status.cur_state;
    hmi_status_tip_            = cur_state_status.cur_state;
  }

  virtual int doStateLoop()
  {
    onEnter();
    onStay();
    onExit();
    return cur_state_status.next_state;
  }

  virtual void onEnter()
  {
    //获取上次状态
    cur_state_status.late_state = p_machine->getLateState();
    if (time_reset_tip_ == 0)
    {
      ROS_INFO("enter %s state, late state is %s", AgvStateStr[cur_state_status.cur_state].c_str(),
               AgvStateStr[cur_state_status.late_state].c_str());
      cur_state_status.time_in = ros::Time::now();
      time_reset_tip_          = 1;
    }
    //进入动作
    doEnterAction(cur_state_status.late_state);
  }

  virtual void onStay()
  {
    p_machine->doStateMachineSpinOnce();
    if (0 < (status_ + vms_status_))
    {
      //判断AD状态
      if (ad_status_tip_ != NULL_STATE_ID && status_ > 0)
      {
        cur_state_status.next_state = ad_status_tip_;
      }
      else
      {
        cur_state_status.next_state = cur_state_status.cur_state;
      }
      //非故障和romote状态进入判断任务
      if (cur_state_status.next_state == cur_state_status.cur_state && vms_status_ > 0)
      {
        cur_state_status.next_state = hmi_status_tip_;
      }
    }
    else
    {
      cur_state_status.next_state = cur_state_status.cur_state;
    }
  }

  virtual void onExit()
  {
    doExitAction(cur_state_status.next_state);
    cur_state_status.time_out = ros::Time::now();
    status_                   = 0;
    vms_status_               = 0;
    if (cur_state_status.cur_state != cur_state_status.next_state)
    {
      ROS_INFO("%s  -->> %.8lf(s) -->>  %s", AgvStateStr[cur_state_status.cur_state].c_str(),
               cur_state_status.time_out.toSec() - cur_state_status.time_in.toSec(),
               AgvStateStr[cur_state_status.next_state].c_str());
      time_reset_tip_ = 0;
    }
  }

  virtual void doEnterAction(int state_)
  {
    //通用初始化状态切换标志
    status_             = 0;
    vms_status_         = 0;
    node_enable_value_  = p_machine->getNodeEnableValue();
    node_enable_status_ = p_machine->getAdEnableValue();
    node_fsm_control_   = fsmControlOperation();
    p_machine->curHmiControlValueInit();
    //根据前置状态选择不同的动作
    if (state_ == STARTUP)
    {
    }
    else if (state_ == STANDBY)
    {
    }
    else if (state_ == PLANNING)
    {
    }
    else if (state_ == OPERATION)
    {
    }
    else
    {
    }
  }

  virtual void doExitAction(int &state_)
  {
    //根据后续状态选择不同的动作
    node_enable_value_ = changeNodeEnableValue(node_enable_value_, state_);
    if (state_ == STARTUP)
    {
      node_enable_status_ = nodeAdDisable();
      node_fsm_control_   = fsmControlNull();
    }
    else if (state_ == STANDBY)
    {
      node_enable_status_ = nodeAdDisable();
      node_fsm_control_   = fsmControlNull();
    }
    else if (state_ == PLANNING)
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
    }
    else if (state_ == OPERATION)
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
    }
    else if (state_ == EXCEPTION) //区分任务和故障
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      if (vms_status_ == 0)
      {
        p_machine->curHmiControlValueInit();
        p_machine->cur_hmi_control_value.recieve_task_num = p_machine->taskTimerPlus();
      }
      control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
    }
    else if (state_ == REMOTE)
    {
      node_enable_status_ = nodeAdDisable();
      node_fsm_control_   = fsmControlNull();
    }
    else
    {
      node_enable_status_ = nodeAdDisable();
      node_fsm_control_   = fsmControlNull();
    }
    //改变控制权
    enableNode(node_enable_value_, node_enable_status_);
    if (1 == state_status_pub_tip)
    {
      state_status_pub_tip = 0;
      //心跳
      p_machine->vcuControlPub();
      //状态
      p_machine->fsmStatusPub();
    }
  }

  virtual int doAdStatusJudge(const AdStatusValue &cur_ad_status_)
  {

    m_cur_task_status = p_machine->getCurTask();

    if (isRemoteState(cur_ad_status_))
    {
      return REMOTE;
    }
    else if (isExceptionState(cur_ad_status_))
    {
      return EXCEPTION;
    }
    else if (isAdStatusOK(cur_ad_status_))
    {
      if (0 == m_cur_task_status.recieve_task_num && 0 == m_cur_task_status.task_id && 0 == m_cur_task_status.cmd_type)
      {
        return cur_state_status.cur_state;
      }
      else
      {
        return doCurTaskJudge();
      }
    }
    else
    {
      return cur_state_status.cur_state;
    }
  }

  virtual int doCurTaskJudge()
  {
    ROS_INFO("Do task judge");
    int state_temp_ = taskReciever2State(m_cur_task_status.reciever);
    if (state_temp_ == NULL_STATE_ID)
    {
      return cur_state_status.cur_state;
    }
    else
    {
      return state_temp_;
    }
  }

  virtual int doHmiControlJudge(const HmiControlValue &cur_hmi_control_value_)
  {
    switch (cur_hmi_control_value_.cmd_type)
    {
    case 0: // 0 – 空操作
      return PLANNING;
      break;
    case 1: // 1 – 行走
      return PLANNING;
      break;
    case 2: // 2 – 顶升上升
      return OPERATION;
      break;
    case 3: // 3 – 顶升下降
      return OPERATION;
      break;
    case 4: // 4 – 充电
      return OPERATION;
      break;
    case 5: // 5 – 停止充电
      return OPERATION;
      break;
    case 6: // 6 – 紧停
      return EXCEPTION;
      break;
    case 7: // 7 – 复位
      return STARTUP;
      break;
    case 8: // 8 – 任务暂停
      return STANDBY;
      break;
    case 9: // 9 – 任务继续
      return STANDBY;
      break;
    case 10: // 10 – 任务取消
      return STANDBY;
      break;
    case 11: // 11 – 停稳
      return PLANNING;
      break;
    case 12: // 12 – 滑停
      return EXCEPTION;
      break;
    default: //任务外命令，报错误
      return EXCEPTION;
    }
  }

public:
  int control_pub_id;
};

} // namespace fsm
} // namespace superg_agv

#endif