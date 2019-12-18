#ifndef FSM_INCLUDE_STATE_REMOTE_H_
#define FSM_INCLUDE_STATE_REMOTE_H_

#include "ros/ros.h"
#include "state_base_machine.h"
#include "state_define.h"

namespace superg_agv
{
namespace fsm
{

class RemoteState : public StateBase
{
public:
  RemoteState(int id_, StateMachine *fsm_) : StateBase(id, p_machine)
  {
    this->id                   = id_;
    this->p_machine            = fsm_;
    this->status_              = 0;
    this->vms_status_          = 0;
    time_reset_tip_            = 0;
    state_status_pub_tip         = 0;
    cur_state_status.time_in   = ros::Time::now();
    cur_state_status.time_out  = ros::Time::now();
    cur_state_status.cur_state = REMOTE;
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

    //判断AD状态
    if (ad_status_tip_ != NULL_STATE_ID && status_ > 0)
    {
      cur_state_status.next_state = ad_status_tip_;
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

  virtual void doEnterAction(int &state_)
  {
    //通用初始化状态切换标志
    status_             = 0;
    vms_status_         = 0;
    node_enable_value_  = p_machine->getNodeEnableValue();
    node_enable_status_ = p_machine->getAdEnableValue();
    node_fsm_control_   = fsmControlOperation();
    //根据前置状态选择不同的动作
    if (state_ == STARTUP)
    {
    }
    else if (state_ == EXCEPTION)
    {
    }
    else if (state_ == STANDBY)
    {
    }
    else if (state_ == REMOTE)
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
    else if (state_ == EXCEPTION)
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      p_machine->taskControlPub(node_fsm_control_, state_);
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
      return STANDBY;
    }
    else
    {
      return cur_state_status.cur_state;
    }
  }

public:
};

} // namespace fsm
} // namespace superg_agv

#endif