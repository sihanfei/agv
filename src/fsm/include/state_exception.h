#ifndef FSM_INCLUDE_STATE_EXCEPTION_HANDLING_H_
#define FSM_INCLUDE_STATE_EXCEPTION_HANDLING_H_

#include "ros/ros.h"
#include "state_base_machine.h"
#include "state_define.h"

namespace superg_agv
{
namespace fsm
{
class ExceptionState : public StateBase
{
public:
  ExceptionState(int id_, StateMachine *fsm_) : StateBase(id, p_machine)
  {
    this->id                     = id_;
    this->p_machine              = fsm_;
    this->status_                = 0;
    this->vms_status_            = 0;
    this->control_pub_id         = 0;
    time_reset_tip_              = 0;
    ad_status_repetition         = 0;
    m_cur_task_status_update_tip = 0;
    state_status_pub_tip         = 0;
    task_send_tip                = 0;
    cur_state_status.time_in     = ros::Time::now();
    cur_state_status.time_out    = ros::Time::now();
    cur_state_status.cur_state   = EXCEPTION;
    ad_status_tip_               = cur_state_status.cur_state;
    hmi_status_tip_              = cur_state_status.cur_state;
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
      //判断任务发布状态
      if (1 == m_cur_task_status_update_tip)
      {
        if (!isTaskControlPubOK(m_cur_task_status.recieve_task_num))
        {
          //重新发布未执行的控制命令
          p_machine->taskControlRePub(m_cur_task_status.recieve_task_num);
        }
      }
      //判断AD状态
      if (ad_status_tip_ != NULL_STATE_ID && status_ > 0)
      {
        cur_state_status.next_state = ad_status_tip_;
      }
      else
      {
        cur_state_status.next_state = cur_state_status.cur_state;
      }
      //判断任务执行状态
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
    status_             = 0;
    vms_status_         = 0;
    node_enable_value_  = p_machine->getNodeEnableValue();
    node_enable_status_ = p_machine->getAdEnableValue();
    node_fsm_control_   = fsmControlOperation();

    //根据前置状态选择不同的动作
    if (state_ == STANDBY)
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
    else if (state_ == EXCEPTION) //区分任务和故障
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      if (ad_status_repetition == 0)
      {
        if (vms_status_ == 0)
        {
          p_machine->curHmiControlValueInit();
          p_machine->cur_hmi_control_value.recieve_task_num = p_machine->taskTimerPlus();
        }
        ad_status_repetition = 1;
        control_pub_id       = p_machine->taskControlPub(node_fsm_control_, state_);
      }
      else
      {
        if (vms_status_ > 0 && task_send_tip == 1)
        {
          task_send_tip  = 0;
          control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
        }
      }
    }
    else if (state_ == REMOTE)
    {
      //下发任务
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
    ROS_INFO("%s do adstatus judge", AgvStateStr[cur_state_status.cur_state].c_str());
    int updata_num_ = p_machine->updateTaskCache();

    if (updata_num_ > 0)
    {
      ROS_WARN("%s adstatus had update, get new Task", AgvStateStr[cur_state_status.cur_state].c_str());
      m_cur_task_status = p_machine->getCurTask();
      if (0 == m_cur_task_status.recieve_task_num && 0 == m_cur_task_status.task_id && 0 == m_cur_task_status.cmd_type)
      {
        m_cur_task_status_update_tip = 0;
      }
      else
      {
        m_cur_task_status_update_tip = 1;
      }
    }

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
    ROS_INFO("%s do task judge", AgvStateStr[cur_state_status.cur_state].c_str());
    int state_temp_ = taskReciever2State(m_cur_task_status.reciever);
    if (state_temp_ == NULL_STATE_ID)
    {
      return cur_state_status.cur_state;
    }
    else if (state_temp_ == cur_state_status.cur_state)
    {
      if (0 == m_cur_task_status.is_send_ok)
      {
        ROS_ERROR("%s do task rsn:%d id:%u cmd:%u status judge ---- wait task recieve",
                  AgvStateStr[cur_state_status.cur_state].c_str(), m_cur_task_status.recieve_task_num,
                  m_cur_task_status.task_id, m_cur_task_status.cmd_type);
        return cur_state_status.cur_state;
      }
      else
      {
        if (m_cur_task_status.status < 4)
        {
          //等待任务完成
          ROS_WARN("%s do task rsn:%d id:%u cmd:%u status judge ---- wait task done",
                   AgvStateStr[cur_state_status.cur_state].c_str(), m_cur_task_status.recieve_task_num,
                   m_cur_task_status.task_id, m_cur_task_status.cmd_type);
          return doAdTaskStatusJudge(m_cur_task_status.status);
        }
        else
        {
          if (0 == m_cur_task_status.result)
          {
            ROS_WARN("%s do task rsn:%d id:%u cmd:%u status judge ---- task had done update result",
                     AgvStateStr[cur_state_status.cur_state].c_str(), m_cur_task_status.recieve_task_num,
                     m_cur_task_status.task_id, m_cur_task_status.cmd_type);
            p_machine->updateCurCacheTaskResult(m_cur_task_status.recieve_task_num);
            m_cur_task_status.result = 1;
            return doAdTaskStatusJudge(m_cur_task_status.status);
          }
          else
          {
            ROS_WARN("%s do task rsn:%d id:%u cmd:%u status judge ---- task had done RESET cur",
                     AgvStateStr[cur_state_status.cur_state].c_str(), m_cur_task_status.recieve_task_num,
                     m_cur_task_status.task_id, m_cur_task_status.cmd_type);
            m_cur_task_status = p_machine->getCurTask();
            ROS_INFO("%s cur task rsn:%d id:%u cmd:%u", AgvStateStr[cur_state_status.cur_state].c_str(),
                     m_cur_task_status.recieve_task_num, m_cur_task_status.task_id, m_cur_task_status.cmd_type);
            return cur_state_status.cur_state;
          }
        }
      }
    }
    else
    {
      return cur_state_status.cur_state;
      // return STARTUP;
    }
  }

  virtual int doAdTaskStatusJudge(const int &cur_status_)
  {
    switch (cur_status_)
    {
    case 0: // 0– 未执行
      return cur_state_status.cur_state;
      break;
    case 1: // 1– 故障处理方式判断中
      return cur_state_status.cur_state;
      break;
    case 2: // 2– 完成故障处理命令下发
      return cur_state_status.cur_state;
      break;
    case 3: // 3– 等待故障处理结果
      return cur_state_status.cur_state;
      break;
    case 4: // 4– 故障完成，车辆保持停止，保持Exception状态
      return cur_state_status.cur_state;
      break;
    case 5: // 5– 故障完成，切换startup
      return STARTUP;
      break;
    case 6: // 6– 故障处理失败，故障锁定，保持Exception状态
      return EXCEPTION;
      break;
    case 7: // 7– 故障处理失败，切换startup
      return STARTUP;
      break;
    case 8: // 8– 故障处理失败，命令未生效，保持Exception状态
      return EXCEPTION;
      break;
    case 9: // 9– 故障处理中止，按照命令，切换startup
      return STARTUP;
      break;
    case 10: // 10– 故障处理中止，自动判断，切换startup
      return STARTUP;
      break;
    case 11: // 11– 故障处理中止，保持Exception
      return EXCEPTION;
      break;
    default:
      return cur_state_status.cur_state;
    }
  }

  virtual int doHmiControlJudge(const HmiControlValue &cur_hmi_control_value_) //根据命令更新ad状态
  {
    switch (cur_hmi_control_value_.cmd_type)
    {
    case 0: // 0 – 空操作
      task_send_tip = 1;
      return cur_state_status.cur_state;
      break;
    case 6: // 6 – 紧停
      task_send_tip = 1;
      return EXCEPTION;
      break;
    case 7: // 7 – 复位
      task_send_tip = 1;
      return STARTUP;
      break;
    case 8: // 8 – 任务暂停
      task_send_tip = 1;
      return cur_state_status.cur_state;
      break;
    case 9: // 9 – 任务继续
      task_send_tip = 1;
      return cur_state_status.cur_state;
      break;
    case 10: // 10 – 任务取消
      task_send_tip = 1;
      return cur_state_status.late_state;
      break;
    case 12: // 12 – 滑停
      task_send_tip = 1;
      return EXCEPTION;
      break;
    default: //任务外命令，报错误 大于范围的报错，范围内留在错误状态
      return EXCEPTION;
    }
  }

  virtual bool isTaskControlPubOK(const int &recieve_task_num_)
  {
    if (recieve_task_num_ > 0)
    {
      return p_machine->getTaskSendStatus(recieve_task_num_);
    }
    else
    {
      return 1;
    }
  }

  virtual int isRepetitionAdStatus(const AdStatusValue &ad_status_a, const AdStatusValue &ad_status_b)
  {
    if (ad_status_a.agv_status != ad_status_b.agv_status)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.node_Status != ad_status_b.node_Status)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.hardware_Status != ad_status_b.hardware_Status)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.exception_task_id != ad_status_b.exception_task_id)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.exception_task_status != ad_status_b.exception_task_status)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.ad_enbale_status != ad_status_b.ad_enbale_status)
    {
      ad_status_repetition = 0;
      return 0;
    }
    if (ad_status_a.node_error_Num != ad_status_b.node_error_Num)
    {
      ad_status_repetition = 0;
      return 0;
    }
    else
    {
      if (ad_status_a.node_error_Num > 0)
      {
        if (0 == compareStringVector(ad_status_a.node_error_code, ad_status_b.node_error_code))
        {
          ad_status_repetition = 0;
          return 0;
        }
      }
    }
    if (ad_status_a.hardware_error_num != ad_status_b.hardware_error_num)
    {
      ad_status_repetition = 0;
      return 0;
    }
    else
    {
      if (ad_status_a.hardware_error_num > 0)
      {
        if (0 == compareStringVector(ad_status_a.hardware_error_code, ad_status_b.hardware_error_code))
        {
          ad_status_repetition = 0;
          return 0;
        }
      }
    }
    ad_status_repetition = 1;
    return 1;
  }

public:
  int control_pub_id;
  bool control_pub_status;
  int msg_operation_status;
  int ad_status_repetition;
  int m_cur_task_status_update_tip;
  int task_send_tip;
};
} // namespace fsm
} // namespace superg_agv

#endif