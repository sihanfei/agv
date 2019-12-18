#ifndef FSM_INCLUDE_STATE_PLANNING_H_
#define FSM_INCLUDE_STATE_PLANNING_H_

#include "ros/ros.h"
#include "state_base_machine.h"
#include "state_define.h"

namespace superg_agv
{
namespace fsm
{

class PlanningState : public StateBase
{
public:
  PlanningState(int id_, StateMachine *fsm_) : StateBase(id, p_machine)
  {
    this->id                   = id_;
    this->p_machine            = fsm_;
    this->status_              = 0;
    this->vms_status_          = 0;
    this->control_pub_id       = 0;
    time_reset_tip_            = 0;
    state_status_pub_tip       = 0;
    cur_state_status.time_in   = ros::Time::now();
    cur_state_status.time_out  = ros::Time::now();
    cur_state_status.cur_state = PLANNING;
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
      if (!isTaskControlPubOK(m_cur_task_status.recieve_task_num))
      {
        //重新发布未执行的控制命令
        p_machine->taskControlRePub(m_cur_task_status.recieve_task_num);
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
    //通用初始化状态切换标志
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
    if (state_ == STANDBY)
    {
      node_enable_status_ = nodeAdDisable();
      node_fsm_control_   = fsmControlNull();
    }
    else if (state_ == PLANNING)
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      if (vms_status_ > 0)
      {
        control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
      }
    }
    else if (state_ == OPERATION)
    {
      node_enable_status_ = nodeAdEnable();
      node_fsm_control_   = fsmControlOperation();
      //下发任务
      if (vms_status_ > 0)
      {
        control_pub_id = p_machine->taskControlPub(node_fsm_control_, state_);
      }
    }
    else if (state_ == EXCEPTION) //区分任务和故障
    {
      node_enable_status_ = nodeAdEnable();
      if ((vms_status_ + status_) > 0)
      {
        //取消当前节点任务
        node_fsm_control_ = fsmControlCancel();
        control_pub_id    = p_machine->taskControlPub(node_fsm_control_, cur_state_status.cur_state);
        //下发故障任务
        node_fsm_control_ = fsmControlOperation();
        control_pub_id    = p_machine->taskControlPub(node_fsm_control_, state_);
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
        ROS_INFO("%s get NULL task", AgvStateStr[cur_state_status.cur_state].c_str());
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
        if (m_cur_task_status.status < 3)
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
            return STANDBY;
          }
        }
      }
    }
    else
    {
      return state_temp_;
    }
  }

  virtual int doAdTaskStatusJudge(const int &cur_status_)
  {
    switch (cur_status_)
    {
    case 0: // 0– 未执行
      return cur_state_status.cur_state;
      break;
    case 1: // 1– 执行任务中
      return cur_state_status.cur_state;
      break;
    case 2: // 2– 等待任务执行结果
      return cur_state_status.cur_state;
      break;
    case 3: // 3– 任务结束，成功，离开当前状态
      return STANDBY;
      break;
    case 4: // 4– 任务结束，失败，离开当前状态
      return STANDBY;
      break;
    case 5: // 5– 任务结束，故障，切换到Exception
      return EXCEPTION;
      break;
    case 6: // 6– 按照命令任务中止，返回STANDBY
      return STANDBY;
      break;
    case 7: // 7– 自动判断任务中止，离开当前状态
      return STANDBY;
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
      return cur_state_status.cur_state;
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
      return EXCEPTION;
      break;
    case 8: // 8 – 任务暂停
      return cur_state_status.cur_state;
      break;
    case 9: // 9 – 任务继续
      return cur_state_status.cur_state;
      break;
    case 10: // 10 – 任务取消
      return cur_state_status.late_state;
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

public:
  int control_pub_id;
  bool control_pub_status;
  int msg_operation_status;
};

} // namespace fsm
} // namespace superg_agv

#endif