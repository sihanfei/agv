#include "state_base_machine.h"

using std::string;

namespace superg_agv
{
namespace fsm
{

StateMachine::StateMachine(ros::NodeHandle &nh_)
{
  ns               = nh_;
  monitor_sub_     = ns.subscribe("status_msgs", 10, &StateMachine::recvAdStatusCallback, this);
  vms_info_sub_    = ns.subscribe("/map/vms_control_info", 1, &StateMachine::recvVmsControlCallback, this);
  planning_pub_    = ns.advertise< hmi_msgs::VMSControlAD >("/fsm/to_planning_order", 10, true);
  exception_pub_   = ns.advertise< hmi_msgs::VMSControlAD >("/fsm/to_exception_order", 10, true);
  operation_pub_   = ns.advertise< hmi_msgs::VMSControlAD >("/fsm/to_operation_order", 10, true);
  vcu_control_pub_ = ns.advertise< hmi_msgs::FsmControVcuDriver >("/fsm/fsm_control_vcu_driver", 10, true);
  // fsm_status_pub_  = ns.advertise< hmi_msgs::FsmControVcuDriver >("/fsm/fsm_control_vcu_driver", 10,
  // true);//需要确定消息格式后修改
  timer_duration = 1.0;
  machine_timer  = ns.createTimer(ros::Duration(timer_duration), &StateMachine::timerCb, this);

  this->stateInit();
}

StateMachine::~StateMachine()
{
}

void StateMachine::timerCb(const ros::TimerEvent &event)
{
  p_current_state->state_status_pub_tip = 1;
}

void StateMachine::doStateMachineSpinOnce()
{
  ros::spinOnce();
}

void StateMachine::stateInit()
{
  this->getNodeInfo();
  this->curAdStatusValueInit();
  m_prvious_state_id = NULL_STATE_ID;
  p_prvious_state    = NULL;
  m_current_state_id = NULL_STATE_ID;
  p_current_state    = NULL;
  task_timer         = 0;
  this->pushNodeEnableValue(1, 0, 0, 0, 0); //控制权初始化
}

void StateMachine::getNodeInfo()
{
  node_name = ros::this_node::getName();
  node_pid  = getpid();
  // ROS_INFO("FSM node name is %s, pid is %lu", node_name.c_str(), node_pid);
}

int StateMachine::translateState(int state_id_)
{
  // ROS_INFO("translate to %s, current state %s, prvious state %s", AgvStateStr[state_id_].c_str(),
  //          AgvStateStr[m_current_state_id].c_str(), AgvStateStr[m_prvious_state_id].c_str());
  int state_temp_ = NULL_STATE_ID;
  if (state_id_ == m_current_state_id)
  {
    // ROS_INFO("transfrom state is curent state %s, return!", AgvStateStr[state_id_].c_str());
    state_temp_ = p_current_state->doStateLoop();
    return state_temp_;
  }
  else
  {

    //检查状态是否存在，存在切换，不存在切换回startup
    std::map< int, StateBase * >::iterator it = m_state_cache.find(state_id_);
    if (it != m_state_cache.end())
    {
      m_prvious_state_id = m_current_state_id;
      p_prvious_state    = p_current_state;

      m_current_state_id = it->first;
      p_current_state    = it->second;

      state_temp_ = it->second->doStateLoop();
      return state_temp_;
    }
    else
    {
      ROS_WARN("The target state is %s doesn't during transfrom state list, go to STARTUP",
               AgvStateStr[state_id_].c_str());
      //列表中未找到要转移的状态ID，切换回startup
      it          = m_state_cache.find(STARTUP);
      state_temp_ = it->second->doStateLoop();
      return state_temp_;
    }
  }
}

void StateMachine::addTranslate(const int &stn_, const StateStatus &state_status_)
{
  if (p_current_state != NULL)
  {
    this->m_state_translate.insert(std::pair< int, StateStatus >(stn_, state_status_));
    this->showAllStateTranslate();
  }
}

void StateMachine::addState(StateBase *p_state_)
{
  std::map< int, StateBase * >::iterator it = m_state_cache.find(p_state_->id);
  if (it == m_state_cache.end())
  {
    this->m_state_cache.insert(std::pair< int, StateBase * >(p_state_->id, p_state_));
    // ROS_INFO("add %d %s state in map", p_state_->id, AgvStateStr[p_state_->id].c_str());
  }
  else
  {
    // ROS_INFO("The map had same state as %d %s", p_state_->id, AgvStateStr[p_state_->id].c_str());
  }
}

void StateMachine::update()
{
  if (p_current_state != NULL)
  {
    std::map< int, StateBase * >::iterator it = m_state_cache.find(p_current_state->id);
    it->second->onStay();
  }
}

void StateMachine::showAllState()
{
  std::map< int, StateBase * >::iterator it;
  int loop_index = 0;
  for (it = m_state_cache.begin(); it != m_state_cache.end(); it++)
  {
    ++loop_index;
    ROS_INFO("read state map %d key is %d %s.", loop_index, it->first, AgvStateStr[it->first].c_str());
  }
}

int StateMachine::getCurState()
{
  return m_current_state_id;
}

int StateMachine::getLateState()
{
  return m_prvious_state_id;
}

void StateMachine::recvAdStatusCallback(const status_msgs::ADStatus::ConstPtr &msg)
{
  if (p_current_state != NULL && p_current_state->status_ == 0)
  {
    //    p_current_state->status_ = 1;
    // ROS_INFO("during state %d %s recieve AD Status msg", m_current_state_id,
    // AgvStateStr[m_current_state_id].c_str());
    AdStatusValue late_ad_status = cur_ad_status;

    cur_ad_status.agv_status            = msg->Agv_Status;            //整体状态
    cur_ad_status.node_Status           = msg->Node_Status;           //整体状态
    cur_ad_status.hardware_Status       = msg->Hardware_Status;       //整体状态
    cur_ad_status.planning_task_id      = msg->planning_task_ID;      //任务状态
    cur_ad_status.planning_task_status  = msg->planning_task_status;  //任务状态
    cur_ad_status.operation_task_id     = msg->operation_task_ID;     //任务状态
    cur_ad_status.operation_task_status = msg->operation_task_status; //任务状态
    cur_ad_status.exception_task_id     = msg->exception_task_ID;     //任务状态
    cur_ad_status.exception_task_status = msg->exception_task_status; //任务状态
    cur_ad_status.ad_enbale_status      = msg->ad_enbale_status;      // ad模式

    //节点错误
    cur_ad_status.node_error_Num = msg->Node_Error_Num;
    cur_ad_status.node_error_code.clear();
    if (cur_ad_status.node_error_Num > 0)
    {
      for (size_t i = 0; i < cur_ad_status.node_error_Num; i++)
      {
        std::string error_code_temp_ = msg->Node_Error_Code[i];
        cur_ad_status.node_error_code.push_back(error_code_temp_);
      }
    }
    //硬件错误
    cur_ad_status.hardware_error_num = msg->Hardware_Error_Num;
    cur_ad_status.hardware_error_code.clear();
    if (cur_ad_status.hardware_error_num > 0)
    {
      for (size_t i = 0; i < cur_ad_status.hardware_error_num; i++)
      {
        std::string error_code_temp_ = msg->Hardware_Error_Code[i];
        cur_ad_status.hardware_error_code.push_back(error_code_temp_);
      }
    }

    //重复性消息判断
    if (0 == p_current_state->isRepetitionAdStatus(late_ad_status, cur_ad_status))
    {
      ROS_WARN("During %s had recieve Dif AdStatus", AgvStateStr[m_current_state_id].c_str());
      ROS_INFO("agv %u node %u hardware %u planning %u %u exception %u %u operation %u %u", msg->Agv_Status,
               msg->Node_Status, msg->Hardware_Status, msg->planning_task_ID, msg->planning_task_status,
               msg->exception_task_ID, msg->exception_task_status, msg->operation_task_ID, msg->operation_task_status);
    }

    p_current_state->ad_status_tip_ = p_current_state->doAdStatusJudge(cur_ad_status);
    p_current_state->status_        = 1;

    if (p_current_state->ad_status_tip_ != m_current_state_id)
    {
      ROS_INFO("AD Status Next State is %s.", AgvStateStr[p_current_state->ad_status_tip_].c_str());
    }
  }
  else
  {
    if (p_current_state == NULL)
    {
      ROS_INFO("state is not init.");
    }
    if (p_current_state->status_ > 0)
    {
      ROS_INFO("state is mathing.");
    }
  }
}

void StateMachine::recvVmsControlCallback(const hmi_msgs::VMSControlAD::ConstPtr &msg)
{
  if (p_current_state != NULL && p_current_state->vms_status_ == 0)
  {
    // ROS_INFO("during state %d %s recieve VMS Control msg", m_current_state_id,
    // AgvStateStr[m_current_state_id].c_str());
    if (cur_hmi_control_value.task_id == msg->task_ID)
    {
      ROS_INFO("recieve same task %u", msg->task_ID);
      //检查该任务执行情况，未下发的任务按照新任务执行，已下发的任务返回执行状态。
    }
    else
    {
      cur_hmi_control_value.recieve_task_num     = taskTimerPlus();
      cur_hmi_control_value.task_id              = msg->task_ID;                 //任务序号
      cur_hmi_control_value.cmd_type             = msg->CMD_Type;                //命令
      cur_hmi_control_value.target_point.x       = msg->target_position.x;       //目标点
      cur_hmi_control_value.target_point.y       = msg->target_position.y;       //目标点
      cur_hmi_control_value.target_point.z       = 0;                            //目标点
      cur_hmi_control_value.target_point.heading = msg->target_position.heading; //目标点

      p_current_state->hmi_status_tip_ = p_current_state->doHmiControlJudge(cur_hmi_control_value); //判断
      p_current_state->vms_status_     = 1;
      ROS_WARN("%s recieve %d %u %u task,will tran to %s .", AgvStateStr[m_current_state_id].c_str(),
               cur_hmi_control_value.recieve_task_num, msg->task_ID, msg->CMD_Type,
               AgvStateStr[p_current_state->hmi_status_tip_].c_str());
    }
  }
  else
  {
    if (p_current_state == NULL)
    {
      ROS_INFO("state is not init.");
    }
    if (p_current_state->vms_status_ > 0)
    {
      ROS_INFO("state is mathing.");
    }
  }
}

int StateMachine::taskTimerPlus()
{
  return ++task_timer;
}

int StateMachine::vcuControlPub()
{
  hmi_msgs::FsmControVcuDriver vcu_control_temp;
  ros::Time current_time = ros::Time::now();

  vcu_control_temp.header.stamp    = current_time;
  vcu_control_temp.header.frame_id = "fsm2vcucontrol";
  vcu_control_temp.enable_node_num = node_enable_status.is_enable_fsm + node_enable_status.is_enable_planning +
                                     node_enable_status.is_enable_exception + node_enable_status.is_enable_operation;
  vcu_control_temp.is_enable_fsm       = node_enable_status.is_enable_fsm;
  vcu_control_temp.is_enable_planning  = node_enable_status.is_enable_planning;
  vcu_control_temp.is_enable_exception = node_enable_status.is_enable_exception;
  vcu_control_temp.is_enable_operation = node_enable_status.is_enable_operation;
  vcu_control_temp.is_AD_status        = node_enable_status.is_AD_status;

  vcu_control_pub_.publish(vcu_control_temp);
  // ROS_INFO("vcu_control_pub OK");
  return 1;
}

void StateMachine::pushNodeEnableValue(const u_int8_t &fsm_value_, const u_int8_t &planning_value_,
                                       const u_int8_t &operation_value_, const u_int8_t &exception_value_,
                                       const u_int8_t &ad_value_)
{
  node_enable_status.is_enable_fsm       = fsm_value_;
  node_enable_status.is_enable_planning  = planning_value_;
  node_enable_status.is_enable_operation = operation_value_;
  node_enable_status.is_enable_exception = exception_value_;
  node_enable_status.is_AD_status        = ad_value_;
}

int StateMachine::getNodeEnableValue()
{
  return (( int )node_enable_status.is_enable_fsm * 1000 + ( int )node_enable_status.is_enable_planning * 100 +
          ( int )node_enable_status.is_enable_operation * 10 + ( int )node_enable_status.is_enable_exception * 1);
}

int StateMachine::getAdEnableValue()
{
  return node_enable_status.is_AD_status;
}

int StateMachine::taskControlPub(const int &fsm_control_, int state_)
{
  hmi_msgs::VMSControlAD task_temp_;
  ros::Time current_time = ros::Time::now();

  task_temp_.header.stamp            = current_time;
  task_temp_.task_ID                 = cur_hmi_control_value.task_id;
  task_temp_.CMD_Type                = cur_hmi_control_value.cmd_type;
  task_temp_.target_position.x       = cur_hmi_control_value.target_point.x;
  task_temp_.target_position.y       = cur_hmi_control_value.target_point.y;
  task_temp_.target_position.heading = cur_hmi_control_value.target_point.heading;
  task_temp_.fsm_control             = fsm_control_;

  if (state_ == PLANNING)
  {
    task_temp_.header.frame_id = "fsm2planning";
    task_temp_.receiver        = 2;
    task_temp_.message_num     = 0;
    planning_pub_.publish(task_temp_);
    //保存任务序列
    this->addTaskCache(task_temp_, cur_hmi_control_value.recieve_task_num);
    return cur_hmi_control_value.recieve_task_num;
  }
  else if (state_ == OPERATION)
  {
    task_temp_.header.frame_id = "fsm2operation";
    task_temp_.receiver        = 4;
    task_temp_.message_num     = 0;
    //发布任务序列
    operation_pub_.publish(task_temp_);
    //保存任务序列
    this->addTaskCache(task_temp_, cur_hmi_control_value.recieve_task_num);
    return cur_hmi_control_value.recieve_task_num;
  }
  else if (state_ == EXCEPTION)
  {
    task_temp_.header.frame_id = "fsm2exception";
    task_temp_.receiver        = 3;
    task_temp_.message_num     = cur_ad_status.node_error_Num + cur_ad_status.hardware_error_num;
    task_temp_.message_code.clear();

    if (cur_ad_status.node_error_Num > 0 && cur_ad_status.node_error_code.size() > 0)
    {
      for (size_t i = 0; i < cur_ad_status.node_error_Num; i++)
      {
        std::string error_code_temp_ = cur_ad_status.node_error_code[i];
        task_temp_.message_code.push_back(error_code_temp_);
      }
    }
    if (cur_ad_status.hardware_error_num > 0 && cur_ad_status.hardware_error_code.size() > 0)
    {
      for (size_t i = 0; i < cur_ad_status.hardware_error_num; i++)
      {
        std::string error_code_temp_ = cur_ad_status.hardware_error_code[i];
        task_temp_.message_code.push_back(error_code_temp_);
      }
    }
    //发布任务序列
    exception_pub_.publish(task_temp_);
    ROS_WARN("%s send id:%u cmd:%u OK", AgvStateStr[m_current_state_id].c_str(), task_temp_.task_ID,
             task_temp_.CMD_Type);
    //保存任务序列
    this->addTaskCache(task_temp_, cur_hmi_control_value.recieve_task_num);
    return cur_hmi_control_value.recieve_task_num;
  }
  else
  {
    return 0;
  }
}

void StateMachine::clearAllTaskCache()
{
  this->m_task_cache.clear();
  this->m_task_pub_cache.clear();
  this->curAdStatusValueInit();
  this->curHmiControlValueInit();
}

void StateMachine::showAllTaskPubCache()
{
  std::map< int, hmi_msgs::VMSControlAD >::iterator it;
  int loop_index = 0;
  for (it = m_task_pub_cache.begin(); it != m_task_pub_cache.end(); it++)
  {
    ++loop_index;
    ROS_WARN("Task pub cache %d %d id:%u cmd:%u fsm_c:%u re:%u ms:%u.", loop_index, it->first, it->second.task_ID,
             it->second.CMD_Type, it->second.fsm_control, it->second.receiver, it->second.message_num);
  }
}

void StateMachine::showAllTaskCache()
{
  // std::vector< TaskStatus > m_task_cache;
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    for (int i = 0; i < count_; i++)
    {
      TaskStatus &ts_ = m_task_cache.at(i);
      ROS_WARN("Task Cache rsn:%d id:%u cmd:%u re:%d rs:%d ss:%d send:%d", ts_.recieve_task_num, ts_.task_id,
               ts_.cmd_type, ts_.reciever, ts_.re_send_times, ts_.status, ts_.is_send_ok);
    }
  }
  else
  {
    ROS_WARN("Task Cache is empty");
  }
}

void StateMachine::showAllStateTranslate()
{
  std::map< int, StateStatus >::iterator it;
  int loop_index = 0;
  for (it = m_state_translate.begin(); it != m_state_translate.end(); it++)
  {
    ++loop_index;
    ROS_WARN("State Translate %d %d :%s  -->> %.8lf(s) -->>  %s", loop_index, it->first,
             AgvStateStr[it->second.cur_state].c_str(), it->second.time_out.toSec() - it->second.time_in.toSec(),
             AgvStateStr[it->second.next_state].c_str());
  }
}

int StateMachine::taskControlRePub(const int &recieve_task_num_)
{
  //查找任务发送序列中是否存在该任务id，存在则不保存，不存在则保存
  std::map< int, hmi_msgs::VMSControlAD >::iterator it = m_task_pub_cache.find(recieve_task_num_);
  if (it == m_task_pub_cache.end())
  {
    ROS_INFO("%s cannot find this %d task in cache", AgvStateStr[m_current_state_id].c_str(), recieve_task_num_);
    return 0;
  }
  else
  {
    hmi_msgs::VMSControlAD task_temp_;
    task_temp_ = it->second;
    switch (task_temp_.receiver)
    {
    case 2:
      planning_pub_.publish(task_temp_);
      updateCurCacheTaskResendTimes(recieve_task_num_);
      ROS_INFO("%s resend %d task ok", AgvStateStr[m_current_state_id].c_str(), recieve_task_num_);
      return recieve_task_num_;
      break;
    case 3:
      exception_pub_.publish(task_temp_);
      updateCurCacheTaskResendTimes(recieve_task_num_);
      ROS_INFO("%s resend %d task ok", AgvStateStr[m_current_state_id].c_str(), recieve_task_num_);
      return recieve_task_num_;
      break;
    case 4:
      operation_pub_.publish(task_temp_);
      updateCurCacheTaskResendTimes(recieve_task_num_);
      ROS_INFO("%s resend %d task ok", AgvStateStr[m_current_state_id].c_str(), recieve_task_num_);
      return recieve_task_num_;
      break;
    default:
      return 0;
    }
  }
}

void StateMachine::curAdStatusValueInit()
{
  cur_ad_status.agv_status            = 0; //整体状态
  cur_ad_status.node_Status           = 0; //整体状态
  cur_ad_status.hardware_Status       = 0; //整体状态
  cur_ad_status.planning_task_id      = 0; //任务状态
  cur_ad_status.planning_task_status  = 0; //任务状态
  cur_ad_status.operation_task_id     = 0; //任务状态
  cur_ad_status.operation_task_status = 0; //任务状态
  cur_ad_status.exception_task_id     = 0; //任务状态
  cur_ad_status.exception_task_status = 0; //任务状态
  cur_ad_status.ad_enbale_status      = 0; // ad模式
  cur_ad_status.node_error_Num        = 0;
  cur_ad_status.hardware_error_num    = 0;
}

void StateMachine::curHmiControlValueInit()
{
  cur_hmi_control_value.recieve_task_num     = 0;
  cur_hmi_control_value.task_id              = 0; //任务序号
  cur_hmi_control_value.cmd_type             = 0; //命令
  cur_hmi_control_value.target_point.x       = 0; //目标点
  cur_hmi_control_value.target_point.y       = 0; //目标点
  cur_hmi_control_value.target_point.z       = 0; //目标点
  cur_hmi_control_value.target_point.heading = 0; //目标点
}

int StateMachine::addTaskCache(const hmi_msgs::VMSControlAD &task_temp_, const int &rtn_)
{
  //查找任务发送序列中是否存在该任务id，存在则不保存，不存在则保存
  std::map< int, hmi_msgs::VMSControlAD >::iterator it = m_task_pub_cache.find(rtn_);
  if (it == m_task_pub_cache.end())
  {
    this->m_task_pub_cache.insert(std::pair< int, hmi_msgs::VMSControlAD >(rtn_, task_temp_));

    TaskStatus task_status_temp_;

    task_status_temp_.recieve_task_num = rtn_;
    task_status_temp_.task_id          = task_temp_.task_ID;
    task_status_temp_.cmd_type         = task_temp_.CMD_Type;
    task_status_temp_.reciever         = task_temp_.receiver;
    task_status_temp_.re_send_times    = 0;
    task_status_temp_.status           = 0;
    task_status_temp_.is_send_ok       = 0;
    task_status_temp_.result           = 0;

    this->m_task_cache.push_back(task_status_temp_);

    ROS_INFO("%s add %d task id:%u cmd:%u in cache %d", AgvStateStr[m_current_state_id].c_str(), rtn_,
             task_status_temp_.task_id, task_status_temp_.cmd_type, static_cast< int >(m_task_cache.size()));
    // this->showAllTaskPubCache();
    // this->showAllTaskCache();
    return 1;
  }
  else
  {
    ROS_WARN("The task cache had same task %d", rtn_);
    // this->showAllTaskPubCache();
    // this->showAllTaskCache();
    return 0;
  }
}

bool StateMachine::getTaskSendStatus(const int &recieve_task_num_)
{
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    for (int i = 0; i < count_; i++)
    {
      TaskStatus &task_status_temp_ = m_task_cache.at(i);
      if (recieve_task_num_ == task_status_temp_.recieve_task_num)
      {
        return task_status_temp_.is_send_ok;
      }
    }
  }
  return 0;
}

TaskStatus StateMachine::getCurTask()
{
  TaskStatus task_status_temp_;
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    for (int i = count_ - 1; i > -1; i--)
    {
      TaskStatus &tc_temp_ = m_task_cache.at(i);
      if (0 == tc_temp_.task_id && 0 == tc_temp_.cmd_type)
      {
        ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d is not VMS Control Order",
                 AgvStateStr[m_current_state_id].c_str(), tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result,
                 tc_temp_.status);
      }
      else
      {
        switch (tc_temp_.reciever)
        {
        case 2: // palnning
          if (tc_temp_.status < 3)
          {
            ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d", AgvStateStr[m_current_state_id].c_str(),
                     tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result, tc_temp_.status);
            return m_task_cache.at(i);
          }
          else
          {
            if (tc_temp_.result == 0)
            {
              ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d",
                       AgvStateStr[m_current_state_id].c_str(), tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result,
                       tc_temp_.status);
              return m_task_cache.at(i);
            }
          }
          break;
        case 3: // exception
          if (tc_temp_.status < 4)
          {
            ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d", AgvStateStr[m_current_state_id].c_str(),
                     tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result, tc_temp_.status);
            return m_task_cache.at(i);
          }
          else
          {
            if (tc_temp_.result == 0)
            {
              ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d",
                       AgvStateStr[m_current_state_id].c_str(), tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result,
                       tc_temp_.status);
              return m_task_cache.at(i);
            }
          }
          break;
        case 4: // operation
          if (tc_temp_.status < 3)
          {
            ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d", AgvStateStr[m_current_state_id].c_str(),
                     tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result, tc_temp_.status);
            return m_task_cache.at(i);
          }
          else
          {
            if (tc_temp_.result == 0)
            {
              ROS_INFO("%s FSM getCurTask recieve %d id %u result %d status %d",
                       AgvStateStr[m_current_state_id].c_str(), tc_temp_.reciever, tc_temp_.task_id, tc_temp_.result,
                       tc_temp_.status);
              return m_task_cache.at(i);
            }
          }
          break;
        default:
          break;
        }
      }
    }
  }

  task_status_temp_.recieve_task_num = 0;
  task_status_temp_.task_id          = 0;
  task_status_temp_.cmd_type         = 0;
  task_status_temp_.reciever         = 0;
  task_status_temp_.re_send_times    = 0;
  task_status_temp_.status           = 0;
  task_status_temp_.is_send_ok       = 0;

  ROS_INFO("%s FSM getCurTask recieve %d taskid %d m_task_cache %d retutn NULL",
           AgvStateStr[m_current_state_id].c_str(), task_status_temp_.reciever, task_status_temp_.task_id, count_);

  return task_status_temp_;
}

int StateMachine::getCurTaskSendID()
{
  TaskStatus cur_task_status_temp_ = this->getCurTask();
  return cur_task_status_temp_.recieve_task_num;
}

int StateMachine::getCurTaskID()
{
  TaskStatus cur_task_status_temp_ = this->getCurTask();
  return cur_task_status_temp_.task_id;
}

bool StateMachine::getCurTaskSendStatus()
{
  TaskStatus cur_task_status_temp_ = this->getCurTask();
  return cur_task_status_temp_.is_send_ok;
}

int StateMachine::getCurTaskStatus()
{
  TaskStatus cur_task_status_temp_ = this->getCurTask();
  return cur_task_status_temp_.status;
}

int StateMachine::updateTaskCache()
{
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    int update_task_num = 0;
    // this->showAllTaskCache();
    ROS_INFO("%s Cur agv %u node %u hardware %u planning %u %u exception %u %u operation %u %u",
             AgvStateStr[m_current_state_id].c_str(), cur_ad_status.agv_status, cur_ad_status.node_Status,
             cur_ad_status.hardware_Status, cur_ad_status.planning_task_id, cur_ad_status.planning_task_status,
             cur_ad_status.exception_task_id, cur_ad_status.exception_task_status, cur_ad_status.operation_task_id,
             cur_ad_status.operation_task_status);

    for (int i = 0; i < count_; i++)
    {
      if (0 == m_task_cache.at(i).task_id && 0 == m_task_cache.at(i).cmd_type)
      {
      }
      else
      {
        if (m_task_cache.at(i).is_send_ok == 0)
        {
          if (m_task_cache.at(i).task_id == cur_ad_status.planning_task_id &&
              m_task_cache.at(i).reciever == 2) //          PLANNING
          {
            // m_task_cache.at(i).status     = cur_ad_status.planning_task_status;
            m_task_cache.at(i).is_send_ok = 1;
            ROS_WARN("%s update Send Status PLANNING %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else if (m_task_cache.at(i).task_id == cur_ad_status.operation_task_id &&
                   m_task_cache.at(i).reciever == 4) // OPERATION
          {
            // m_task_cache.at(i).status     = cur_ad_status.operation_task_status;
            m_task_cache.at(i).is_send_ok = 1;
            ROS_WARN("%s update Send Status OPERATION %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else if (m_task_cache.at(i).task_id == cur_ad_status.exception_task_id &&
                   m_task_cache.at(i).reciever == 3) // EXCEPTION
          {
            // m_task_cache.at(i).status     = cur_ad_status.exception_task_status;
            m_task_cache.at(i).is_send_ok = 1;
            ROS_WARN("%s update Send Status EXCEPTION %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else
          {
          }
        }

        if (m_task_cache.at(i).is_send_ok == 1)
        {
          if (m_task_cache.at(i).task_id == cur_ad_status.planning_task_id && m_task_cache.at(i).reciever == 2 &&
              m_task_cache.at(i).result == 0) //          PLANNING
          {
            m_task_cache.at(i).status = cur_ad_status.planning_task_status;
            ROS_WARN("%s update Task Result PLANNING %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else if (m_task_cache.at(i).task_id == cur_ad_status.operation_task_id && m_task_cache.at(i).reciever == 4 &&
                   m_task_cache.at(i).result == 0) // OPERATION
          {
            m_task_cache.at(i).status = cur_ad_status.operation_task_status;
            ROS_WARN("%s update Task Result OPERATION %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else if (m_task_cache.at(i).task_id == cur_ad_status.exception_task_id && m_task_cache.at(i).reciever == 3 &&
                   m_task_cache.at(i).result == 0) // EXCEPTION
          {
            m_task_cache.at(i).status = cur_ad_status.exception_task_status;
            ROS_WARN("%s update Task Result EXCEPTION %d task id:%u cmd:%u status to %d",
                     AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num,
                     m_task_cache.at(i).task_id, m_task_cache.at(i).cmd_type, m_task_cache.at(i).status);
            ++update_task_num;
          }
          else
          {
          }
        }
      }
      ROS_INFO("%s Task rsn:%d id:%u cmd:%u rer:%d rst:%d ss:%d send:%d rsu:%d",
               AgvStateStr[m_current_state_id].c_str(), m_task_cache.at(i).recieve_task_num, m_task_cache.at(i).task_id,
               m_task_cache.at(i).cmd_type, m_task_cache.at(i).reciever, m_task_cache.at(i).re_send_times,
               m_task_cache.at(i).status, m_task_cache.at(i).is_send_ok, m_task_cache.at(i).result);
    }
    return update_task_num; //更新状态数量
  }
  return count_;
}

int StateMachine::updateCurCacheTaskResult(const int &recieve_task_num_)
{
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    for (int i = 0; i < count_; i++)
    {
      if (recieve_task_num_ == m_task_cache.at(i).recieve_task_num)
      {
        m_task_cache.at(i).result = 1;
        ROS_WARN("Update CurTask recieve %d id %u result %d status %d", m_task_cache.at(i).reciever,
                 m_task_cache.at(i).task_id, m_task_cache.at(i).result, m_task_cache.at(i).status);
        return 1;
      }
    }

    ROS_WARN("resend %d task is not during task cache", recieve_task_num_);
    return 0;
  }
  else
  {
    ROS_WARN("task cache is empty");
    return count_;
  }
}

int StateMachine::updateCurCacheTaskResendTimes(const int &recieve_task_num_)
{
  int count_ = this->m_task_cache.empty() ? -1 : static_cast< int >(this->m_task_cache.size());
  if (count_ > 0)
  {
    for (int i = 0; i < count_; i++)
    {
      if (recieve_task_num_ == m_task_cache.at(i).recieve_task_num)
      {
        m_task_cache.at(i).re_send_times += 1;
        ROS_INFO("resend %d task", recieve_task_num_);
        return 1;
      }
    }

    ROS_INFO("resend %d task is not during task cache", recieve_task_num_);
    return 0;
  }
  else
  {
    ROS_INFO("task cache is empty");
    return count_;
  }
}

int StateMachine::fsmStatusPub()
{
  //当前状态，当前状态持续时间，上一个状态,当前任务，故障数及列表
  return 1;
}

} // namespace fsm
} // namespace superg_agv
