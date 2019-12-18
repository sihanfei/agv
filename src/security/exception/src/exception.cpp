

#include "exception.h"

namespace exception
{

Exception::Exception(ros::NodeHandle &node, std::string node_name) : node_name_(node_name)
{
  // GLogHelper gh(argv[0]);
  // gh.setLogDirectory("/work/log/exception");

  //初始化变量
  varInit();

  //载入故障列表
  if (loadExceptionList())
  {
    ROS_INFO_STREAM("Successfully loaded the exception list...");
  }
  else
  {
    ROS_WARN_STREAM("Failed to load exception list...");
  }

  //接收FSM消息 hmi_msgs
  FSMControlAD_sub_ = node.subscribe("/fsm/to_exception_order", 10, &Exception::exceptionCallBackFSM, this);
  //接收VCU消息 control_msgs
  AGVStatus_sub_ = node.subscribe("/drivers/com2agv/agv_status", 10, &Exception::exceptionCallBackVCU, this);

  //发布status消息 status_msgs
  exception_status_pub_ = node.advertise< exception::NodeStatus >("/exception/exception_status", 10);
  //发布消息到CAN control_msgs
  ADControlAGV_pub_ = node.advertise< exception::ADControlAGV >("/control/control_agv", 10);

  exception();
}

void Exception::varInit()
{
  current_exception_id_ = "0";
  stop_flag_            = 0;
  task_flag_            = 0;
  FSM_control_          = 0;
  FSM_work_mode_        = 0;
  FSM_exception_id_     = "0";
  FSM_msg_flag_         = false;
  VCU_veh_spd_          = 0.0;
  VCU_estop_status_     = false;
  VCU_msg_flag_         = false;
  estop_reset_permit_   = true;
  task_reset_permit_    = true;
  reset_flag_           = false;
  current_FSM_control_  = 0;
  current_work_mode_    = 0;
  estop_                = false;

  exception_task_ID_value_      = 0;
  exception_task_status_value_  = 0;
  Exception_Status_value_       = 0;
  Exception_Type_value_         = 0;
  Exception_Error_Ser_Num_value = "0";
  state_num_                    = 0;
  E08011003_flag_               = false;
}

void Exception::varReset()
{
  current_work_mode_            = 0;
  E08011003_flag_               = 0;
  exception_task_ID_value_      = 0;
  exception_task_status_value_  = 0;
  Exception_Status_value_       = 0;
  Exception_Type_value_         = 0;
  Exception_Error_Ser_Num_value = "0";
  VCU_msg_flag_                 = false;
  stop_flag_                    = 0;
  estop_                        = 0;
  estop_reset_permit_           = true;
  task_flag_                    = 0;
  current_FSM_control_          = 0;
  reset_flag_                   = 0;
}

bool Exception::loadExceptionList()
{
  //打开故障列表文件并检测是否成功
  std::ifstream exception_list_file;
  //改一下路径~~
  exception_list_file.open("/home/wrll/work/superg_agv/src/security/exception/data/exception_list.csv", std::ios::in);
  if (!exception_list_file.is_open())
  {
    ROS_ERROR("Failed to open file");
    E08011002();
    return false;
  }

  //截止到末尾行逐行读取，以“，”分隔
  while (!exception_list_file.eof())
  {
    std::string line, line_orig;
    while (getline(exception_list_file, line_orig))
    {
      line = "";
      for (char a : line_orig) //过滤非法字符
      {
        // if((a >= 65 && a <= 122) || (a >= 48 && a <= 57) || a == 44)
        if (a >= 0 && a <= 127)
        {
          line += a;
        }
      }
      std::istringstream sin(line);
      std::string info;
      std::vector< std::string > infos;
      while (getline(sin, info, ','))
      {
        infos.emplace_back(info);
      }
      std::stringstream s_exception_level, s_reset_permission;

      s_exception_level << infos[2];
      s_reset_permission << infos[4];

      ExceptionInfoItem exception_info_item;
      std::string exception_id                  = infos[1];
      exception_info_item.exception_description = infos[3];
      s_exception_level >> exception_info_item.exception_level;
      s_reset_permission >> exception_info_item.reset_permission;

#if (0)
      ROS_DEBUG_STREAM("exception_id : " << exception_id);
      ROS_DEBUG_STREAM("exception_description : " << exception_info_item.exception_description);
      ROS_DEBUG_STREAM("exception_level : " << exception_info_item.exception_level);
      ROS_DEBUG_STREAM("s_reset_permission : " << exception_info_item.reset_permission);
#endif

      std::pair< std::string, ExceptionInfoItem > exception_info_item_pair(exception_id, exception_info_item);
      exception_info_item_map_.insert(exception_info_item_pair);
    }
  }
  //打印存入的id
  for (std::unordered_map< std::string, ExceptionInfoItem >::iterator it = exception_info_item_map_.begin();
       it != exception_info_item_map_.end(); it++)
  {
    ROS_INFO("id : %s", it->first.c_str());
  }

  return true;
}

void Exception::exceptionCallBackFSM(const exception::VMSControlAD &FSMControlAD_msg)
{
  if (FSMControlAD_msg.receiver == 3)
  {
    //接收任务类型 0 6 7 8 9 10 12
    FSM_work_mode_ = FSMControlAD_msg.CMD_TYpe;
    //接收状态机控制指令 0 1 2 3
    FSM_control_ = FSMControlAD_msg.fsm_control;

    //接收故障id
    if (FSMControlAD_msg.message_num == 0)
    {
    }
    else if (FSMControlAD_msg.message_num == 1)
    {
      FSM_exception_id_ = FSMControlAD_msg.message_code[0];
    }
    else
    {
      for (auto &msg_code : FSMControlAD_msg.message_code)
      {
        FSM_exception_ids_.emplace_back(msg_code);
      }
    }

    // FSM接收标识有效
    FSM_msg_flag_ = true;
  }
  else
  {
    FSM_msg_flag_ = false;
  }
}

void Exception::exceptionCallBackVCU(const exception::AGVStatus &AGVStatus_msg)
{
  // VCU实际速度
  VCU_veh_spd_ = AGVStatus_msg.ActualSpd;
  // VCU紧停状态
  VCU_estop_status_ = AGVStatus_msg.EStopStatus;
  // VCU接收标识有效
  VCU_msg_flag_ = true;
}

void Exception::exception()
{
  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    if (FSM_msg_flag_ == true)
    {
      exception_task_status_value_ = 1; //故障处理方式判断中
      Exception_Status_value_      = 1; //故障处理中
      switch (FSM_control_)
      {
      case 0: //空操作
        break;
      case 1: //执行本条信息中的CMD_Type
        executeTask(FSM_work_mode_);
        break;
      case 2: //取消本条信息中的CMD_Type
        cancelTask(current_work_mode_);
        break;
      case 3: //取消本节点当前任务
        cancelCurrentTask();
        break;
      default:
        ROS_INFO_STREAM("FSM_control_ command is invalid");
        if (!E08011003_flag_)
          E08011003();
        break;
      }
      publishVCUMsg();
      FSM_msg_flag_                = false;
      exception_task_status_value_ = 2;              //完成故障处理命令下发
      Exception_Status_value_      = 4;              //故障处理完成，重置到standby
      exception_task_ID_value_     = FSM_work_mode_; //故障处理节点返回当前执行或者最后执行的任务ID

      if (VCU_msg_flag_ == true)
      {
        feedbackProcessingState();
        publishStatusMsg();
        VCU_msg_flag_ = false;
      }
    }
    varReset();

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void Exception::executeTask(int &work_mode)
{
  switch (work_mode)
  {
  case 0: //处理故障
    triggerException(FSM_exception_id_);
    Exception_Type_value_ = 0;
    break;
  case 6: //紧停
    stop_flag_            = 2;
    estop_                = 1;
    estop_reset_permit_   = false;
    Exception_Type_value_ = 2;
    break;
  case 7: //复位
    resetException(current_exception_id_);
    Exception_Type_value_ = 3;
    break;
  case 8: //任务暂停
    task_flag_            = 1;
    Exception_Type_value_ = 4;
    break;
  case 9: //任务继续
    task_flag_            = 0;
    Exception_Type_value_ = 5;
    break;
  case 10: //任务取消
    task_flag_            = 2;
    Exception_Type_value_ = 6;
    break;
  case 12: //滑停
    stop_flag_            = 1;
    estop_                = 1;
    estop_reset_permit_   = false;
    Exception_Type_value_ = 1;
    break;
  default:
    ROS_ERROR_STREAM("FSMControlAD_msg->CMD_Type command is invalid");
    if (!E08011003_flag_)
      E08011003();
    break;
  }

  current_FSM_control_ = 1;
  current_work_mode_   = work_mode;
}

void Exception::cancelTask(int &work_mode)
{
  switch (work_mode)
  {
  case 0: //
    ROS_INFO_STREAM("Empty operation");
    break;
  case 6: //取消紧停
    stop_flag_          = 0;
    estop_              = 0;
    estop_reset_permit_ = true;
    break;
  case 7: //取消复位
    triggerException(current_exception_id_);
    break;
  case 8: //取消任务暂停
    task_flag_            = 0;
    Exception_Type_value_ = 5;
    break;
  case 9: //
    ROS_INFO_STREAM("Empty operation");
    break;
  case 10: //
    ROS_INFO_STREAM("Empty operation");
    break;
  case 12: //取消滑停
    stop_flag_          = 0;
    estop_              = 0;
    estop_reset_permit_ = true;
    break;
  default:
    ROS_ERROR_STREAM("FSMControlAD_msg->CMD_Type command is invalid");
    if (!E08011003_flag_)
      E08011003();
    break;
  }

  current_FSM_control_ = 0;
}

void Exception::cancelCurrentTask()
{
  switch (current_FSM_control_)
  {
  case 1:
    cancelTask(current_work_mode_);
    break;
  default:
    ROS_INFO_STREAM("No current task can be canceled");
    if (!E08011003_flag_)
      E08011003();
    break;
  }
}

void Exception::publishVCUMsg()
{
  exception::ADControlAGV AD_control_AGV;

  AD_control_AGV.header.stamp = ros::Time::now();
  // AD_control_AGV.header.frame_id = "exc";
  AD_control_AGV.sender = 3;
  AD_control_AGV.EStop  = estop_;

  ADControlAGV_pub_.publish(AD_control_AGV);
  return;
}

void Exception::publishStatusMsg()
{
  exception_node_status_.header.stamp = ros::Time::now();
  exception_node_status_.node_name    = node_name_;
  exception_node_status_.node_pid     = getpid();
  I08011001();
  exception_node_status_.state_num = state_num_;

  exception_status_pub_.publish(exception_node_status_);
  memset(&exception_node_status_, 0, sizeof(exception::NodeStatus));
  state_num_      = 0;
  E08011003_flag_ = false;
}

void Exception::I08011001()
{
  std::stringstream exception_task_ID_ss, exception_task_status_ss, Exception_Status_ss, Exception_Type_ss;
  exception_task_ID_ss << exception_task_ID_value_;
  exception_task_status_ss << exception_task_status_value_;
  Exception_Status_ss << Exception_Status_value_;
  Exception_Type_ss << Exception_Type_value_;

  exception::KeyValue exception_task_ID, exception_task_status, Exception_Status, Exception_Type,
      Exception_Error_Ser_Num;
  exception_task_ID.key       = "exception_task_ID";
  exception_task_ID.valuetype = 2;
  exception_task_ID_ss >> exception_task_ID.value;

  exception_task_status.key       = "exception_task_status";
  exception_task_status.valuetype = 2;
  exception_task_status_ss >> exception_task_status.value;

  Exception_Status.key       = "Exception_Status";
  Exception_Status.valuetype = 2;
  Exception_Status_ss >> Exception_Status.value;

  Exception_Type.key       = "Exception_Type";
  Exception_Type.valuetype = 2;
  Exception_Type_ss >> Exception_Type.value;

  Exception_Error_Ser_Num.key       = "Exception_Error_Ser_Num";
  Exception_Error_Ser_Num.valuetype = 9;
  Exception_Error_Ser_Num.value     = Exception_Error_Ser_Num_value;

  exception::SafetyStatus safety_status;
  safety_status.message_code = "I08011001";
  safety_status.counter      = 0;
  safety_status.hardware_id  = 0;
  safety_status.value_num    = 5;
  safety_status.values.emplace_back(exception_task_ID);
  safety_status.values.emplace_back(exception_task_status);
  safety_status.values.emplace_back(Exception_Status);
  safety_status.values.emplace_back(Exception_Type);
  safety_status.values.emplace_back(Exception_Error_Ser_Num);

  exception_node_status_.status.emplace_back(safety_status);
  state_num_++;
}

void Exception::E08011002()
{
  exception::SafetyStatus safety_status;
  safety_status.message_code = "E08011002";
  safety_status.counter      = 0;
  safety_status.hardware_id  = 0;
  safety_status.value_num    = 0;

  exception_node_status_.status.emplace_back(safety_status);
  state_num_++;
}

void Exception::E08011003()
{
  std::stringstream FSM_control_ss, CMD_Type_ss, FSM_exception_id_ss;
  FSM_control_ss << FSM_control_;
  CMD_Type_ss << FSM_work_mode_;
  FSM_exception_id_ss << FSM_exception_id_;

  exception::KeyValue FSM_control, CMD_Type, FSM_exception_id;
  FSM_control.key       = "FSM_control";
  FSM_control.valuetype = 2;
  FSM_control_ss >> FSM_control.value;

  CMD_Type.key       = "CMD_Type";
  CMD_Type.valuetype = 2;
  CMD_Type_ss >> CMD_Type.value;

  FSM_exception_id.key       = "FSM_exception_id";
  FSM_exception_id.valuetype = 2;
  FSM_exception_id_ss >> FSM_exception_id.value;

  exception::SafetyStatus safety_status;
  safety_status.message_code = "E08011003";
  safety_status.counter      = 0;
  safety_status.hardware_id  = 0;
  safety_status.value_num    = 3;
  safety_status.values.emplace_back(FSM_control);
  safety_status.values.emplace_back(CMD_Type);
  safety_status.values.emplace_back(FSM_exception_id);

  exception_node_status_.status.emplace_back(safety_status);
  state_num_++;
  E08011003_flag_ = true;
}

//单故障情况
void Exception::triggerException(std::string &exception_id)
{
  if (exception_id == "0")
  {
    ROS_INFO_STREAM("no exception");
    return;
  }

  if (exception_info_item_map_.find(exception_id) == exception_info_item_map_.end())
  {
    ROS_ERROR("Failed to search triggered id : %s", exception_id.c_str());
    if (!E08011003_flag_)
      E08011003();
    return;
  }

  ROS_DEBUG("Successfully search triggered id : %s", exception_id.c_str());

  auto &got_exception   = exception_info_item_map_.at(exception_id);
  current_exception_id_ = exception_id; //记录当前故障编号

  if (estop_reset_permit_)
  {
    if (got_exception.exception_level == 2) // error
    {
      stop_flag_ = 1;
      estop_     = 1;
    }
    else if (got_exception.exception_level == 3) // fatal
    {
      stop_flag_ = 2;
      estop_     = 1;
    }
    else
    {
      stop_flag_ = 0;
      estop_     = 0;
    }
  }

  if (task_reset_permit_)
  {
    if (got_exception.exception_level == 2 || got_exception.exception_level == 3) // error
    {
      task_flag_ = 2;
    }
    else
    {
      task_flag_ = 0;
    }
  }

  //故障内容显示
  int exce_level        = got_exception.exception_level;
  std::string exce_desc = got_exception.exception_description;

  if (exce_level == 0)
  {
    ROS_INFO_STREAM(exce_desc);
    // SUPERG_INFO << exce_desc << " " << recordTime();
  }
  else if (exce_level == 1)
  {
    ROS_WARN_STREAM(exce_desc);
    // SUPERG_WARN << exce_desc << " " << recordTime();
  }
  else if (exce_level == 2)
  {
    ROS_ERROR_STREAM(exce_desc);
    // SUPERG_ERROR << exce_desc << " " << recordTime();
  }
  else
  {
    ROS_FATAL_STREAM(exce_desc);
    // SUPERG_FATAL << exce_desc << " " << recordTime();
  }

  return;
}

void Exception::resetException(std::string &exception_id)
{
  if (exception_id == "0")
  {
    ROS_INFO_STREAM("No triggered exception, reset command is invalid");
    return;
  }

  //当前记录的故障编号在故障表里找不到
  if (exception_info_item_map_.find(exception_id) == exception_info_item_map_.end())
  {
    ROS_INFO("The exception id is invalid...");
    return;
  }

  auto &got_exception = exception_info_item_map_.at(exception_id);

  if (got_exception.reset_permission == true) //该故障允许复位
  {
    stop_flag_  = 0; //停车解除
    estop_      = 0;
    task_flag_  = 0; //任务继续
    reset_flag_ = 1;
    ROS_INFO_STREAM("reset " << exception_id);
    // SUPERG_INFO << "reset " << exception_id << " " << recordTime();
  }
  else //维持原本状态
    ROS_INFO_STREAM(exception_id << " is no reset allowed");

  return;
}

std::string Exception::recordTime()
{
  time_t now = time(0); // 基于当前系统的当前日期/时间
  tm *ltm    = localtime(&now);

  char ihour[50], imin[50], isec[50];
  sprintf(ihour, "%02d", ltm->tm_hour);
  sprintf(imin, "%02d", ltm->tm_min);
  sprintf(isec, "%02d", ltm->tm_sec);

  std::vector< std::string > sTime{ihour, imin, isec};

  std::string myTime = boost::algorithm::join(sTime, ":");

  return myTime;
}

void Exception::feedbackProcessingState()
{
  if (estop_ == 1) //需要停车
  {
    if (VCU_estop_status_ == 1) // VCU紧停指令收到
    {
      if (VCU_veh_spd_ == 0 && reset_flag_ == 0) //停下且未复位
      {
        exception_task_status_value_ = 4; //故障完成，车辆保持停止，保持Exception状态
        Exception_Status_value_      = 2; //故障处理完成，停稳
      }
      else if (VCU_veh_spd_ == 0 && reset_flag_ == 1) //停下且复位
      {
        exception_task_status_value_ = 5; //故障完成，切换startup
        Exception_Status_value_      = 4; //故障处理完成，重置到standby
      }
      else if (VCU_veh_spd_ != 0 && reset_flag_ == 0) //未停下且未复位
      {
        exception_task_status_value_ = 3; //等待故障处理结果
        Exception_Status_value_      = 3; //故障未处理完成，故障锁定
      }
      else //未停下且复位
      {
        exception_task_status_value_ = 9; //故障处理中止，按照命令，切换startup
        Exception_Status_value_      = 5; //故障未处理完成，重置到standby
      }
    }
    else
    {
      if (reset_flag_ == 0)
      {
        exception_task_status_value_ = 8; //故障处理失败，命令未生效
      }
      else
      {
        exception_task_status_value_ = 7; //故障处理失败，切换startup
      }
    }
  }

  Exception_Error_Ser_Num_value = current_exception_id_;
  return;
}

} // namespace exception

/*
//多故障情况
bool Exception::triggerExceptions(std::vector<std::string>& exception_ids)
{
  int count = exception_ids.size();
  for(int i=0; i<count; i++)
  {
    ROS_DEBUG("trigger id : %s", exception_ids[i].c_str());
    if(exception_info_item_map_.find(exception_ids[i]) == exception_info_item_map_.end())
    {
      ROS_ERROR_STREAM("Failed to search id" << ": " << exception_ids[i]);
      return false;
    }
  }
  ROS_DEBUG("Successfully searched all...");

  std::vector<int> stop_flags;
  for(int i=0; i<count; i++)
  {
    auto& got_exception = exception_info_item_map_.at(exception_ids[i]);
    int stop_flag = got_exception.vehicle_behavior;
    stop_flags.emplace_back(stop_flag);
  }

  std::vector<int>::iterator max_stop_flag = max_element(stop_flags.begin(), stop_flags.end());
  int index = max_stop_flag - stop_flags.begin();

  current_exception_id_ = exception_ids[index];//记录当前故障编号
  auto& got_max_exception = exception_info_item_map_.at(current_exception_id_);
  stop_flag_ = got_max_exception.vehicle_behavior;//停车标识
  task_flag_ = got_max_exception.task_state;//任务标识

  //故障内容显示
  int exce_level = got_max_exception.exception_level;
  std::string exce_desc = got_max_exception.exception_description;

  if(exce_level == 0){
    ROS_WARN_STREAM(exce_desc);
    //SUPERG_WARN << exce_desc << " " << recordTime();
  } else if(exce_level == 1) {
    ROS_ERROR_STREAM(exce_desc);
    //SUPERG_ERROR << exce_desc << " " << recordTime();
  } else {
    ROS_FATAL_STREAM(exce_desc);
    //SUPERG_FATAL << exce_desc << " " << recordTime();
  }

  return true;
}

bool Exception::bypassException()
{
  //当前记录的故障编号在故障表里找不到
  if(exception_info_item_map_.find(current_exception_id_) == exception_info_item_map_.end())
  {
    ROS_INFO("The exception has not been triggered, the bypass is invalid...");
    return false;
  }
  auto& got_exception = exception_info_item_map_.at(current_exception_id_);
  std::string msg;

  if(got_exception.bypass_permission == true)//该故障允许旁路
  {
    stop_flag_ = 0;//停车解除
    task_flag_ = true;//任务继续

    got_exception.vehicle_behavior = 0;//该故障永久停车解除
    got_exception.task_state = true;//该故障永久任务继续
    msg = "bypass " + current_exception_id_;
  }
  else//维持原本状态
  {
    msg = current_exception_id_ + " is no bypass allowed";
  }

  ROS_INFO_STREAM(msg);
  //SUPERG_INFO << msg << " " << recordTime();

  return true;
}

bool Exception::clearAllExceptions()
{
  stop_flag_ = 0;//停车解除
  task_flag_ = 0;//任务继续

  if(exception_info_item_map_.empty() == true)
  {
    ROS_INFO("No exception load...");
    return false;
  }
  std::unordered_map<std::string, ExceptionInfoItem>::iterator iter_exc;
  for(iter_exc = exception_info_item_map_.begin(); iter_exc != exception_info_item_map_.end(); iter_exc++)
  {
    iter_exc->second.vehicle_behavior = 0;//所有故障永久停车解除
    iter_exc->second.task_state = true;//所有故障永久任务继续
  }
  std::string msg = "clear all exceptions...";
  ROS_INFO_STREAM(msg);
  //SUPERG_INFO << msg << " " << recordTime();

  return true;
}

bool Exception::restoreAllExceptions()
{
  exception_info_item_map_.erase(exception_info_item_map_.begin(), exception_info_item_map_.end());

  std::string msg;
  if(loadExceptionList())
  {
    msg = "Successfully restore All Exceptions!";
  }
  else
  {
    msg = "Failed restore All Exceptions!";
    //return false;
  }

  ROS_INFO_STREAM(msg);
  //SUPERG_INFO << msg << " " << recordTime();

  return true;
}

*/