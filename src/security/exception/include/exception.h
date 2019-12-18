#ifndef _EXCEPTION_H_
#define _EXCEPTION_H_

#include "ros/ros.h"
#include "ros/console.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <algorithm>
#include <unordered_map>
#include <ctime>
#include <boost/algorithm/string.hpp>
#include <unistd.h>
#include <pthread.h>
#include <cstring>
//#include "glog_helper.h"

//#include "std_msgs/UInt8.h"
//#include "std_msgs/String.h"
//#include "std_msgs/Bool.h"

#include "exception/KeyValue.h"
#include "exception/SafetyStatus.h"
#include "exception/ADControlAGV.h"
#include "exception/AGVStatus.h"
#include "exception/NodeStatus.h"
#include "exception/VMSControlAD.h"




namespace exception{

struct ExceptionInfoItem
{
  std::string exception_description;//故障描述
  int exception_level;//故障等级 0:Infomation 1：Warning 2：Error 3：Fatal
  bool reset_permission;//可否复位 true 1：可以 false 0：不可以
};


class Exception
{
  public:
    Exception(ros::NodeHandle &node, std::string node_name);
    ~Exception() = default;
    //载入故障列表
    bool loadExceptionList();

  private:
    ros::Subscriber FSMControlAD_sub_;
    ros::Subscriber AGVStatus_sub_;
    ros::Publisher exception_status_pub_;
    ros::Publisher ADControlAGV_pub_;
    //故障信息map
    std::unordered_map<std::string, ExceptionInfoItem> exception_info_item_map_;
  	//当前正在触发的故障id
  	std::string current_exception_id_;

	  //当前车辆运行状态 0:行进 1:滑停 2：急停
    int stop_flag_;
	  //当前任务执行状态 0:继续 1:任务暂停 2:任务中断
    int task_flag_;

    /** FSM指令 **/
    int FSM_control_;
    int FSM_work_mode_;
    std::string FSM_exception_id_;
    std::vector<std::string> FSM_exception_ids_;
    bool FSM_msg_flag_;

    /** VCU消息 **/
    double VCU_veh_spd_;
    bool VCU_estop_status_;//0：无紧停，1：启动紧停
    bool VCU_msg_flag_;

    //紧急停车复位许可
    bool estop_reset_permit_;
    //任务状态更改许可
    bool task_reset_permit_;
    //复位标识
    bool reset_flag_;
    //当前任务
    int current_FSM_control_;
    int current_work_mode_;

    bool estop_;

    /** 下发的状态值 **/
    //当前执行或者最后执行的任务ID
    int exception_task_ID_value_;
    //当前执行的任务状态 0-11
    int exception_task_status_value_;
    //故障处理状态 0-5
    int Exception_Status_value_;
    //故障处理类型 0-6
    int Exception_Type_value_;
    //当前处理的故障代码，空闲状态为0
    std::string Exception_Error_Ser_Num_value;

    //状态数量
    int state_num_;

    //
    exception::NodeStatus exception_node_status_;

    //true:已触发过
    bool E08011003_flag_;
    std::string node_name_;

    void varInit();
    void varReset();
    void exceptionCallBackFSM(const exception::VMSControlAD &FSMControlAD_msg);
    void exceptionCallBackVCU(const exception::AGVStatus &AGVStatus_msg);
    void exception();
    void executeTask(int& work_mode);
    void cancelTask(int& work_mode);
    void cancelCurrentTask();
    void publishVCUMsg();
    void publishStatusMsg();
    void I08011001();
    void E08011002();
    void E08011003();
    //单故障情况
    void triggerException(std::string& exception_id);
    void resetException(std::string& exception_id);
    std::string recordTime();
    void feedbackProcessingState();
};


} // namespace exception


#endif
