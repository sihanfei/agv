#ifndef FSM_INCLUDE_STATE_DEFINE_H_
#define FSM_INCLUDE_STATE_DEFINE_H_

#include <string>
#include <vector>

namespace superg_agv
{
namespace fsm
{
// #define _GLIBCXX_USE_CXX11_ABI 0
#define STATE_FRE 200

struct StateStatus
{
  ros::Time time_in;
  ros::Time time_out;
  int late_state;
  int cur_state;
  int next_state;
};

struct VcuControlNodeStatus
{
  u_int8_t is_enable_fsm;
  u_int8_t is_enable_planning;
  u_int8_t is_enable_exception;
  u_int8_t is_enable_operation;
  u_int8_t is_AD_status;
};

struct AdStatusValue
{
  u_int8_t agv_status;
  u_int8_t node_Status;
  u_int16_t node_error_Num;
  std::vector< std::string > node_error_code;
  u_int8_t hardware_Status;
  u_int16_t hardware_error_num;
  std::vector< std::string > hardware_error_code;
  u_int8_t planning_task_id;
  u_int8_t planning_task_status;
  u_int8_t operation_task_id;
  u_int8_t operation_task_status;
  u_int8_t exception_task_id;
  u_int8_t exception_task_status;
  u_int8_t ad_enbale_status;
};

struct Point3D
{
  double x;
  double y;
  double z;
  double heading;
};

struct HmiControlValue
{
  int recieve_task_num;
  u_int8_t task_id;
  u_int8_t cmd_type;
  Point3D target_point;
};

struct TaskStatus
{
  int recieve_task_num;
  u_int8_t task_id;
  u_int8_t cmd_type;
  int reciever;
  int re_send_times;
  int status;
  bool is_send_ok;
  int result;
};

enum AgvStateID
{
  NULL_STATE_ID = 0,
  STARTUP,   //
  STANDBY,   //空闲状态 车辆处于远程自由状态，当前无任务，随时准备动作
  PLANNING,  //正常运行状态 车辆正在执行FMS任务，无异常产生
  OPERATION, //非运动任务
  EXCEPTION, //故障处理状态 车辆存在故障，无法正常进行作业，车辆进行滑停或急停操作
  REMOTE,    //手动操作状态 车辆处于手动操作状态
};

const std::string AgvStateStr[7] = {
    "NULL_STATE_ID",
    "STARTUP",   //
    "STANDBY",   //空闲状态 车辆处于远程自由状态，当前无任务，随时准备动作
    "PLANNING",  //正常运行状态 车辆正在执行FMS任务，无异常产生
    "OPERATION", //正常运行状态 车辆正在执行FMS任务，无异常产生
    "EXCEPTION", //故障处理状态 车辆存在故障，无法正常进行作业，车辆进行滑停或急停操作
    "REMOTE"     //手动操作状态 车辆处于手动操作状态
};

enum AgvTransition
{
  NULL_STARTUP_TRANSITION = 0,

  STARTUP_TO_STANDBY,
  STARTUP_TO_EXCEPTION,
  STARTUP_TO_REMOTE,
  STARTUP_TO_STARTUP,

  STANDBY_TO_PlANNING,
  STANDBY_TO_OPERATION,
  STANDBY_TO_EXCEPTION,
  STANDBY_TO_REMOTE,
  STANDBY_TO_STANDBY,

  REMOTE_TO_EXCEPTION,
  REMOTE_TO_STARTUP,
  REMOTE_TO_REMOTE,

  PlANNING_TO_STANDBY,
  PlANNING_TO_EXCEPTION,
  PlANNING_TO_REMOTE,
  PlANNING_TO_OPERATION,
  PlANNING_TO_PlANNING,

  OPERATION_TO_STANDBY,
  OPERATION_TO_EXCEPTION,
  OPERATION_TO_REMOTE,
  OPERATION_TO_PlANNING,
  OPERATION_TO_OPERATION,

  EXCEPTION_TO_STARTUP,
  EXCEPTION_TO_REMOTE,
  EXCEPTION_TO_EXCEPTION,
};

enum AgvEvent
{
  NULL_EVENT = 0,

  AGV_AD_MODE,
  AGV_DEBUG_MODE,
  AGV_ERROR_MODE,
  AGV_REMOTE_MODE,

  NODE_ERROR,
  HARDWARE_ERROR,

  TASK_NOT_RECIEVE,
  TASK_OPERATION_ERROR,

  TASK_REMOTE_ORDER_RECEIVE,
  TASK_NORMAL_ORDER_RECEIVE,
  SYSTEM_RESET,
  STARTUP_WAIT_TIMEOUT,
  TASK_NORMAL_ORDER_WAIT_TIMEOUT,
  MONITOR_NODE_OK,
  MONITOR_NODE_OPERATION_ERROR,
  MONITOR_NODE_START_TIME_OUT_ERROR,
  PLANNING_OK,
  PLANNING_ERROR,
  TASK_NORMAL_ORDER_ERROR_TO_EXCEPTION,
  TASK_NORMAL_ORDER_ERROR_TO_STANDBY,
  TASK_NORMAL_ORDER_FINISH,
  TASK_NORMAL_ORDER_TIME_RESET,
};

} // namespace fsm
} // namespace superg_agv

#endif