#ifndef FSM_INCLUDE_STATE_TRANSLATE_H_
#define FSM_INCLUDE_STATE_TRANSLATE_H_

#include "state_define.h"

namespace superg_agv
{
namespace fsm
{

// NULL_STATE_ID = 0,
//     STARTUP,          //
//     STANDBY,          //空闲状态 车辆处于远程自由状态，当前无任务，随时准备动作
//     NORMAL_OPERATION, //正常运行状态 车辆正在执行FMS任务，无异常产生
//     EXCEPTION_HANDLING, //故障处理状态 车辆存在故障，无法正常进行作业，车辆进行滑停或急停操作
//     MANUAL,             //手动操作状态 车辆处于手动操作状态
//     TASK_STAND_STILL, //停稳 任务完成后，车辆保持停止状态
//     TASK_MOVING,      //运动控制 得到有效的轨迹后，进入车辆控制状态
//     TASK_LIFT_UP,     //顶升上升 执行顶升上升任务
//     TASK_LIFT_DOWN,   //顶升下降 执行顶升下降任务
//     TASK_CHARGE,      //充电 执行充电任务
//     EXCEPTION_CONTROLLED_STOP, //车辆滑停 车辆仍然按当前参考路径进行运动控制，但会以最大的减速度减速停车
//     EXCEPTION_EMERGENCY_STOP, //车辆急停 PNC向VCU下发一个紧停指令，由VCU负责以最快速度停车
//     EXCEPTION_LATCHED,        //故障锁定 车辆保持异常停车状态

// enum AgvTransition
// {
//   NULL_STARTUP_TRANSITION = 0,
//   STARTUP_TO_STANDBY,
//   STARTUP_TO_EXCEPTION_HANDLING,
//   STARTUP_TO_MANUAL,
//   STARTUP_TO_STARTUP,
//   STANDBY_TO_NORMAL_OPERATION,
//   STANDBY_TO_EXCEPTION_HANDLING,
//   STANDBY_TO_MANUAL,
//   STANDBY_TO_STANDBY,
//   MANUAL_TO_EXCEPTION_HANDLING,
//   MANUAL_TO_STANDBY,
//   MANUAL_TO_MANUAL,
//   TASK_TO_STANDBY,
//   TASK_TO_EXCEPTION_HANDLING,
//   TASK_TO_MANUAL,
//   TASK_TO_TASK,
//   EXCEPTION_TO_STARTUP,
// };

int getNextState(int state_id_last, int translate_id)
{
  int state_id_next = 0;
  switch (state_id_last)
  {
  case NULL_STATE_ID:
    state_id_next = nullTranslate(translate_id);
    break;
  case STARTUP:
    state_id_next = startupTranslate(translate_id);
    break;
  case STANDBY:
    break;
  case NORMAL_OPERATION:
    break;
  case EXCEPTION_HANDLING:
    break;
  case MANUAL:
    break;
  case TASK_STAND_STILL:
    break;
  case TASK_MOVING:
    break;
  case TASK_LIFT_UP:
    break;
  case TASK_CHARGE:
    break;
  case EXCEPTION_CONTROLLED_STOP:
    break;
  case EXCEPTION_EMERGENCY_STOP:
    break;
  case EXCEPTION_LATCHED:
    break;
  default:
  }

  return state_id_next;
}

int nullTranslate(int translate_id_)
{
  return NULL_STATE_ID;
}

int startupTranslate(int translate_id_)
{
  switch (translate_id_)
  {
  case STARTUP_TO_STANDBY:
    return STANDBY;
  case STARTUP_TO_EXCEPTION_HANDLING:
    return EXCEPTION_HANDLING;
  case STARTUP_TO_MANUAL:
    return MANUAL;
  case STARTUP_TO_STARTUP:
    return STARTUP;
  default:
    return NULL_STATE_ID;
  }
}

} // namespace fsm
} // namespace superg_agv

#endif
