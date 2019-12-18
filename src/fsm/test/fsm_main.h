#ifndef FSM_INCLUDE_FSM_MAIN_H_
#define FSM_INCLUDE_FSM_MAIN_H_

#include "ros/ros.h"

namespace superg_agv
{
namespace fsm
{

enum StateID
{
  NULL_STATE_ID,
};

enum AgvStateID
{
  STARTUP,          //
  STANDBY,          //空闲状态 车辆处于远程自由状态，当前无任务，随时准备动作
  NORMAL_OPERATION, //正常运行状态 车辆正在执行FMS任务，无异常产生
  EXCEPTION_HANDLING, //故障处理状态 车辆存在故障，无法正常进行作业，车辆进行滑停或急停操作
  MANUAL,             //手动操作状态 车辆处于手动操作状态
};

enum TaskStateID
{
  NULL_TASK_STATE_ID, //
  STAND_STILL,        //停稳 任务完成后，车辆保持停止状态
  MOVING,             //运动控制 得到有效的轨迹后，进入车辆控制状态
  LIFT_UP,            //顶升上升 执行顶升上升任务
  LIFT_DOWN,          //顶升下降 执行顶升下降任务
  CHARGE,             //充电 执行充电任务
};

enum ExceptionStateID
{
  NULL_EXCEPTION_STATE_ID,
  CONTROLLED_STOP, //车辆滑停 车辆仍然按当前参考路径进行运动控制，但会以最大的减速度减速停车
  EMERGENCY_STOP, //车辆急停 PNC向VCU下发一个紧停指令，由VCU负责以最快速度停车
  LATCHED,        //故障锁定 车辆保持异常停车状态
};

enum StartupTransition
{
  NULL_STARTUP_TRANSITION,
  STARTUP_TO_STANDBY,
  STARTUP_TO_EXCEPTION_HANDLING,
  STARTUP_TO_MANUAL,
  STARTUP_TO_STARTUP,
};

enum StandbyTransition
{
  NULL_STANDBY_TRANSITION,
  STANDBY_TO_NORMAL_OPERATION,
  STANDBY_TO_EXCEPTION_HANDLING,
  STANDBY_TO_MANUAL,
  STANDBY_TO_STANDBY,
};

enum ManualTransition
{
  NULL_MANUAL_TRANSITION,
  MANUAL_TO_EXCEPTION_HANDLING,
  MANUAL_TO_STANDBY,
  MANUAL_TO_MANUAL,
};

enum TaskTransition
{
  NULL_TASK_TRANSITION,
  TASK_TO_STANDBY,
  TASK_TO_EXCEPTION_HANDLING,
  TASK_TO_MANUAL,
  TASK_TO_TASK,
};

enum ExceptionTransition
{
  NULL_TRANSITION,
  CHANGE_STARTUP,
};

enum CarType
{
  AGV_NO1,
  VEH_H7,
};

class CarOwner
{
public:
  int id;
  CarType car_type;
}

class StatePar
{
public:
  int a;
}

class StateBase
{
public:
  int state_id;
  StateMachine machine;
  ros::NodeHandle n;

public:
  StateBase(int id, const ros::NodeHandle &nh)
  {
    this.state_id = id;
    this.n        = nh;
  }
  virtual void OnEnter(StatePar params_)
  {
  }
  virtual void OnStay(StatePar params_)
  {
  }
  virtual void OnExit(StatePar params_)
  {
  }
}

class StateMachine
{
  //用来存储当前机器所控制的所有状态
public:
  map< int, StateBase > m_StateCache;
  //定义上一个状态
  StateBase m_prviousState;
  //定义当前状态
  StateBase m_currentState;
  //机器初始化时，没有上一个状态
public:
  StateMachine(StateBase beginState)
  {
    m_prviousState = NULL;
    m_currentState = beginState;
    m_StateCache   = new map< int, StateBase >();
    //把状态添加到集合中
    AddState(beginState);
    m_currentState.OnEnter();
  }

  void AddState(StateBase state)
  {
    if (!m_StateCache.ContainsKey(state.ID))
    {
      m_StateCache.Add(state.ID, state);
      state.machine = this;
    }
  }

  //删除状态
  void deleteState(StateBase state)
  {
  }

  //获得状态
  void getState(StateBase state)
  {
    
  }

  //通过Id来切换状态
  void TranslateState(int id)
  {
    if (!m_StateCache.ContainsKey(id))
    {
      return;
    }
    m_prviousState = m_currentState;
    m_currentState = m_StateCache[id];
    m_currentState.OnEnter();
  }
  //状态保持
  void Update()
  {
    if (m_currentState != null)
    {
      m_currentState.OnStay();
    }
  }
};

class StateTemplate< T > : StateBase
{
public:
  T owner;
  int owner_id;
  //拥有者(范型)
public:
  StateTemplate(int id, T o) : id(0)
  {
    owner    = o;
    owner_id = id;
  }
};

class StartUpState : StateTemplate< CarOwner >
{
public:
  StartUpState(int id, CarOwner p)
  {
  }

}

enum TransitionID {
  NullTransitionID,
  Switch_On_Manual,  // 1 车辆处于非自动状态 VCU监控函数 车辆处于本地或待机模式
  Switch_Off_Manual, // 2 车辆处于自动状态 VCU监控函数 车辆处于自动模式，可远程使用
  Task_Cmd_Received, // 3 收到FMS的有效指令 调度指令解析 收到FMS的指令后，校验正确。
  Task_Null,         // 4 当前没有作业指令 车辆控制 所有任务都已经顺利完成
  Liftup_Job,        // 5 当前任务为举升上升 车辆任务序列函数 在AGV的任务序列中，当前尚未完成的任务为举升上升任务
  Liftup_OK,         // 6 举升到位 顶升系统监控函数 收到VCU发来的举升到位消息。
  Liftdown_Job,      // 7 当前任务为举升下降 车辆任务序列函数 在AGV的任务序列中，当前尚未完成的任务为举升下降任务
  Liftdown_OK,       // 8 降下到位 顶升系统监控函数 收到VCU发来的下降到位消息。
  Charge_Job,        // 9 当前任务为充电 车辆任务序列函数 在AGV的任务序列中，当前尚未完成的任务为充电任务
  Charge_OK,         // 10 充电完成 充电监控函数 收到VCU发来的充电断开反馈
  Moving_Job,        // 11 当前任务为行车 车辆任务序列函数 在AGV的任务序列中，当前尚未完成的任务为行车任务
  Moving_OK,         // 12 车辆移动到位 车辆控制 运动控制到达目标点并满足停车到位条件
  Estop,             // 13 急停故障 故障监控函数 车辆存在急停故障
  Runing_Estop,      // 14 车辆进行急停操作 车辆控制 响应急停故障
  Break,             // 15 滑停故障 故障监控函数 车辆存在滑停故障
  Runing_Break,      // 16 车辆进行滑停操作 车辆控制 响应滑停故障
  Reset,             // 17 重置 车辆任务序列函数 收到FMS的“重置”指令
  Bypass,            // 18 旁路 车辆任务序列函数 收到FMS的“旁路”指令
  Stop_ok,           // 19 车辆故障后停稳 车辆控制 异常后，车辆停车完成，操作中断完成
};

enum PNC_SF_Status // 1 PNC状态机运行结果 PNC状态机
{
  NullStatusID,
  Standby,        //空闲状态
  Standstill,     //任务执行完毕
  Moving,         //车辆控制阶段
  Liftup,         //顶升上升阶段
  Liftdown,       //顶升下降阶段
  Charge,         //充电阶段
  Manual,         //手动状态
  ControlledStop, //滑停阶段
  EmergencyStop,  //急停阶段
  Latched,        //故障锁定阶段
}

//状态类
// public virtual void OnEnter(params object[] args){}
// public virtual void OnStay(params object[] args){}
// public virtual void OnExit(params object[] args){}
// update

//初始状态
//启动  ->  检查其他模块   状态ok     -> Standby
//         等待  延时参数 状态error   -> ExceptionHandling
//         等待  延时参数  状态为手动  -> Manual

enum Transition
{
  NullTransition
};

class FSMmain
{
};

class FSMSystem
{
};

class FSMState
{
};

} // namespace fsm
} // namespace superg_agv

#endif