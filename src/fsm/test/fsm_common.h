#ifndef FSM_INCLUDE_FSM_COMMON_H_
#define FSM_INCLUDE_FSM_COMMON_H_

#include "ros/ros.h"

namespace superg_agv
{
namespace fsm
{

class StateBase
{
  //给每个状态设置一个ID
public:
  int ID;
  //被当前机器所控制
  StateMachine machine;
  ros::NodeHandle n;

public:
  StateBase(int id)
  {
    this.ID = id;
  };

  virtual void OnEnter(params object[] args)
  {
  }

  virtual void OnStay(params object[] args)
  {
  }
  
  virtual void OnExit(params object[] args)
  {
  }
};

class StateTemplate< T > : StateBase
{
public:
  T owner;
  //拥有者(范型)
public:
  StateTemplate(int id, T o) : base(id)
  {
    owner = o;
  }
};

class StateMachine
{
  //用来存储当前机器所控制的所有状态
public:
  Dictionary< int, StateBase > m_StateCache;
  //定义上一个状态
  StateBase m_prviousState;
  //定义当前状态
  StateBase m_currentState;
  //机器初始化时，没有上一个状态
public:
  StateMachine(StateBase beginState)
  {
    m_prviousState = null;
    m_currentState = beginState;
    m_StateCache   = new Dictionary< int, StateBase >();
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

} // namespace fsm
} // namespace superg_agv

#endif