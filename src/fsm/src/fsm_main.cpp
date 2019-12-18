#include "ros/ros.h"
#include "state_base_machine.h"
#include "state_define.h"
#include "state_exception.h"
#include "state_operation.h"
#include "state_planning.h"
#include "state_remote.h"
#include "state_standby.h"
#include "state_startup.h"

using namespace superg_agv::fsm;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "fsm_t");
  ros::NodeHandle n;

  static StateMachine *new_fsm = new StateMachine(n);
  // Task

  static StartUpState *temp_startup_state     = new StartUpState(STARTUP, new_fsm);
  static StandbyState *temp_standby_state     = new StandbyState(STANDBY, new_fsm);
  static RemoteState *temp_remote_state       = new RemoteState(REMOTE, new_fsm);
  static PlanningState *temp_planning_state   = new PlanningState(PLANNING, new_fsm);
  static OperationState *temp_operation_state = new OperationState(OPERATION, new_fsm);
  static ExceptionState *temp_exception_state = new ExceptionState(EXCEPTION, new_fsm);

  new_fsm->addState(temp_startup_state);
  new_fsm->addState(temp_standby_state);
  new_fsm->addState(temp_remote_state);
  new_fsm->addState(temp_planning_state);
  new_fsm->addState(temp_operation_state);
  new_fsm->addState(temp_exception_state);

  new_fsm->showAllState();

  int cur_state_temp  = STARTUP;
  int next_state_temp = STARTUP;
  //状态切换计数器
  int state_translate_num = 0;
  StateStatus state_translate_status;

  state_translate_status.time_in    = ros::Time::now();
  state_translate_status.time_out   = ros::Time::now();
  state_translate_status.late_state = NULL_STATE_ID;
  state_translate_status.cur_state  = cur_state_temp;
  state_translate_status.next_state = next_state_temp;

  ros::Rate loop_rate(STATE_FRE);
  while (ros::ok())
  {
    next_state_temp = new_fsm->translateState(cur_state_temp);
    //    new_fsm->doStateMachineSpinOnce();
    if (next_state_temp == cur_state_temp)
    {
      loop_rate.sleep();
    }
    else
    {
      ++state_translate_num;

      state_translate_status.time_out   = ros::Time::now();
      state_translate_status.cur_state  = cur_state_temp;
      state_translate_status.next_state = next_state_temp;

      new_fsm->addTranslate(state_translate_num, state_translate_status);
      ROS_INFO("%s  -->> %.8lf(s) -->>  %s", AgvStateStr[state_translate_status.cur_state].c_str(),
               state_translate_status.time_out.toSec() - state_translate_status.time_in.toSec(),
               AgvStateStr[state_translate_status.next_state].c_str());

      state_translate_status.time_in    = ros::Time::now();
      state_translate_status.late_state = cur_state_temp;
      cur_state_temp                    = next_state_temp;
    }
  }
  ROS_INFO("shutting down!\n");
  ros::shutdown();
}