#include "ros/ros.h"
#include "std_msgs/String.h"
#include <csignal>
#include <string>
#include <unistd.h>

using std::__cxx11::string;

void exit(int signum)
{
  printf("You choose to stop me.");
  exit(0);
}

string state;
class Idle
{

private:
  ros::NodeHandle n;
  string goal;
  ros::Subscriber sub_;

public:
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "IDLE")
    {
      goal = msg->data;
    }
  }

public:
  void init(const ros::NodeHandle &n1)
  {

    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Idle::callback, this);
  }
  string execute()
  {

    state = "IDLE";
    ros::Rate loop_rate(5);

    while (true)
    {

      ros::spinOnce();
      loop_rate.sleep();
      if (goal == "a")
      {
        ROS_INFO("Having a new goal %s", goal.c_str());
        goal = "";
        return "got_goal";
      }
      std::cout << "goal=" << goal << std::endl;
      ROS_INFO("In state Idle : %s", goal.c_str());
    }

    return "";
  }
};

class Planner
{
private:
  string global_plan;
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "PLANNER")
    {
      global_plan = msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Planner::callback, this);
  }
  string execute()
  {
    state = "PLANNER";
    ros::Rate loop_rate(5);
    while (true)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (global_plan == "a")
      {
        ROS_INFO("Having a new topological plan");
        global_plan = "";
        return "got_plan";
      }
      ROS_INFO("In state Planner: %s", global_plan.c_str());
    }
  }
};

class Approach_start
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  string succeeded_flag;

  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "APPROACH_START")
    {
      succeeded_flag = msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Approach_start::callback, this);
  }
  string execute()
  {
    state = "APPROACH_START";
    ros::Rate loop_rate(5);
    while (true)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (succeeded_flag == "a")
      {
        ROS_INFO("Arrived starting node");
        succeeded_flag = "";
        return "succeeded";
      }
      ROS_INFO("In state Approach_to_startnode");
    }
  }
};

class Approach_end
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  string succeeded_flag;
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "APPROACH_END")
    {
      succeeded_flag = msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Approach_end::callback, this);
  }
  string execute()
  {
    state = "APPROACH_END";
    ros::Rate loop_rate(5);
    while (true)
    {

      ros::spinOnce();
      loop_rate.sleep();
      if (succeeded_flag == "a")
      {
        ROS_INFO("Arrived ending node");
        succeeded_flag = "";
        return "reach_goal";
      }
      ROS_INFO("In state Approach_to_endnode");
    }
  }
};

class Repeat
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  string succeeded_flag;
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "REPEAT")
    {
      succeeded_flag = msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Repeat::callback, this);
  }
  string execute()
  {
    state = "REPEAT";
    ros::Rate loop_rate(5);
    while (true)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (succeeded_flag == "a")
      {
        ROS_INFO("Repeat the taught content");
        succeeded_flag = "";
        return "succeeded";
      }
      ROS_INFO("In state Repeat");
    }
  }
};

class Obstacle
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  string cleared_flag;
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "OBSTACLE_AVOID")
    {
      cleared_flag == msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Obstacle::callback, this);
  }
  string execute()
  {
    state = "OBSTACLE_AVOID";
    ros::Rate loop_rate(5);
    while (true)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (cleared_flag == "a")
      {
        ROS_INFO("Now free to go");
        cleared_flag = "";
        return "cleared";
      }
      ROS_INFO("In state Obstacle");
    }
  }
  void run()
  {
  }
};

class Resume
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_;

public:
  string resumed_flag;
  void callback(const std_msgs::String::ConstPtr &msg)
  {
    if (state == "RESUME")
    {
      resumed_flag = msg->data;
    }
  }
  void init(const ros::NodeHandle &n1)
  {
    n    = n1;
    sub_ = n.subscribe("Idle/goal", 1, &Resume::callback, this);
  }
  string execute()
  {
    state = "APPROACH_START";
    ros::Rate loop_rate(5);
    while (true)
    {
      ros::spinOnce();
      loop_rate.sleep();
      if (resumed_flag == "a")
      {
        ROS_INFO("Now resume to new goal");
        resumed_flag = "";
        return "resume_goal";
      }
      ROS_INFO("In state Resume");
    }
  }
};

void work(const ros::NodeHandle &n)
{
  static Idle tmp_idle;
  tmp_idle.init(n);

  static Planner tmp_planner;
  tmp_planner.init(n);

  static Approach_start tmp_approach_start;
  tmp_approach_start.init(n);

  static Approach_end tmp_approach_end;
  tmp_approach_end.init(n);

  static Repeat tmp_repeat;
  tmp_repeat.init(n);

  static Obstacle tmp_obstacle;
  tmp_obstacle.init(n);

  static Resume tmp_resume;
  tmp_resume.init(n);
  while (true)
  {

    string res;
    if (state == "IDLE")
    {
      res = tmp_idle.execute();
      if (res == "got_goal")
        state = "PLANNER";
    }
    else if (state == "PLANNER")
    {
      res = tmp_planner.execute();
      if (res == "got_plan")
        state = "APPROACH_START";
    }
    else if (state == "APPROACH_START")
    {
      res = tmp_approach_start.execute();
      if (res == "succeeded")
        state = "REPEAT";
      else if (res == "detect_obstacle")
        state = "detect_obstacle";
    }
    else if (state == "REPEAT")
    {
      res = tmp_repeat.execute();
      if (res == "succeeded")
        state = "APPROACH_END";
      else if (res == "detect_obstacle")
        state = "detect_obstacle";
    }
    else if (state == "APPROACH_END")
    {
      res = tmp_approach_end.execute();
      if (res == "reach_goal")
        state = "reach_goal";
      else if (res == "detect_obstacle")
        state = "detect_obstacle";
    }
    else if (state == "EXECUTE")
    {
      // res=???.execute();
      if (res == "reach_goal")
        state = "IDLE";
      else if (res == "detect_obstacle")
        state = "OBSTACLE_AVOID";
    }
    else if (state == "OBSTACLE_AVOID")
    {
      res = tmp_obstacle.execute();
      if (res == "cleared")
        state = "RESUME";
      else if (res == "stuck")
        state = "stuck";
    }
    else if (state == "RESUME")
    {
      res = tmp_resume.execute();
      if (res == "resume_goal")
        state = "PLANNER";
    }
  }
}

std::string goal;

void callback_Idle(const std_msgs::String::ConstPtr &msg)
{
  if (state == "IDLE")
  {
    goal = msg->data;
  }
}

int main(int argc, char **argv)
{
  signal(SIGINT, exit);
  signal(SIGTERM, exit);

  ros::init(argc, argv, "test_smach2_node");
  ros::NodeHandle n;

  state = "IDLE";
  work(n);

  return 0;
}
