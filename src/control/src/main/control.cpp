#include "control.h"

#define NODE_NAME "control"

using namespace std;
using namespace control;

int main(int argc, char **argv)
{
  ROS_INFO("%s is star, ros node name is "NODE_NAME, argv[0]);
  int mode_tip = 0;
  mode_tip = atoi(argv[1]);
  //检查输入参数，单个参数
  if(mode_tip != 1 && mode_tip != 2 && mode_tip != 3 && mode_tip != 4 && mode_tip != 5){
    ROS_INFO("The valid number is 1,2,3, Please input correct parameter!");
 //   exit(1);
    ros::shutdown();
  }
  else
  {
    //显示参数
    ROS_INFO("The mode tip is %s",argv[1]);
    //mode_tip = atoi(argv[1]);
  }

  //启动节点
  ros::init(argc, argv, "control");
  ros::NodeHandle n_sr;
  ros::NodeHandle n_rtk;
  ros::NodeHandle n_lf; //follow lattice path
  ros::NodeHandle n_gp; //follow global path
  ros::NodeHandle n_dg; //diagonal control

  if(mode_tip == 1){
    ROS_INFO("Goto Save Router.");
    SaveRoutePoint save_node(n_sr);
  }
  
  if(mode_tip == 2){
    ROS_INFO("Goto RTK Control.");
    RTKControl rtk_node(n_rtk);
  }
  
  if(mode_tip == 3){
	ROS_INFO("Goto Lattice Path Follow.");
    LFControl lf_control(n_lf);
  }

  if(mode_tip == 4){
	ROS_INFO("Goto Global Path Follow.");
    GPControl gp_control(n_gp);	
  }
    
  if(mode_tip == 5)
  {
    ROS_INFO("Goto Diagonal Control.");
    DIAGControl dg_control(n_dg);
  }
  //主程序休眠
  ros::spin();

}
