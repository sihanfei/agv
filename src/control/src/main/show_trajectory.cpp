#include "show_trajectory.h"
#include "STrajectory.h"

//#include "ros/ros.h"

using namespace std;
using namespace display;

int main(int argc,char **argv)
{

    ros::init(argc,argv,"show_trajectory");
    ros::NodeHandle n_st;

    STrajectory show_trajectory(n_st);

    ros::spin();
}
