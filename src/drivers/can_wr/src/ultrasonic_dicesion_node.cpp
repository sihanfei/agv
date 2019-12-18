#include "ultrasonic_dicesion.h"
#include <thread>

using namespace superg_agv;
using namespace std;


int main(int argc,char* argv[])
{
    ros::init(argc, argv, "ultrasonic_dicesion");
    ros::NodeHandle nh;

    Ultrasonic_Dicesion ultrasonic_dicesion(nh);

    thread t_ultrasonic_dicesion(std::bind(&Ultrasonic_Dicesion::Ultrasonic_Dicesion_Node, &ultrasonic_dicesion));
    t_ultrasonic_dicesion.detach();

    while(ros::ok())
    {
        printf("main sleep 1s~!\n");
        sleep(1);
    }
    return 0;
}

