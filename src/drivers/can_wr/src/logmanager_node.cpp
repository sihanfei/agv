#include "logmanager.h"


int main(int argc, char **argv)
{
    ros::init(argc, argv, "logmanager");
    ros::NodeHandle nh;

    logmanage logmanage_;

    thread checkDisk(std::bind(&logmanage::checkTheDisk,&logmanage_));
    checkDisk.detach();
    
    thread time2creat(std::bind(&logmanage::time2Createfile,&logmanage_));
    time2creat.detach();

    thread time2Package(std::bind(&logmanage::time2Package,&logmanage_));
    time2Package.detach();

    while(ros::ok())
    {
        sleep(1);
    }
    return 0;
}