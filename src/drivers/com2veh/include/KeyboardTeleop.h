#ifndef KEYBOARDTELEOP_H_
#define KEYBOARDTELEOP_H_

#include "ros/ros.h"
#include "control_msgs/com2veh.h"
#include "signal.h"
#include "termios.h"
#include "stdio.h"
#include <sys/fcntl.h>

namespace com2agv
{
    class AGVKeyboardTeleopNode
    {
    public:
        AGVKeyboardTeleopNode();
        ~AGVKeyboardTeleopNode();

        void keyloop();

    private:
        ros::NodeHandle nh;
        float velmax_, accvel_, omegamax_, speed_, angle_; 
        ros::Publisher agv_pub;
        
        enum keycode
        {                       //number:
            VK_LEFT4    = 0x34, //4
            VK_UP8      = 0x38, //8
            VK_RIGHT6   = 0x36, //6
            VK_DOWN2    = 0x32, //2
            VK_STOP5    = 0x35, //5
            KEYCODE_L   = 0X6C,
            KEYCODE_D   = 0X64,
            KEYCODE_B   = 0X62,
            KEYCODE_E   = 0X65,
            KEYCODE_A   = 0X61,
            KEYCODE_U   = 0X75, //up
            KEYCODE_N   = 0X6e,  //down
            KEYCODE_C   = 0X63  //clear
            
            // KEYCODE_W = 0X77,
            // KEYCODE_A = 0X61,
            // KEYCODE_S = 0X73,
            // KEYCODE_D = 0X64,

        };
    };
}

#endif

