/************************************
**  键盘控制逻辑：
**  按键/             功能/
**  a           自动驾驶功能开关
**  e           紧急停车标志位开关
**  b           手刹功能开关
**  l           顶升预处理切换开关，按键是字母l
**   +u         顶升机构抬起
**   +n         顶升机构落下
**  d           档位预处理切换开关
**   +u         档位切换到D档前进档
**   +n         档位切换到R档后退档
**  8           档位切换到D档并且给定车辆固定速度
**  2           档位切换到R档并且给定车辆固定速度
**  4           车辆转向机构向左转向角度持续增加
**  6           车辆转向机构向右转向角度持续增加
************************************/
#include "KeyboardTeleop.h"

using namespace std;
using namespace com2agv;

namespace com2agv
{
    AGVKeyboardTeleopNode::AGVKeyboardTeleopNode():
    velmax_(0.5),       //纵向速度 unit: m/s
    accvel_(0.4),       //纵向加速度 unit: m/s2
    omegamax_(3.0),     //角速度 unit: degree/s
    speed_(0.0),
    angle_(0.0)
    {
        nh.param("velmax", velmax_, velmax_);
        nh.param("omegamax", omegamax_, omegamax_);

        agv_pub = nh.advertise<control_msgs::com2veh>("com2agv/teleopcmd", 1);
    }

    AGVKeyboardTeleopNode::~AGVKeyboardTeleopNode()
    {
    }

    int kfd = 0;
    struct termios cooked,raw;

    void quit(int sig)
    {
        tcsetattr(kfd, TCSANOW, &cooked);
        ros::shutdown();
        exit(0);
    }

    void AGVKeyboardTeleopNode::keyloop()
    {
        char c,c_pre;//[3];
        bool Action=false;
        double starttime=0;
        double deltet=0.03;     //循环运行一次时间
        float angle_max = 40.0f;
        float speed_max = 4.0f;
        speed_ =0.0;
        angle_ = 0.0;
        uint8_t Gears = 0x3;    //Neutral
        uint8_t liftcmd = 0x2;  //停止
        bool EPB = false;        //手刹闭合
        bool EStop = false;      //紧停开关：无动作
        bool AutoDrive_Enable = false;//自动驾驶无效

        //kfd=open("/dev/tty",O_RDONLY);
        tcgetattr(kfd, &cooked);
        memcpy(&raw, &cooked, sizeof(struct termios));
        raw.c_lflag &=~ (ICANON | ECHO);
        raw.c_cc[VEOL] = 1; 
        raw.c_cc[VEOF] = 2;
        tcsetattr(kfd, TCSANOW, &raw);
        // int flags = fcntl(kfd,F_GETFL);//以下设置read函数为非阻塞模式,目前没有用
        // flags |= O_NONBLOCK;
        // if(fcntl(kfd,F_SETFL,flags)==-1){exit(1);}
        // flags = fcntl(0,F_GETFL);
        // if(fcntl(0,F_SETFL,flags)==-1){exit(1);}

        cout<< "Reading from keybord" <<endl;
        cout<< "Please use keys: 8 2 4 6 5 control agv."<<endl;

        starttime = ros::Time::now().toSec();
        for (;;)
        {
            // memset(c,3,0);
            if(read(kfd, &c, 3) < 0)
            {
                perror("read()abc:");
                exit(-1);
            }

            // ROS_INFO("KEY:0x %2x",c);
            //ROS_INFO("KEY:0x %2x %2x %2x",c[0],c[1],c[2]);
            switch (c)
            {
                case VK_UP8:
                    speed_ += deltet * accvel_;
                    // Gears = 0x1;    //Drive
                    Action = true;
                    ROS_INFO("VK_UP");
                    break;
                case VK_DOWN2:
                    speed_ -= deltet * accvel_;
                    // Gears = 0x4;    //Reverse
                    Action = true;
                    ROS_INFO("VK_DOWN");
                    break;
                case VK_LEFT4:
                    angle_ -= deltet * omegamax_;
                    Action = true;
                    ROS_INFO("VK_LEFT");
                    break;
                case VK_RIGHT6:
                    angle_ += deltet * omegamax_;
                    Action = true;
                    ROS_INFO("VK_RIGHT");
                    break;
                case VK_STOP5:
                    speed_ = 0;
                    Gears = 0x2;    //Park
                    Action = true;
                    ROS_INFO("VK_DOWN");
                    break;
                case KEYCODE_L:
                    liftcmd = 0x2;  //Stop
                    Action = true;
                    break;
                case KEYCODE_D:
                    Gears = 0x3;    //Neutral
                    Action = true;
                    break;
                case KEYCODE_U: //上
                    if (c_pre == KEYCODE_L)
                    {
                        liftcmd = 0x1; //Rise
                        Action = true;
                    } 
                    else if (c_pre == KEYCODE_D)
                    {
                        Gears = 0x1; //drive
                        Action = true;
                    } 
                    else
                    {
                        ROS_INFO("Please switch Key_D or Key_L first");
                    }                
                    break;
                case KEYCODE_N: //下
                    if (c_pre == KEYCODE_L)
                    {
                        liftcmd = 0x4; //Land
                        Action = true;
                    } 
                    else if (c_pre == KEYCODE_D)
                    {
                        Gears = 0x4; //Reverse
                        Action = true;
                    } 
                    else
                    {
                        ROS_INFO("Please switch Key_D or Key_L first");
                    }                
                    break;
                case KEYCODE_B: //EPB
                    EPB = !EPB;        //手刹打开
                    Action = true;
                    break;
                case KEYCODE_E: //EStop
                    EStop = !EStop;      //紧停开关：打开
                    Action = true;
                    break;
                case KEYCODE_A: //AutoDrive_Enable
                    AutoDrive_Enable = !AutoDrive_Enable;//自动驾驶有效
                    Action = true;
                    break;
                case KEYCODE_C: //Init parameters
                    speed_ =0.0;
                    angle_ = 0.0;
                    Gears = 0x3;    //Neutral
                    liftcmd = 0x2;  //停止
                    EPB = true;        //手刹闭合
                    EStop = false;      //紧停开关：无动作
                    AutoDrive_Enable = true;//自动驾驶无效
                    Action = true;
                    break;
                

            }
            c_pre = c; 
            (angle_ < -angle_max) ? -angle_max : ((angle_ > angle_max) ? angle_max : angle_);  //limit angle 
            (speed_ < -speed_max) ? -speed_max : ((speed_ > speed_max) ? speed_max : speed_);  //limit speed        
            if (Action == true)
            {
                control_msgs::com2veh msg;
                msg.Vel_Req = speed_;
                msg.VehAgl_F = angle_;
                msg.VehAgl_R = -angle_;
                msg.Dir_PRND_Tran = Gears;
                msg.LiftCmd = liftcmd;
                msg.EPB = EPB;
                msg.EStop = EStop;
                msg.AutoDrive_Enable = AutoDrive_Enable;
                agv_pub.publish(msg);
                // ROS_INFO("hahaha");
                Action=false;
            }           
        }
        
        return;
    }

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "KeyboardTeleop");
    AGVKeyboardTeleopNode TeleopControl;

    signal(SIGINT,quit);

    TeleopControl.keyloop();

    return 0;
}



