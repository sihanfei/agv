ROS包名称：com2agv (communication to agv)
            |
            |
ros节点1：   |---com2agv:
            |---功能：整合所有自动驾驶控制指令，通过CAN网络和车辆vcu进行通信。
            |
            |---rostopic:
            |   |---广播(pub)："/drivers/com2agv/agv_status", "/drivers/com2agv/ADControlAGV", "/control/allinfo", "/com2agv/com2agv_status"
            |   |---订阅(sub): "/control/control_agv", "/drivers/com2agv/teleopcmd"
            |
            |---可运行指令：(区分测试模式与非测试模式，进入测试模式需要注释掉include/com2agv.h中的" Test_* "宏定义)
            |   |
            |   |---指令1：rosrun com2agv com2agv
            |   |   |---功能：正常输出所有自动驾驶指令（非测试模式）
            |   |
            |   |---指令2：rosrun com2agv com2agv test -1
            |   |   |---功能：测试倒车（测试模式）
            |   |
            |   |---指令2：rosrun com2agv com2agv test 1
            |   |   |---功能：测试前进（测试模式）
            |   |
            |   |---指令2：rosrun com2agv com2agv test 2
            |   |   |---功能：测试前轮正弦转动（测试模式）
            |   |
            |   |---指令2：rosrun com2agv com2agv test 3
            |   |   |---功能：测试后轮正弦转动（测试模式）
            |   |
            |   |---指令2：rosrun com2agv com2agv test 4
            |       |---功能：测试前后轮同时正弦转动（测试模式）
            |
ros节点2：   |---KeyboardTeleop:
            |---功能：利用键盘输出基本的车辆可操作指令，用于自动驾驶临时直接测试车辆VCU功能。
            |
            |---rostopic:
            |   |---广播(pub)："/drivers/com2agv/teleopcmd"
            |
            |---可运行指令：
            |   |
            |   |---指令1：rosrun com2agv KeyboardTeleop
            |       |---功能：键盘模拟输出控制车辆的指令
            |
            |---PS：
                |
                |   键盘控制逻辑：
                |   按键/             功能/
                |   a           自动驾驶功能开关
                |   e           紧急停车标志位开关
                |   b           手刹功能开关
                |   l           顶升预处理切换开关，按键是字母l
                |    +u         顶升机构抬起
                |    +n         顶升机构落下
                |   d           档位预处理切换开关
                |    +u         档位切换到D档前进档
                |    +n         档位切换到R档后退档
                |   8           档位切换到D档并且给定车辆固定速度
                |   2           档位切换到R档并且给定车辆固定速度
                |   4           车辆转向机构向左转向角度持续增加
                |   6           车辆转向机构向右转向角度持续增加
