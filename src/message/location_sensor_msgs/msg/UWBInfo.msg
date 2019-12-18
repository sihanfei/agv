Header header                                          #信息序列帧ID,计算完位置的时间戳 
uint16 year                                            #记录原始数据时间戳
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float32 msec
uint8 fuwb_id                                          #场端UWB基站的身份编号
geometry_msgs/Point fuwb_pose                          #场端UWB基站的绝对位置（港区全局坐标系,单位m，xyz东北天)
float32 radius                                         #场端UWB的有效半径
bool fuwb_valid_flag                                   #是否在场端UWB的有效区域范围 0:不在有效区域 1:在有效区域
uint8 agv_id                                           #AGV的身份编号
geometry_msgs/PoseWithCovariance pose_cov              #IMU位置，及协方差,(大地坐标系，单位:deg)
float32 yaw                                            #方向角（单位：deg）
float32 pitch                                          #俯仰角
float32 roll                                           #横滚角
geometry_msgs/TwistWithCovariance vel_cov
float32 pose_confidence                               #位置置信度
float32 orien_confidence                              #姿态角置信度
float32 linear_confidence                             #线速度置信度
float32 angular_confidence                            #角速度置信度
