Header header                                          #信息序列帧ID,计算完位置的时间戳 
uint16 year                                            #记录原始数据时间戳
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float32 msec
geometry_msgs/PoseWithCovariance pose_cov              #IMU位置，及协方差,(大地坐标系，顺序：纬经高，单位:deg)
float32 yaw                                            #方向角（单位：deg）
float32 pitch                                          #俯仰角
float32 roll                                           #横滚角
geometry_msgs/TwistWithCovariance vel_cov              #速度及协方差,线速度(全局坐标系,顺序：东北天，单位m/s),角速度(车体坐标系,单位deg/s)
float32 pose_confidence                               #位置置信度
float32 orien_confidence                              #姿态角置信度
float32 linear_confidence                             #线速度置信度
float32 angular_confidence                            #角速度置信度