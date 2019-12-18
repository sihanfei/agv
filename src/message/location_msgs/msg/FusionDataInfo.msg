Header header                                          #帧ID+时间戳
uint16 year                                            #开始解析数据的时间
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float32 msec
float32 pose_confidence                                 #位置置信度
geometry_msgs/Point pose                                #全局坐标系下车辆中心点坐标位置(单位:m)
geometry_msgs/Point pose_llh                            #大地坐标系下车辆天线坐标位置（顺序：纬经高，单位deg）
float64 yaw                                             #姿态角信息，Yaw(单位:deg)
float64 pitch                                           #姿态角信息，Pitch(单位:deg)
float64 roll                                            #姿态角信息，Roll(单位:deg)
geometry_msgs/Point vel_ctr                             #全局坐标系下车辆中心线速度(xyz分别为东北天单位:m/s)
geometry_msgs/Twist velocity                            #全局坐标系下车辆定位天线的线速度(单位:m/s)及车辆坐标系下角速度(单位:deg/s)
geometry_msgs/Accel accel                               #车辆坐标系下车辆的线加速度(单位:m/s2),角加速度暂无
uint8 satellite_status                                  #卫星状态
                                                        #0：不定位不定向
                                                        #1：单点定位定向
                                                        #2：伪距差分定位定向
                                                        #3：组合推算
                                                        #4：RTK稳定解定位定向
                                                        #5：RTK浮点解定位定向
                                                        #6：单点定位不定向
                                                        #7：伪距差分定位不定向
                                                        #8：RTK稳定解定位不定向
                                                        #9：RTK浮点解定位不定向
uint8 system_status                                     #系统状态
                                                        #0：初始化 
                                                        #1：卫导模式 
                                                        #2：组合导航模式 
                                                        #3：纯惯导模式 
uint8 fuse_state                                        #融合数据输出状态:
                                                        #1:组合导航P2输出
                                                        #2:IMU+误差补偿
                                                        #3:IMU+ZUPT(零速修正)
                                                        #4:IMU+场端lidar
                                                        #5:IMU+场端lidar+ZUPT
                                                        #6:IMU+场端UWB
                                                        #7:IMU+场端UWB+ZUPT
                                                        #8:IMU+lidar_slam
                                                        #9:IMU+lidar_slam+ZUPT
                                                        #10:IMU+场端lidar+lidar_slam
                                                        #11:IMU+场端lidar+lidar_slam+ZUPT
                                                        #12:IMU+场端UWB+lidar_slam
                                                        #13:IMU+场端UWB+lidar_slam+ZUPT
                                                        #14:IMU+DR
                                                        