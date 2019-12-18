Header header
uint32 GPS_week                                         #GPS周
float64 GPS_sec                                         #GPS周内秒
float64 base_line                                       #基线长
uint8 NSV1                                              #搜索到的卫星数
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
uint16 year                                             #开始解析数据的时间
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float64 msec

geometry_msgs/Point pose                                #位置纬经高
float64 yaw                                             #姿态角信息，Yaw
float64 pitch                                           #姿态角信息，Pitch
float64 roll                                            #姿态角信息，Roll
geometry_msgs/Vector3 velocity                          #速度东北天

geometry_msgs/Accel accelgyro                           #惯导的陀螺角速度与加表加速度
