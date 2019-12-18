Header header
uint8 sensor_type
uint8 sensor_index
uint32 obstacle_num
common_msgs/DetectionInfo[] object_list
float32 yaw                                             #姿态角信息，Yaw(单位:deg)
float32 pitch                                           #姿态角信息，Pitch(单位:deg)
float32 roll                                            #姿态角信息，Roll(单位:deg)
geometry_msgs/Point pose                                #全局坐标系下车辆中心点坐标位置(单位:m)
geometry_msgs/Twist velocity                            #全局坐标系下车辆的线速度(单位:m/s)及车辆坐标系下角速度(单位:deg/s)