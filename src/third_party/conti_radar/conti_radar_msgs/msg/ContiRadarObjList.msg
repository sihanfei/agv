std_msgs/Header header
float64 gps_time                                     #gps时间
uint32 num_of_conti_radar                            #障碍物目标数量
uint32 meas_counter                                  #测量周期计数器(从传感器启动之后就开始计数,当大于65535计数器就清零)
uint32 reserve           
conti_radar_msgs/ContiRadarObj[]  conti_radar_objs   #障碍物列表
