Header header                                       #信息序列帧ID,计算完位置的时间戳 

uint16 flidar_id                                    #场端Lidar的身份编号
uint16 AGV_id                                       #AGV的身份编号

uint16 year                                         #记录原始数据时间戳
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float32 msec


geometry_msgs/Point tager_position                  # 停位点的港区坐标（港区全局坐标系,单位m,xyz东北天)
                                                    # tager_position.x = 东向 
                                                    # tager_position.y = 北向 
                                                    # tager_position.z = 天向 

geometry_msgs/Point agv_distance                    # AGV距离停位点横向、纵向距离(相对坐标，单位 m，负值时为越过停止位的距离) 
                                                    # agv_distance.x = 横向距离（港区坐标系X轴）  
                                                    # agv_distance.y = 纵向距离（港区坐标系Y轴）
                                                    # agv_distance.z = 未定义

float64   heading                                   #方向角 北偏东为正（yaw单位 deg）

float64   Vx                                        #横向（垂直车道线）速度 
float64   Vy                                        #纵向（平行车道线）速度


