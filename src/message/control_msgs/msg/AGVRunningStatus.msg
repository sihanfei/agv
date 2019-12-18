Header header
float32 offset_y            ## 单位：m     描述：横向路径偏差,车身中心点与路径最近点的横向偏差
float32 offset_heading      ## 单位：deg   描述：航向角偏差
float32 offset_speed        ## 单位：m/s   描述：速度偏差
uint8   fault_status        ## 单位：num   描述：辆故障状态（0:正常;1:警告;2:一般故障;3:严重故障）
int8    running_status      ## 单位：num   描述：运动状态（-1:未知；0:静止；1:直行；2:转弯）
float32 vehicle_x           ## 单位：m     描述：车体在全局坐标系下的实际X位置
float32 vehicle_y           ## 单位：m     描述：车体在全局坐标系下的实际Y位置
float32 min_index           ## 单位：num   描述：车身中心点在路径中对应的最近点索引号
