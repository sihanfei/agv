Header header
float32 offset_y
float32 offset_heading
float32 offset_speed
uint8   fault_status
float32 vehicle_x           ## 单位：m     描述：车体在全局坐标系下的实际X位置
float32 vehicle_y           ## 单位：m     描述：车体在全局坐标系下的实际Y位置
float32 vehicle_err         ## 单位：m     描述：车身中心点与路径最近点的横向偏差
float32 min_index           ## 单位：num   描述：车身中心点在路径中对应的最近点索引号
float32 Vel_Req             ## 单位：m/s   描述：控制下发vcu需要达到的目标速度
float32 VehAgl_F            ## 单位：deg   描述：AGV前轮请求转角
float32 VehAgl_R            ## 单位：deg   描述：AGV后轮请求转角
uint8   LiftCmd             ## 单位：num   描述：举升目标指令，1：车身举起，2：停止动作，3：无效，4：车身落下
uint8   Dir_PRND_Tran       ## 单位：num   描述：车辆档位请求指令，1：前进档，2：停车档，3：空档，4：倒车档
bool    EPB                 ## 单位：num   描述：手刹启动请求，0：闭合，1：松开
bool    EStop               ## 单位：num   描述：紧急停车请求，0：无请求，1：请求制动
bool    AutoDrive_Enable    ## 单位：num   描述：自动驾驶使能开关，0：自动驾驶无效，1：自动驾驶有效
