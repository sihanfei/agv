Header header
##HavalControl Params
float32 VelSpeed            ## 单位：km/h  描述：哈弗车实际车速
float32 SteerAngle          ## 单位：deg   描述：哈弗车方向盘实际转向角度（非车身航向角）
float32 ExpSpeed            ## 单位：km/h  描述：控制期望的车辆速度
float32 ExpAngle            ## 单位：deg   描述：控制期望的方向盘转角
float32 ACCexp              ## 单位：m/s2  描述：控制期望的车辆加速度
float32 vehicle_x           ## 单位：m     描述：车体在全局坐标系下的实际X位置
float32 vehicle_y           ## 单位：m     描述：车体在全局坐标系下的实际Y位置
float32 vehicle_err         ## 单位：m     描述：车身中心点与路径最近点的横向偏差
float32 min_index           ## 单位：num   描述：车身中心点在路径中对应的最近点索引号
##AGVStatus Params (vcu data)
float32 ActualSpd           ## 单位：m/s   描述：VCU反馈车体实际速度
float32 ActualAgl_R         ## 单位：deg   描述：AGV后轮实际转角
float32 ActualAgl_F         ## 单位：deg   描述：AGV前轮实际转角
uint8   VEHMode             ## 单位：num   描述：AGV当前所在的模式，0：故障，1：调试，2：自动，3：手动（遥控）
uint8   VEHFlt              ## 单位：num   描述：故障状态，0：无故障，1：一级故障，2：二级故障，3：三级故障
uint8   LiftStatus          ## 单位：num   描述：举升状态，0：未知，1： 上升到位，2：下降到位，3：运动中      
bool    HVStatus            ## 单位：num   描述：高压状态，0：非启动，1：启动
bool    EStopStatus         ## 单位：num   描述：紧停状态，0：无紧停，1：启动紧停
bool    EPBStatus           ## 单位：num   描述：手刹状态，0：闭合（制动），1：松开
uint8   Dir_PRND            ## 单位：num   描述：档位状态，1：前进档，2：停车档，3：空档，4：后退档
uint8   SOC                 ## 单位：num   描述：动力电池剩余电量，0：满电，1：小于75%，2：小于50%，3：小于25%
uint8   Rolling_Counter     ## 单位：num   描述：vcu反馈的的心跳值
##ADControlAGV Params
float32 Vel_Req             ## 单位：m/s   描述：控制下发vcu需要达到的目标速度
uint8   LiftCmd             ## 单位：num   描述：举升目标指令，1：车身举起，2：停止动作，3：无效，4：车身落下
uint8   Dir_PRND_Tran       ## 单位：num   描述：车辆档位请求指令，1：前进档，2：停车档，3：空档，4：倒车档
float32 VehAgl_F            ## 单位：deg   描述：AGV前轮请求转角
bool    EPB                 ## 单位：num   描述：手刹启动请求，0：闭合，1：松开
bool    EStop               ## 单位：num   描述：紧急停车请求，0：无请求，1：请求制动
bool    AutoDrive_Enable    ## 单位：num   描述：自动驾驶使能开关，0：自动驾驶无效，1：自动驾驶有效
float32 VehAgl_R            ## 单位：deg   描述：AGV后轮请求转角