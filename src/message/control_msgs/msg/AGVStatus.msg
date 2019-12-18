Header header
float32 ActualSpd           ## 单位：m/s   描述：VCU反馈车体实际速度
float32 ActualAgl_R         ## 单位：deg   描述：AGV后轮实际转角
float32 ActualAgl_F         ## 单位：deg   描述：AGV前轮实际转角
uint8   VEHMode             ## 单位：num   描述：AGV当前所在的模式，0：故障，1：调试，2：自动，3：手动（遥控）
uint8   VEHFlt              ## 单位：num   描述：故障状态，0：无故障，1：一级故障，2：二级故障，3：三级故障
uint8   LiftStatus          ## 单位：num   描述：举升状态，0：未知，1：上升到位，2：下降到位，3：运动中      
bool    HVStatus            ## 单位：num   描述：高压状态，0：非启动，1：启动
bool    EStopStatus         ## 单位：num   描述：紧停状态，0：无紧停，1：启动紧停
bool    EPBStatus           ## 单位：num   描述：手刹状态，0：闭合（制动），1：松开
uint8   Dir_PRND            ## 单位：num   描述：档位状态，1：前进档，2：停车档，3：空档，4：后退档
uint8   SOC                 ## 单位：num   描述：动力电池剩余电量，0：满电，1：小于75%，2：小于50%，3：小于25%
uint8   Rolling_Counter     ## 单位：num   描述：vcu反馈的的心跳值
