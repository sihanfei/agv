Header  header
uint8   work_mode   #车辆控制模式（0-未知；1-自动；2-远程；3-待机）
uint8   task_ID     #车辆正在处理的指令序列号
uint8   task_Status #当前处理指令的状态（1-接收到；2-校验成功；3-执行中；4-执行完成；5-校验失败；6-执行失败）
float64 x           #港区坐标X
float64 y           #港区坐标Y
float64 heading     #港区坐标
float64 velocity       #速度 m/s
uint8   container_size #箱型（0-空车；1-前20尺；2-后20尺；3-双20尺；4-40尺；5-45尺；6-未知）
uint8   load_weight    #载重            （单位：t）
int8    running_status #运动状态（-1：未知；0：静止；1：直行；2：转弯；3：斜行；4：顶升上升；5：顶升下降；6：充电）