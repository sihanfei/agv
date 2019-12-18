Header header                                          #信息序列帧ID,计算完位置的时间戳 
uint16 year                                            #记录原始数据时间戳
uint8 month
uint8 day
uint8 hour
uint8 min
uint8 sec
float32 msec

float64 x                                              #港区坐标下的位置x(东向)
float64 y                                              #港区坐标下的位置y(北向)
float64 z

float64 heading
float64 roll
float64 pitch

float64 velocity_north
float64 velocity_east
float64 velocity_up

float64 velocity_vx
float64 velocity_vy
float64 velocity_vz

float64 dx
float64 dy
float64 dz

uint8 dr_valid_flag                                     #DR有效性标志:
                                                        #1:RTK固定解状态下(使用组合导航修正航位推算)
                                                        #2:非RTK固定解状态下(使用VCU速度,用于修正组合导航)