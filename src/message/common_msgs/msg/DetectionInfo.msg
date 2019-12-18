uint32      id          # 障碍物跟踪id
uint8       obj_class   # 障碍物类别
float32[6]    state       # 障碍物 min x  min y max x  max y vx vy
float32[36] measurement_cov # 传感器的测量协方差
float32     confidence      # 障碍物置信度
Point2D[4]  peek            #障碍物外接矩形的四个角点
