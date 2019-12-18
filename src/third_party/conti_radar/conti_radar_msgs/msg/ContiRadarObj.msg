uint32 obstacle_id                 # obstacle Id

uint32 obstacle_class              # 0: point; 1: car; 2: truck; 3: pedestrian; 4: motorcycle; 5: bicycle; 6: wide; 7: unknown

float64 longitude_dist             #longitude distance to the radar; (+) = forward; unit = m

float64 lateral_dist               #lateral distance to the radar; (+) = left; unit = m

float64 longitude_vel              #longitude velocity to the radar; (+) = forward; unit = m/s

float64 lateral_vel                #lateral velocity to the radar; (+) = left; unit = m/s

float64 rcs                        #obstacle Radar Cross-Section; unit = dBsm雷达散射截面

uint32 dynprop                     #目标状态0 = moving, 1 = stationary, 2 = oncoming, 3 = stationary candidate   4 = unknown, 5 = crossing stationary, 6 = crossing moving, 7 = stopped 

float64 longitude_dist_rms         #longitude distance standard deviation to the radar; (+) = forward; unit = m

float64 lateral_dist_rms           #lateral distance standard deviation to the radar; (+) = left; unit = m

float64 longitude_vel_rms          #longitude velocity standard deviation to the radar; (+) = forward; unit = m/s

float64 lateral_vel_rms            #lateral velocity standard deviation to the radar; (+) = left; unit = m/s

float64 probexist                  #obstacle probability of existence置信度

uint32 meas_state                  #测量状态,指示目标是否有效,并在新的测量周期中被集群确认The following is only valid for the track object message 
				   #0 = deleted, 1 = new, 2 = measured, 3 = predicted, 4 = deleted for, 5 = new from merge


float64 longitude_accel            #longitude acceleration to the radar; (+) = forward; unit = m/s2

float64 lateral_accel              #lateral acceleration to the radar; (+) = left; unit = m/s2

float64 oritation_angle            #orientation angle to the radar; (+) = counterclockwise; unit = degree

float64 longitude_accel_rms        #longitude acceleration standard deviation to the radar; (+) = forward; unit = m/s2

float64 lateral_accel_rms          #lateral acceleration standard deviation to the radar; (+) = left; unit = m/s2

float64 oritation_angle_rms        #目标的方位角标准误差orientation angle standard deviation to the radar; (+) = counterclockwise; unit = degree

float64 length                     #obstacle length; unit = m

float64 width                      #obstacle width; unit = m
