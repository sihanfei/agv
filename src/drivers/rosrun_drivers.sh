#!/bin/bash

sleep 1s
{
    mate-terminal --window -t "can_wr" -e 'bash -c "rosrun can_wr can_wr_n;exec bash"'\
    # --tab -t "tanway_m16_lidar_node" -e 'bash -c "rosrun tanway_lidar_driver tanway_m16_lidar_node;exec bash"'\
    # --tab -t "tanway_m16_syncLidars" -e 'bash -c "rosrun tanway_lidar_driver tanway_m16_syncLidars;exec bash"'\
    --tab -t "cannet_radar_node" -x bash -c "rosrun conti_radar_driver cannet_radar_node;exec bash"
}&

sleep 1s
{
    mate-terminal -t "velodyne" -x bash -c "roslaunch ~/work/superg_agv/src/drivers/velodyne_driver/launch/multiple_VLP16_points.launch;exec bash"
}&


# sleep 1s
# {
#     mate-terminal -t "com2agv" -x bash -c "rosrun com2veh com2agv 1;exec bash"
# }&

##  键盘控制车
# sleep 1s
# {
#     mate-terminal -t "KeyboardTeleop" -x bash -c "rosrun com2veh KeyboardTeleop;exec bash"
# }&

##  定位模拟
# sleep 1s
# {
#     mate-terminal -t "pl_sender" -x bash -c "rosrun gps_imu pl_sender 1 1;exec bash"
# }&

##  融合定位
sleep 1s
{
    mate-terminal -t "FusionCenter" -x bash -c "rosrun data_fusion FusionCenter;exec bash"
}&

