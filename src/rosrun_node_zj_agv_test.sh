#!/bin/bash
sleep 1s
{
    mate-terminal --window -t "rviz_visualization" -e 'bash -c "rosrun rviz_monitor rviz_visualization_pub;exec bash"'\
    --tab -t "rviz" -e 'bash -c "rviz -d ~/work/superg_agv/src/monitor/rviz_monitor/monitor.rviz;exec bash"'\
    --tab -t "map routing" -x bash -c "cd ~/work/superg_agv/src/routing/map_new/;python3 routing.py;exec bash"
}&

# sleep 1s
# {
#     mate-terminal -t "pl_sender" -x bash -c "rosrun gps_imu pl_sender 5 50;exec bash"
# }&

sleep 1s
{
    mate-terminal -t "ref_sender" -x bash -c "rosrun map ref_sender;exec bash"
}&

sleep 1s
{
    mate-terminal -t "planning_sender" -x bash -c "rosrun map planning_sender_pub 8 9;exec bash"
}&

sleep 1s
{
    mate-terminal -t "per_test" -x bash -c "rosrun perception per_test;exec bash"
}&

