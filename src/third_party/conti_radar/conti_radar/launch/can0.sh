#!/bin/sh

#装载CAN模块:

sudo modprobe can
echo "can success!"
sudo modprobe can_raw
echo "can_raw success!"
sudo modprobe can_dev
echo "can_dev success!"
sudo modprobe mttcan
echo "mttcan success!"
#配置 CAN0 and CAN1:

sudo ip link set can0 type can bitrate 500000
echo "can0 bitrate is 500000"
sudo ip link set up can0
echo "can0 set success!"

