#! /bin/bash 

 #set -xv
#exec 1>/var/log/agv.log 2>&1

echo "**************************************************时间是：`date '+%Y-%m-%d %H:%M:%S'`" >/var/log/agv.log

source /opt/ros/kinetic/setup.bash

source /home/higo/work/superg_agv/devel/setup.bash

export SUDO_ASKPASS=/home/higo/work/superg_agv/src/monitor/rviz_monitor/scripts/_PWD_TEMP_

export ROS_MASTER_URI=http://192.168.2.200:11311
export ROS_IP=192.168.2.200
export ROS_LOG_DIR=/home/higo/work/log/roslog/

echo "sudo mount `date '+%Y-%m-%d %H:%M:%S'`" >>/var/log/agv.log
echo 123456 | sudo -S mount /dev/sda1 /home/higo/work/log >>/var/log/agv.log 2>&1 &

sleep 3s
echo "python3 routing.py `date '+%Y-%m-%d %H:%M:%S'`" >>/var/log/agv.log
screen_name=$"routing"
screen -dmS $screen_name

cmd=$"cd /home/higo/work/superg_agv/src/routing/map_new/ && python3 routing.py";
screen -x -S $screen_name -p 0 -X stuff "$cmd"
screen -x -S $screen_name -p 0 -X stuff $'\n'

# echo "python3 routing.py `date '+%Y-%m-%d %H:%M:%S'`" >/var/log/agv0.log
# cd /home/higo/work/superg_agv/src/routing/map_new/ 
# python3 routing.py >>/var/log/agv0.log 2>&1 &

sleep 3s
echo "civetweb `date '+%Y-%m-%d %H:%M:%S'`" >>/var/log/agv.log
screen_name=$"civetweb"
screen -dmS $screen_name

cmd=$"cd /home/higo/work/superg_agv/src/monitor/civetweb/www/ && ./civetweb";
screen -x -S $screen_name -p 0 -X stuff "$cmd"
screen -x -S $screen_name -p 0 -X stuff $'\n'
# echo "civetweb `date '+%Y-%m-%d %H:%M:%S'`" >/var/log/agv1.log
# cd /home/higo/work/superg_agv/src/monitor/civetweb/www/ 
# ./civetweb  >>/var/log/agv1.log 2>&1 &

sleep 3s
echo "roslaunch rviz_monitor start.launc `date '+%Y-%m-%d %H:%M:%S'`" >>/var/log/agv.log
cd
roslaunch rviz_monitor start.launch &
sleep 10s
echo "==================================================时间是：`date '+%Y-%m-%d %H:%M:%S'`" >>/var/log/agv.log
# exec 1>&-
# exec 2>&-
wait
exit 0
