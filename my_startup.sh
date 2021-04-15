#!/bin/bash


echo "start my agricultural car program"

sudo mount /dev/sda1  /mnt/zed_data/

time_cur=`date -d next-day +%Y-%m-%d-%H_%M_%S`
mkdir -p /mnt/zed_data/zed_log/${time_cur}



source /opt/ros/melodic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /home/ubuntu/agritural_car/devel/setup.bash
export CUBA_HOME=/usr/local/cuda-10.2
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.2/bin:$PATH


cd /home/ubuntu/catkin_ws
echo "[zed wrapper begin]" >> /mnt/zed_data/zed_log/${time_cur}/run.log

roslaunch zed_wrapper zed.launch >> /mnt/zed_data/zed_log/${time_cur}/run.log  & #background



cd /home/ubuntu/agritural_car/
echo "" >> /mnt/zed_data/zed_log/${time_cur}/run.log
echo "[data capture begin]" >>/mnt/zed_data/zed_log/${time_cur}/run.log
rosrun data_capture data_capture

echo "" >> /mnt/zed_data/zed_log/${time_cur}/run.log
echo "[pwm control begin]" >>/mnt/zed_data/zed_log/${time_cur}/run.log
