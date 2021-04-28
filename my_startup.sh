#!/bin/bash


echo "start my agricultural car program"

sudo mount /dev/sda1  /mnt/zed_data/
chmod 777 /dev/ttyUSB0

time_cur=`date -d next-day +%Y-%m-%d-%H_%M_%S`
mkdir -p /mnt/zed_data/zed_log/${time_cur}



source /opt/ros/melodic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
source /home/ubuntu/agritural_car/devel/setup.bash
export CUBA_HOME=/usr/local/cuda-10.2
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.2/bin:$PATH


cd /home/ubuntu/catkin_ws
echo "[zed wrapper begin]" >> /mnt/zed_data/zed_log/${time_cur}/zed_wrapper.log

roslaunch zed_wrapper zed.launch >> /mnt/zed_data/zed_log/${time_cur}/zed_wrapper.log  & #background

echo "" >> /mnt/zed_data/zed_log/${time_cur}/pwm_ctrl.log
echo "[pwm control begin]" >>/mnt/zed_data/zed_log/${time_cur}/pwm_ctrl.log
rosrun pwm_ctrl pwm_ctrl &

cd /home/ubuntu/agritural_car/
echo "" >> /mnt/zed_data/zed_log/${time_cur}/data_capture.log
echo "[data capture begin]" >>/mnt/zed_data/zed_log/${time_cur}/data_capture.log
rosrun data_capture data_capture &



#kill -9 $(pidof zed_wrapper)

#kill -9 $(pidof pwm_ctrl)

#kill -9 $(pidof data_capture)

