#!/bin/bash


echo "start my agricultural car program"

sudo mount /dev/sda1  /mnt/zed_data/
chmod 777 /dev/ttyUSB0

if [ ! -d "/mnt/zed_data/pointclond_zed/" ];then
	echo "mount failed!"
	exit -1
fi
echo "mount successfully"


time_cur=`date -d next-day +%Y-%m-%d-%H_%M_%S`
mkdir -p /mnt/zed_data/zed_log/${time_cur}



source /opt/ros/melodic/setup.bash
source /home/ubuntu/zed_wrapper_modified/devel/setup.bash
/home/ubuntu/agritural_car/devel/setup.bash
export CUBA_HOME=/usr/local/cuda-10.2
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
export PATH=/usr/local/cuda-10.2/bin:$PATH


echo "[zed wrapper begin]" >> /mnt/zed_data/zed_log/${time_cur}/zed_wrapper.log

roslaunch zed_wrapper zed.launch >> /mnt/zed_data/zed_log/${time_cur}/zed_wrapper.log  & #background

echo "" >> /mnt/zed_data/zed_log/${time_cur}/pwm_ctrl.log
echo "[pwm control begin]" >>/mnt/zed_data/zed_log/${time_cur}/pwm_ctrl.log
rosrun pwm_ctrl pwm_ctrl &

echo "" >> /mnt/zed_data/zed_log/${time_cur}/data_capture.log
echo "[data capture begin]" >>/mnt/zed_data/zed_log/${time_cur}/data_capture.log
rosrun data_capture data_capture &

echo "" >> /mnt/zed_data/zed_log/${time_cur}/data_process.log
echo "[data capture begin]" >>/mnt/zed_data/zed_log/${time_cur}/data_process.log
rosrun data_process data_process &


rosrun image_view image_saver "_filename_format:=/mnt/zed_data/picture_zed/image_%06d.%s" image:=/zed/zed_node/left/image_rect_color &
#kill -9 $(pidof zed_wrapper)

#kill -9 $(pidof pwm_ctrl)

#kill -9 $(pidof data_capture)

#kill -9 $(pidof image_saver)
