#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

#include <sstream>

#include "common_func/move_ctrl.h"
#include "common_func/drv_uart.h"




#define WORK_MODE_THRESHOLD1    (900)
#define WORK_MODE_THRESHOLD2    (1100)

#define WORK_STATUS_THRESHOLD1  (900)

#define LOOP_RATE_DEFAULT   (30)

using namespace std;

//typedef signed char        int8_t;
//typedef unsigned char      uint8_t;
//
//typedef signed short       int16_t;
//typedef unsigned short     uint16_t;
//
//typedef signed int         int32_t;
//typedef unsigned int       uint32_t;


static WORK_STATUS_t local_work_status = PAUSE;
static bool pwm_is_used_by_program = false;

#define WORK_STATUS_DEFAULT (PAUSE)
#define MAX_CHANNELS        (16)
#define WORK_MODE_CHANNEL   (6)
#define WORK_STATUS_CHANNEL (5)
#define SPEED_CHANNEL       (4)
#define DIRECT_CHANNEL      (3)



static void Process(uint8_t* raw,uint16_t* result)
{
  uint8_t bitsToRead=3; // bitsToRead表示需要从下一个字节中读取多少bit。规律：bitsToRead 每次总是增加 3
  uint8_t bitsToShift;
  uint8_t startByte=21;
  uint8_t channelId=15;

  do
  {
    result[channelId]=raw[startByte];

    if(bitsToRead<=8)
    {
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }
    else
    {
      result[channelId]<<=8;
      result[channelId]+=raw[startByte-1];
      startByte--;
      bitsToRead-=8;
      result[channelId]<<=bitsToRead;
      bitsToShift=8-bitsToRead;
      result[channelId]+=(raw[startByte-1]>>bitsToShift);
    }

    result[channelId]&=0x7FF;

    channelId--;
    startByte--;
    bitsToRead+=3;

  }while(startByte>0);  
}

static int decode_value(uint8_t* buf, int len, uint8_t *frame_data, ctrl_desc_t *desc)
{
	int  			left_len = len;

	if((buf == NULL) || (len <= 0))
	{
		return -1;
	}

	int last_index = 0, cur_index = 0;

	while(cur_index < len)
	{
		if((buf[last_index] ==FRAME_END_VALUE) && (buf[cur_index] == FRAME_START_VALUE))
		{
			if(left_len >= FRAME_LEN)
			{
				memcpy(frame_data, &buf[cur_index], FRAME_LEN);
				//decode
                uint16_t out[MAX_CHANNELS] = {0};
                Process(frame_data+1, out);

                if(out[WORK_MODE_CHANNEL - 1] < WORK_MODE_THRESHOLD1)
                    desc->work_mode = MANUAL_MODE;
                else if(out[WORK_MODE_CHANNEL - 1] > WORK_MODE_THRESHOLD2)
                    desc->work_mode = SAMPLE_MODE;
                else
                    desc->work_mode = AUTO_MODE;

                desc->status = (out[WORK_STATUS_CHANNEL - 1] < WORK_STATUS_THRESHOLD1) ? PAUSE : WORKING;
                desc->speed_pwm_info = out[SPEED_CHANNEL - 1];
                desc->direction_pwm_info = out[DIRECT_CHANNEL - 1];
                
                //uart_send(frame_data, sizeof(frame_data));

                return 0;

			}
		}
		else
		{
			last_index = cur_index;
			cur_index++;
			left_len--;
		}


	}

	return -1;
}




void rc_info_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  //ROS_INFO("nick enter rc_info_callback");
  static uint16_t last_work_status = PAUSE;
  static uint16_t cur_work_status = PAUSE;
  uint8_t frame_temp1[FRAME_LEN] = {0};
  uint8_t frame_temp2[FRAME_LEN] = {0};
  uint8_t frame_temp3[FRAME_LEN] = {0};  


  uint8_t recv_data = msg->data;

  pwm_is_used_by_program = true;
  switch(recv_data)
  {
    case(PROG_TASK_READY):
    {
        set_direct_left(frame_temp1, frame_temp2);
        set_speed_0(frame_temp2, frame_temp3);
        uart_send(frame_temp3, sizeof(frame_temp3));
        sleep(1);


        memset(frame_temp1, 0, sizeof(frame_temp1));
        memset(frame_temp2, 0, sizeof(frame_temp2));
        memset(frame_temp3, 0, sizeof(frame_temp3));

        set_direct_right(frame_temp1, frame_temp2);
        set_speed_0(frame_temp2, frame_temp3);
        uart_send(frame_temp3, sizeof(frame_temp3));
        sleep(1);

        set_direct_front(frame_temp1, frame_temp2);
        set_speed_0(frame_temp2, frame_temp3);
        uart_send(frame_temp3, sizeof(frame_temp3));

        cout << "power on & ready" <<endl;
        break;        
    }
    case(PROG_TASK_BEGIN):
    {
        memset(frame_temp1, 0, sizeof(frame_temp1));
        memset(frame_temp2, 0, sizeof(frame_temp2));
        memset(frame_temp3, 0, sizeof(frame_temp3));


        set_direct_left(frame_temp1, frame_temp2);


        uint8_t decode_temp[FRAME_LEN] = {0};
        ctrl_desc_t local_ctrl;
        memset(&local_ctrl, 0, sizeof(local_ctrl));
        if(decode_value(frame_temp2, FRAME_LEN, decode_temp, &local_ctrl) == 0)
        {
            printf("decode1: %u, %u, %u, %u\n", local_ctrl.work_mode, local_ctrl.status, local_ctrl.speed_pwm_info, local_ctrl.direction_pwm_info);
        }
        else
        {
            printf("decode1: decode failed\r\n");
        }


        set_speed_0(frame_temp2, frame_temp3);

        memset(decode_temp, 0, FRAME_LEN);
        memset(&local_ctrl, 0, sizeof(local_ctrl));
        if(decode_value(frame_temp3, FRAME_LEN, decode_temp, &local_ctrl) == 0)
        {
            printf("decode2: %u, %u, %u, %u\n", local_ctrl.work_mode, local_ctrl.status, local_ctrl.speed_pwm_info, local_ctrl.direction_pwm_info);
        }
        else
        {
            printf("decode2: decode failed\r\n");
        }


        uart_send(frame_temp3, sizeof(frame_temp3));

        sleep(1);
        cout << "start sample" <<endl;
        break;
    }
    case(PROG_TASK_END):
    {
        memset(frame_temp1, 0, sizeof(frame_temp1));
        memset(frame_temp2, 0, sizeof(frame_temp2));
        memset(frame_temp3, 0, sizeof(frame_temp3));

        set_direct_right(frame_temp1, frame_temp2);
        set_speed_0(frame_temp2, frame_temp3);
        uart_send(frame_temp3, sizeof(frame_temp3));
        sleep(1);
        cout << "stop sample" <<endl;
        break;
    }
    pwm_is_used_by_program = false;

  }
}


int main(int argc, char **argv)
{
    uart_init();
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt64>(RC_CTRL_INFO, 1000);
    ros::Subscriber pgro_status_sub = n.subscribe(PROG_SAMPLE_STATUS_TOPIC, 100, rc_info_callback);


    ros::Rate loop_rate(LOOP_RATE_DEFAULT);

    int count = 0;
    uint8_t buf[MAX_BUF_SIZE] = {0};
    int len = 0;



    while (ros::ok())
    {
        std_msgs::UInt64 msg;

        len = uart_recv_timeout(buf, sizeof(buf), 100);
        //printf("[info] read bytes size: %d\n", len);

    	if(len > 0)
        {
            ctrl_desc_t local_ctrl;
            uint8_t  frame_data[FRAME_LEN];

            if(decode_value(buf, len, frame_data, &local_ctrl) == 0)
            {
                if((local_ctrl.work_mode == MANUAL_MODE) && (pwm_is_used_by_program == false))
                {
                    //in mannul mode, sendout sbus data as RC input
                    uart_send(frame_data, sizeof(frame_data));
                }
                else
                {
                    //TODO: ctrl speed & direction as program in auto mode
                    uart_send(frame_data, sizeof(frame_data));
                }
                local_work_status = local_ctrl.status;

                uint64_t data_to_send = 0;
                data_to_send = (uint64_t)((uint16_t)local_ctrl.work_mode);
                data_to_send |= ((uint64_t)((uint16_t)local_ctrl.status)) << 16;
                data_to_send |= ((uint64_t)((uint16_t)local_ctrl.speed_pwm_info)) << 32;
                data_to_send |= ((uint64_t)((uint16_t)local_ctrl.direction_pwm_info)) << 48;
                msg.data = data_to_send;
                chatter_pub.publish(msg);

                //printf("rc info:0x%llx", msg.data);
                //printf("%u, %u, %u, %u\n", local_ctrl.work_mode, local_ctrl.status, local_ctrl.speed_pwm_info, local_ctrl.direction_pwm_info);
            }
            memset(buf, 0, sizeof(buf));
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }

    return 0;
}
