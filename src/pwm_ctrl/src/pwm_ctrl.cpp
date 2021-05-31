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
#include <pthread.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include "ros/ros.h"
//#include "std_msgs/String.h"
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

#include <sstream>

#include "common_func/move_ctrl.h"
#include "common_func/drv_uart.h"



#define PWM_CTRL_LOOP_US        (100000)
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

typedef enum
{
    PWM_CTRL_IDLE = 1,
    PWM_CTRL_READY,
    AUTO_MODE_INIT,
    AUTO_MODE_RUN,
    MANUAL_MODE_INIT,
    MANUAL_MODE_RUN,
    SAMPLE_MODE_INIT,
    SAMPLE_MODE_RUN,
} pwm_control_stage_t;

static pwm_control_stage_t ctrl_stage = PWM_CTRL_IDLE;
static uint8_t sample_task_status = PROG_TASK_UNKONW;
static uint8_t navigate_task_status = PROG_TASK_UNKONW;

static uint8_t pwm_data_global[FRAME_LEN] = {0};


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





void sample_task_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  uint8_t recv_data = msg->data;

  switch(recv_data)
  {
    cout << "[sample task info]: " << recv_data << endl;

    case(PROG_TASK_READY):
    {
        sample_task_status = PROG_TASK_READY;
        break;
    }
    case(PROG_TASK_BEGIN):
    {
        sample_task_status = PROG_TASK_BEGIN;
        break;
    }
    case(PROG_TASK_END):
    {
        sample_task_status = PROG_TASK_END;
        break;
    }
    default:
    {
    }
  }
}


void navi_task_callback(const std_msgs::UInt8::ConstPtr& msg)
{
  uint8_t recv_data = msg->data;

  switch(recv_data)
  {
    cout << "[navigation task info]: " << recv_data << endl;

    case(PROG_TASK_READY):
    {

        navigate_task_status = PROG_TASK_READY;
        break;
    }
    case(PROG_TASK_BEGIN):
    {
        navigate_task_status = PROG_TASK_BEGIN;
        break;
    }
    case(PROG_TASK_END):
    {
        navigate_task_status = PROG_TASK_END;
        break;
    }
    default:
    {
    }
  }
}

static void* ctrl_mission(void *arg)
{
    printf("enter ctrl_mission\r\n");

    while(1)
    {
        if(ctrl_stage == AUTO_MODE_RUN)
        {

        }
        else
        {
            uart_send(pwm_data_global, sizeof(pwm_data_global));
        }

        usleep(PWM_CTRL_LOOP_US);
    }

    return NULL;
}


int main(int argc, char **argv)
{
    uart_init();
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt64>(RC_CTRL_INFO, 1000);
    ros::Subscriber sample_task_sub = n.subscribe(PROG_SAMPLE_STATUS_TOPIC, 100, sample_task_callback);
    ros::Subscriber navi_task_sub = n.subscribe(PROG_NAV_STATUS_TOPIC, 100, navi_task_callback);

    pthread_t thread;
    static ctrl_desc_t local_ctrl;

    ros::Rate loop_rate(LOOP_RATE_DEFAULT);

    int count = 0;
    uint8_t buf[MAX_BUF_SIZE] = {0};
    int len = 0;

    //create sbus output thread
    if ((pthread_create(&thread, NULL, ctrl_mission, NULL) == -1))
    {
        printf("thread ctrl_mission create error!\n");
        return 1;
    }


    //start state machine translate
    while(ros::ok())
    {

        std_msgs::UInt64 msg;
        uint8_t decode_ret = 1;
        uint8_t  frame_data[FRAME_LEN] = {0};

        len = uart_recv_timeout(buf, sizeof(buf), 100);
        //printf("[info] read bytes size: %d\n", len);

    	if(len > 0)
        {
            decode_ret = decode_value(buf, len, frame_data, &local_ctrl);
        }
        memset(buf, 0, sizeof(buf));

        //publish RC infomation firstly
        if(decode_ret == 0)
        {
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

        //translate the state machine 2ndly
        if(decode_ret == 0)
        {
            switch(ctrl_stage)
            {
                case(PWM_CTRL_IDLE):
                {
                    if((sample_task_status == PROG_TASK_READY) && (navigate_task_status == PROG_TASK_READY))
                    {
                        ctrl_stage = PWM_CTRL_READY;
                        //indicate ready
                        set_speed_0(pwm_data_global);
                        set_direct_left(pwm_data_global);
                        sleep(1);
                        set_direct_right(pwm_data_global);
                        sleep(1);
                        set_direct_front(pwm_data_global);
                        cout << "[next ctrl_stage]: PWM_CTRL_READY" << endl;
                    }

                    break;
                }
                case(PWM_CTRL_READY):
                {
                    if(local_ctrl.work_mode == MANUAL_MODE)
                    {
                        ctrl_stage = MANUAL_MODE_INIT;
                    }
                    else if((local_ctrl.work_mode == AUTO_MODE) && (navigate_task_status == PROG_TASK_BEGIN))
                    {
                        ctrl_stage = AUTO_MODE_INIT;
                    }
                    else if((local_ctrl.work_mode == SAMPLE_MODE) && (sample_task_status == PROG_TASK_BEGIN))
                    {
                        ctrl_stage = SAMPLE_MODE_INIT;
                    }

                    break;
                }

                case(AUTO_MODE_INIT):
                {
                    ctrl_stage = AUTO_MODE_RUN;
                    //indicate auto mode run
                    set_speed_0(pwm_data_global);
                    set_direct_left(pwm_data_global);
                    sleep(1);
                    cout << "[next ctrl_stage]: AUTO_MODE_RUN" << endl;
                    break;
                }
                case(AUTO_MODE_RUN):
                {
                    if(navigate_task_status = PROG_TASK_END)
                    {
                        ctrl_stage = PWM_CTRL_READY;
                        set_speed_0(pwm_data_global);
                        set_direct_right(pwm_data_global);
                        sleep(1);
                    }

                    break;
                }
                case(MANUAL_MODE_INIT):
                {
                    ctrl_stage = MANUAL_MODE_RUN;
                    break;
                }
                case(MANUAL_MODE_RUN):
                {
                    if((local_ctrl.work_mode == AUTO_MODE) && (navigate_task_status == PROG_TASK_BEGIN))
                    {
                        ctrl_stage = AUTO_MODE_INIT;
                    }
                    else if((local_ctrl.work_mode == SAMPLE_MODE) && (sample_task_status == PROG_TASK_BEGIN))
                    {
                        ctrl_stage = SAMPLE_MODE_INIT;
                    }

                    memcpy(pwm_data_global, frame_data ,sizeof(frame_data));

                    break;
                }
                case(SAMPLE_MODE_INIT):
                {
                    ctrl_stage = SAMPLE_MODE_RUN;
                    set_speed_0(pwm_data_global);
                    set_direct_left(pwm_data_global);
                    sleep(1);
                    cout << "[next ctrl_stage]: SAMPLE_MODE_RUN" << endl;

                    break;
                }
                case(SAMPLE_MODE_RUN):
                {
                    if(sample_task_status = PROG_TASK_END)
                    {
                        ctrl_stage = PWM_CTRL_READY;
                        set_speed_0(pwm_data_global);
                        set_direct_right(pwm_data_global);
                        sleep(1);
                    }

                    break;
                }
                default:
                {
                    ctrl_stage = PWM_CTRL_READY;
                }
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }

    return 0;
}
