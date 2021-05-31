#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include <termios.h>
#include <errno.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>
#include "ros/ros.h"
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>



#define SBUS_PACK_LEN	(25)
#define SBUS_HEADER		(0x0F)

#define FRONT_PWM_VALUE	(947)
#define LEFT_PWM_VALUE	(200)
#define RIGHT_PWM_VALUE	(1800)


#define STOP_PWM_VALUE	(1012)


static void direction_pwm_pack(uint16_t pwm_value, uint8_t *frame_data)
{
	frame_data[5] =  frame_data[5] | ((pwm_value >> 10) &0x1);
	frame_data[4] =  frame_data[4] | ((pwm_value >> 2) & 0xFF);
	frame_data[3] =  frame_data[3] | ((pwm_value << 6) & 0xC0);

	frame_data[0] = SBUS_HEADER;
	frame_data[23] = 0;
	frame_data[24] =0;

}

static void speed_pwm_pack(uint16_t pwm_value, uint8_t *frame_data)
{
	frame_data[6] =  frame_data[6] | ((pwm_value >> 7) &0x0F);
	frame_data[5] =  frame_data[5] | ((pwm_value << 1) & 0xFF);

	frame_data[0] = SBUS_HEADER;
	frame_data[23] = 0;
	frame_data[24] =0;

}


int32_t set_direct_front(uint8_t *frame_data)
{
	direction_pwm_pack(FRONT_PWM_VALUE, frame_data);
	
	return 0;
}


int32_t set_direct_left(uint8_t *frame_data)
{
	direction_pwm_pack(LEFT_PWM_VALUE, frame_data);
	return 0;
}


int32_t set_direct_right(uint8_t *frame_data)
{
	direction_pwm_pack(RIGHT_PWM_VALUE, frame_data);
	return 0;
}


int32_t set_speed_0(uint8_t *frame_data)
{
	speed_pwm_pack(STOP_PWM_VALUE, frame_data);

	return 0;
}