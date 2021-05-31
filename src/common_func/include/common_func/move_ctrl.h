#ifndef MOVE_CTRL_H__
#define MOVE_CTRL_H__


#include <stdio.h>
#include <string>
#include <stdlib.h>
#include "ros/ros.h"


#define PROG_PWM_SERVO_VALUE			"/prog/pwm_servo"
#define PROG_PWM_MOTOR_VALUE			"/prog/pwm_motor"
#define PROG_SAMPLE_STATUS_TOPIC		"/prog/task_sample_status"
#define PROG_NAV_STATUS_TOPIC			"/prog/task_nav_status"
#define RC_CTRL_INFO					"/rc_control_info"


#define PROG_TASK_READY			(0x1)
#define PROG_TASK_BEGIN			(0x2)
#define PROG_TASK_END			(0x3)
#define PROG_TASK_UNKONW		(0xFF)


typedef enum
{
    MANUAL_MODE = 0,
    AUTO_MODE = 1,
    SAMPLE_MODE = 2,
} WORK_MODE_t;

typedef enum
{
    PAUSE = 0,
    WORKING = 1,
} WORK_STATUS_t;

typedef struct
{
    WORK_MODE_t         work_mode;              //come from channel6 (RC channel4)
    WORK_STATUS_t       status;                 //come from channel5 (RC channel3)
    uint16_t            speed_pwm_info;         //come from channel4 (RC channel2)
    uint16_t            direction_pwm_info;     //come from channel3 (RC channel1)
} ctrl_desc_t;


int32_t set_direct_front(uint8_t *frame_data);
int32_t set_direct_left(uint8_t *frame_data);
int32_t set_direct_right(uint8_t *frame_data);
int32_t set_speed_0(uint8_t *frame_data);

#endif //MOVE_CTRL_H__
