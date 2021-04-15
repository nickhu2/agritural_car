#ifndef COMMON_MACRO_H__
#define COMMON_MACRO_H__

#define PROG_PWM_SERVO_VALUE	"/prog/pwm_servo"
#define PROG_PWM_MOTOR_VALUE	"/prog/pwm_motor"

#define RC_CTRL_INFO			"/rc_control_info"

typedef enum
{
    MANUAL_MODE = 0,
    AUTO_MODE = 1,
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



#endif //COMMON_MACRO_H__

