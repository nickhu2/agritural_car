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

#include <sstream>

#include "pwm_ctrl/common_macro.h"

#define DEV_NAME  "/dev/ttyUSB0"

#define MAX_BUF_SIZE		(80)
#define FRAME_LEN			(25)
#define FRAME_END_VALUE		(0x0)
#define FRAME_START_VALUE	(0x0F)
#define WORD_BAUDRATE		(28800)

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


#define WORK_MODE_DEFAULT   (MANUAL_MODE)
#define WORK_STATUS_DEFAULT (PAUSE)
#define MAX_CHANNELS        (16)
#define WORK_MODE_CHANNEL   (6)
#define WORK_STATUS_CHANNEL (5)
#define SPEED_CHANNEL       (4)
#define DIRECT_CHANNEL      (3)

int sbus_config(int sbus_fd)
{
    struct termios options;

    if (tcgetattr(sbus_fd, &options) != 0) {
        return -1;
    }

    tcflush(sbus_fd, TCIFLUSH);
    bzero(&options, sizeof(options));

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= PARENB;
    options.c_cflag &= ~PARODD;
    options.c_iflag |= INPCK;
    options.c_cflag |= CSTOPB;

    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;

    cfsetispeed(&options, B38400);
    cfsetospeed(&options, B38400);

    tcflush(sbus_fd, TCIFLUSH);

    if ((tcsetattr(sbus_fd, TCSANOW, &options)) != 0) {
        return -1;
    }

    int baud = 100000;
    struct serial_struct serials;

    if ((ioctl(sbus_fd, TIOCGSERIAL, &serials)) < 0) {
        return -1;
    }

    serials.flags = ASYNC_SPD_CUST;
    serials.custom_divisor = serials.baud_base / baud;

    if ((ioctl(sbus_fd, TIOCSSERIAL, &serials)) < 0) {
        return -1;
    }

    ioctl(sbus_fd, TIOCGSERIAL, &serials);

    tcflush(sbus_fd, TCIFLUSH);
    return 0;
}

static int uart_send(int fd,void *buf, int len)
{
    int ret = 0;
    int count = 0;

    tcflush(fd, TCIFLUSH);

    while (len > 0) 
    {

        ret = write(fd, (char*)buf + count, len);
        if (ret < 1) 
        {
            break;
        }
        count += ret;
        len = len - ret;
    }

    return count;
}

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

static int decode_value(int fd, char* buf, int len, uint8_t *frame_data, ctrl_desc_t *desc)
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
                    desc->work_mode = WORK_MODE_DEFAULT;
                else
                    desc->work_mode = AUTO_MODE;

                desc->status = (out[WORK_STATUS_CHANNEL - 1] < WORK_STATUS_THRESHOLD1) ? PAUSE : WORKING;
                desc->speed_pwm_info = out[SPEED_CHANNEL - 1];
                desc->direction_pwm_info = out[DIRECT_CHANNEL - 1];
                
                //uart_send(fd, frame_data, sizeof(frame_data));

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


int set_opt(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;

    /*获取原有串口配置*/
    if  ( tcgetattr( fd,&oldtio)  !=  0) { 
        perror("SetupSerial 1");
        return -1;
    }
    memset( &newtio, 0, sizeof(newtio) );
    /*CREAD 开启串行数据接收，CLOCAL并打开本地连接模式*/
    newtio.c_cflag  |=  CLOCAL | CREAD;

    /*设置数据位*/
    newtio.c_cflag &= ~CSIZE;
    switch( nBits )
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    }
    /* 设置奇偶校验位 */
    switch( nEvent )
    {
    case 'O':
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP);
        break;
    case 'E': 
        newtio.c_iflag |= (INPCK | ISTRIP);
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        break;
    case 'N':  
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* 设置波特率 */
    switch( nSpeed )
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /*设置停止位*/
    if( nStop == 1 )/*设置停止位；若停止位为1，则清除CSTOPB，若停止位为2，则激活CSTOPB*/
        newtio.c_cflag &=  ~CSTOPB;/*默认为一位停止位； */
    else if ( nStop == 2 )
        newtio.c_cflag |=  CSTOPB;
    /*设置最少字符和等待时间，对于接收字符和等待时间没有特别的要求时*/
    newtio.c_cc[VTIME]  = 0;/*非规范模式读取时的超时时间；*/
    newtio.c_cc[VMIN] = 0;/*非规范模式读取时的最小字符数*/
    /*tcflush清空终端未完成的输入/输出请求及数据；TCIFLUSH表示清空正收到的数据，且不读取出来 */
    tcflush(fd,TCIFLUSH);
    if((tcsetattr(fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}



int uart_recv_timeout(int uart_fd, void *buf, int len, int timeout_ms)
{
    int ret;
    size_t  rsum = 0;
    ret = 0;
    fd_set rset;
    struct timeval t;

    while (rsum < len)
    {
        t.tv_sec = timeout_ms/1000;
        t.tv_usec = (timeout_ms - t.tv_sec*1000)*1000;
        FD_ZERO(&rset);
        FD_SET(uart_fd, &rset);
        ret = select(uart_fd+1, &rset, NULL, NULL, &t);
        if (ret <= 0) {
            if (ret == 0) 
            {
                //timeout
                return -1;
            }
            if (errno == EINTR) 
            {
                // 信号中断
                continue;
            }
            return -errno;
        } 
        else
        {
            ret = read(uart_fd, (char *)buf + rsum, len - rsum);
            if (ret < 0)
            {
                return ret;
            }
            else
            {
                rsum += ret;
            }
        }
    }

    return rsum;
}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Publisher chatter_pub = n.advertise<std_msgs::UInt64>(RC_CTRL_INFO, 1000);
    ros::Rate loop_rate(LOOP_RATE_DEFAULT);

    int count = 0;
    char buf[MAX_BUF_SIZE] = {0};

	//int	fd = open(DEV_NAME, O_RDWR | O_NOCTTY);
    int fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);
	int len = 0;
    if(fd < 0)
	{
        printf("[info] open uart failed\n");
		perror(DEV_NAME);
		return -1;
    }
    printf("[info] open uart successfully\n");

    int ret = sbus_config(fd);

    if(ret != 0)
    {
        printf("config uart error\n");
    }
    else
    {
        printf("config uart OK\n");
    }



    while (ros::ok())
    {
        std_msgs::UInt64 msg;

        len = uart_recv_timeout(fd, buf, sizeof(buf), 100);
        //printf("[info] read bytes size: %d\n", len);

    	if(len > 0)
        {
            ctrl_desc_t local_ctrl;
            uint8_t  frame_data[FRAME_LEN];

            if(decode_value(fd, buf, len, frame_data, &local_ctrl) == 0)
            {
                if(local_ctrl.work_mode == MANUAL_MODE)
                {
                    //in mannul mode, sendout sbus data as RC input
                    uart_send(fd, frame_data, sizeof(frame_data));
                }
                else
                {
                    //TODO: ctrl speed & direction as program in auto mode
                    uart_send(fd, frame_data, sizeof(frame_data));
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
