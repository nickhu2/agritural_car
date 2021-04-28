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

#include "common_func/move_ctrl.h"
#include "common_func/drv_uart.h"

#define DEV_NAME  "/dev/ttyUSB0"

static int uart_fd = -1;

int uart_recv_timeout(void *buf, int len, int timeout_ms)
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


int uart_send(void *buf, int len)
{
    int ret = 0;
    int count = 0;

    tcflush(uart_fd, TCIFLUSH);

    while (len > 0) 
    {

        ret = write(uart_fd, (char*)buf + count, len);
        if (ret < 1) 
        {
            break;
        }
        count += ret;
        len = len - ret;
    }

    return count;
}




int set_opt(int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio,oldtio;

    /*获取原有串口配置*/
    if  ( tcgetattr( uart_fd,&oldtio)  !=  0) { 
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
    tcflush(uart_fd,TCIFLUSH);
    if((tcsetattr(uart_fd,TCSANOW,&newtio))!=0)
    {
        perror("com set error");
        return -1;
    }
    return 0;
}


static int sbus_config(int sbus_fd)
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


int uart_init(void)
{
    uart_fd = open(DEV_NAME, O_RDWR | O_NONBLOCK);

    if(uart_fd < 0)
	{
        printf("[info] open uart failed\n");
		perror(DEV_NAME);
		return -1;
    }
    printf("[info] open uart successfully\n");

    int ret = sbus_config(uart_fd);

    if(ret != 0)
    {
        printf("config uart error\n");
    }
    else
    {
        printf("config uart OK\n");
    }
}