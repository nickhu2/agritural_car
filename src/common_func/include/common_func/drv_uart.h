#ifndef DRV_UART_H__
#define DRV_UART_H__


#define MAX_BUF_SIZE		(80)
#define FRAME_LEN			(25)
#define FRAME_END_VALUE		(0x0)
#define FRAME_START_VALUE	(0x0F)
#define WORD_BAUDRATE		(28800)


int uart_recv_timeout(void *buf, int len, int timeout_ms);


int uart_send(void *buf, int len);


int set_opt(int nSpeed, int nBits, char nEvent, int nStop);

int uart_init(void);


#endif
