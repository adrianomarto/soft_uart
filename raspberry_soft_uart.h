#ifndef RASPBERRY_SOFT_UART_H
#define RASPBERRY_SOFT_UART_H

#include <linux/tty.h>
#include "queue.h"

int  raspberry_soft_uart_init(const int gpio_tx, const int gpio_rx);
int  raspberry_soft_uart_finalize(void);
int  raspberry_soft_uart_open(struct tty_struct* tty);
int  raspberry_soft_uart_close(void);
int  raspberry_soft_uart_set_baudrate(const int baudrate);
struct queue* raspberry_soft_uart_get_tx_queue(void);

#endif
