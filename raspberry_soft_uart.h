#ifndef RASPBERRY_SOFT_UART_H
#define RASPBERRY_SOFT_UART_H

#include <linux/tty.h>
#include <linux/gpio/consumer.h>

int raspberry_soft_uart_init(struct gpio_desc *_gpio_tx, struct gpio_desc *_gpio_rx);
int raspberry_soft_uart_finalize(void);
int raspberry_soft_uart_open(struct tty_struct* tty);
int raspberry_soft_uart_close(void);
int raspberry_soft_uart_set_baudrate(const int baudrate);
int raspberry_soft_uart_send_string(const unsigned char* string, int string_size);
int raspberry_soft_uart_get_tx_queue_room(void);
int raspberry_soft_uart_get_tx_queue_size(void);
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char));

#endif
