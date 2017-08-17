
#include "raspberry_soft_uart.h"
#include "raspberry_gpio.h"

#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

static struct queue tx_queue;
static struct tty_struct* current_tty = NULL;
static DEFINE_MUTEX(current_tty_mutex);
static unsigned long baudrate = 4800;
static int gpio_tx = 0;
static int gpio_rx = 0;

static struct hrtimer timer;
static enum hrtimer_restart timer_callback(struct hrtimer* timer);
static void handle_tx(void);
static void handle_rx(void);
static void push_character_to_kernel(unsigned char character);

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pin used as TX
 * @param gpio_rx GPIO pin used as RX
 * @return 1 if the initialization is successful. 0 otherwise.
 */
int raspberry_soft_uart_init(const int _gpio_tx, const int _gpio_rx)
{
  int success = 0;
  mutex_init(&current_tty_mutex);
  
  // Initializes the timer.
  hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  timer.function = &timer_callback;
  
  // Initializes the GPIO pins.
  if (raspberry_gpio_init())
  {
    gpio_tx = _gpio_tx;
    gpio_rx = _gpio_rx;
    raspberry_gpio_set_function(gpio_tx, gpio_output);
    raspberry_gpio_set_function(gpio_rx, gpio_input);
    raspberry_gpio_set(gpio_tx, 1);
    success = 1;
  }
  
  return success;
}

/**
 * Finalizes the Raspberry Soft UART infrastructure.
 */
int raspberry_soft_uart_finalize(void)
{
  return raspberry_gpio_finalize();
}

/**
 * Opens the Soft UART.
 * @param tty
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_open(struct tty_struct* tty)
{
  int success = 0;
  mutex_lock(&current_tty_mutex);
  if (current_tty == NULL)
  {
    current_tty = tty;
    initialize_queue(&tx_queue);
    hrtimer_start(&timer, ktime_set(0, 0), HRTIMER_MODE_REL);
    success = 1;
  }
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(void)
{
  int success = 0;
  mutex_lock(&current_tty_mutex);
  if (current_tty != NULL)
  {
    hrtimer_cancel(&timer);
    current_tty = NULL;
    success = 1;
  }
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Sets the Soft UART baudrate.
 * @param baudrate desired baudrate
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_set_baudrate(const int _baudrate) 
{
  baudrate = _baudrate;
  return 1;
}

/**
 * Gets the TX queue.
 * @return TX queue (never returns NULL)
 */
struct queue* raspberry_soft_uart_get_tx_queue(void)
{
  return &tx_queue;
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

/**
 * Handles the TX and RX scheduled by a given timer.
 * @param timer given timer.
 */
static enum hrtimer_restart timer_callback(struct hrtimer* timer)
{
  ktime_t current_time = ktime_get();
  ktime_t interval = ktime_set(0, 1000000000/baudrate);
    
  handle_tx();
  handle_rx();
    
  hrtimer_forward(timer, current_time, interval);
  return HRTIMER_RESTART;
}

/**
 * Dequeues a character from the TX queue and sends it.
 */
static void handle_tx(void)
{
  static unsigned char character = 0;
  static int bit_index = -1;
  
  // Start bit.
  if (bit_index == -1)
  {
    if (dequeue_character(&tx_queue, &character))
    {
      raspberry_gpio_set(gpio_tx, 0);
      bit_index++;
    }
  }
  
  // Data bits.
  else if (0 <= bit_index && bit_index < 8)
  {
    raspberry_gpio_set(gpio_tx, 1 & (character >> bit_index));
    bit_index++;
  }
  
  // Stop bit.
  else if (bit_index == 8)
  {
    raspberry_gpio_set(gpio_tx, 1);
    character = 0;
    bit_index = -1;
  }
}

/*
 * Receives a character and sends it to the kernel.
 */
static void handle_rx(void)
{
  static unsigned int character = 0;
  static int bit_index = -1;
  
  int bit_value = raspberry_gpio_get(gpio_rx);
  
  // Start bit.
  if (bit_value == 0 && bit_index == -1)
  {
    character = 0;
    bit_index++;
  }
  
  // Data bits.
  else if (0 <= bit_index && bit_index < 8)
  {
    if (bit_value == 0)
    {
      character &= 0xfeff;
    }
    else
    {
      character |= 0x0100;
    }
    
    character >>= 1;
    bit_index++;
  }
  
  // Stop bit.
  else if (bit_index == 8)
  {
    push_character_to_kernel(character);
    bit_index = -1;
  }
}

/**
 * Pushes a given (received) character into the kernel buffer.
 * @param character given character
 */
static void push_character_to_kernel(unsigned char character)
{
  mutex_lock(&current_tty_mutex);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
  if (current_tty != NULL && current_tty->port != NULL)
  {
    tty_insert_flip_char(current_tty->port, character, TTY_NORMAL);
    tty_flip_buffer_push(current_tty->port);
  }
#else
  if (tty != NULL)
  {
    if (tty->flip.count >= TTY_FLIPBUF_SIZE)
    {
      tty_flip_buffer_push(tty);
    }
    tty_insert_flip_char(current_tty, character, TTY_NORMAL);
  }
#endif
  mutex_unlock(&current_tty_mutex);
}
