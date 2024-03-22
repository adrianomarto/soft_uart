
#include "raspberry_soft_uart.h"
#include "queue.h"

#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>

static irqreturn_t handle_rx_start(int irq, void *device);
static enum hrtimer_restart handle_tx(struct hrtimer* timer);
static enum hrtimer_restart handle_rx(struct hrtimer* timer);
static void receive_character(unsigned char character);

static struct queue queue_tx;
static struct tty_struct* current_tty = NULL;
static DEFINE_MUTEX(current_tty_mutex);
static struct hrtimer timer_tx;
static struct hrtimer timer_rx;
static ktime_t period;
static ktime_t half_period;
static struct gpio_desc *gpio_tx;
static struct gpio_desc *gpio_rx;
static int rx_bit_index = -1;
static void (*rx_callback)(unsigned char) = NULL;

/**
 * Initializes the Raspberry Soft UART infrastructure.
 * This must be called during the module initialization.
 * The GPIO pin used as TX is configured as output.
 * The GPIO pin used as RX is configured as input.
 * @param gpio_tx GPIO pin used as TX
 * @param gpio_rx GPIO pin used as RX
 * @return 1 if the initialization is successful. 0 otherwise.
 */
int raspberry_soft_uart_init(struct gpio_desc *_gpio_tx, struct gpio_desc *_gpio_rx)
{
  bool success = true;
  
  mutex_init(&current_tty_mutex);
  
  // Initializes the TX timer.
  hrtimer_init(&timer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  timer_tx.function = &handle_tx;
  
  // Initializes the RX timer.
  hrtimer_init(&timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
  timer_rx.function = &handle_rx;
  
  // Initializes the GPIO pins.
  gpio_tx = _gpio_tx;
  gpio_rx = _gpio_rx;
  
  // Initializes the interruption.
  success &= request_irq(
    gpiod_to_irq(gpio_rx),
    handle_rx_start,
    IRQF_TRIGGER_FALLING,
    "soft_uart_irq_handler",
    NULL) == 0;
  disable_irq(gpiod_to_irq(gpio_rx));
    
  return success;
}

/**
 * Finalizes the Raspberry Soft UART infrastructure.
 */
int raspberry_soft_uart_finalize(void)
{
  free_irq(gpiod_to_irq(gpio_rx), NULL);
  gpiod_set_value(gpio_tx, 0);
  gpiod_put(gpio_tx);
  gpiod_put(gpio_rx);
  return 1;
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
  rx_bit_index = -1;
  if (current_tty == NULL)
  {
    current_tty = tty;
    initialize_queue(&queue_tx);
    success = 1;
    enable_irq(gpiod_to_irq(gpio_rx));
  }
  mutex_unlock(&current_tty_mutex);
  return success;
}

/**
 * Closes the Soft UART.
 */
int raspberry_soft_uart_close(void)
{
  mutex_lock(&current_tty_mutex);
  disable_irq(gpiod_to_irq(gpio_rx));
  hrtimer_cancel(&timer_tx);
  hrtimer_cancel(&timer_rx);
  current_tty = NULL;
  mutex_unlock(&current_tty_mutex);
  return 1;
}

/**
 * Sets the Soft UART baudrate.
 * @param baudrate desired baudrate
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_soft_uart_set_baudrate(const int baudrate) 
{
  period = ktime_set(0, 1000000000/baudrate);
  half_period = ktime_set(0, 1000000000/baudrate/2);
  gpiod_set_debounce(gpio_rx, 1000/baudrate/2);
  return 1;
}

/**
 * Adds a given string to the TX queue.
 * @paran string given string
 * @param string_size size of the given string
 * @return The amount of characters successfully added to the queue.
 */
int raspberry_soft_uart_send_string(const unsigned char* string, int string_size)
{
  int result = enqueue_string(&queue_tx, string, string_size);
  
  // Starts the TX timer if it is not already running.
  if (!hrtimer_active(&timer_tx))
  {
    hrtimer_start(&timer_tx, period, HRTIMER_MODE_REL);
  }
  
  return result;
}

/*
 * Gets the number of characters that can be added to the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_room(void)
{
  return get_queue_room(&queue_tx);
}

/*
 * Gets the number of characters in the TX queue.
 * @return number of characters.
 */
int raspberry_soft_uart_get_tx_queue_size(void)
{
  return get_queue_size(&queue_tx);
}

/**
 * Sets the callback function to be called on received character.
 * @param callback the callback function
 */
int raspberry_soft_uart_set_rx_callback(void (*callback)(unsigned char))
{
	rx_callback = callback;
	return 1;
}

//-----------------------------------------------------------------------------
// Internals
//-----------------------------------------------------------------------------

/**
 * If we are waiting for the RX start bit, then starts the RX timer. Otherwise,
 * does nothing.
 */
static irqreturn_t handle_rx_start(int irq, void *device)
{
  if (rx_bit_index == -1)
  {
    hrtimer_start(&timer_rx, half_period, HRTIMER_MODE_REL);
  }
  return IRQ_HANDLED;
}


/**
 * Dequeues a character from the TX queue and sends it.
 */
static enum hrtimer_restart handle_tx(struct hrtimer* timer)
{
  ktime_t current_time = ktime_get();
  static unsigned char character = 0;
  static int bit_index = -1;
  enum hrtimer_restart result = HRTIMER_NORESTART;
  bool must_restart_timer = false;
  
  // Start bit.
  if (bit_index == -1)
  {
    if (dequeue_character(&queue_tx, &character))
    {
      gpiod_set_value(gpio_tx, 0);
      bit_index++;
      must_restart_timer = true;
    }
  }
  
  // Data bits.
  else if (0 <= bit_index && bit_index < 8)
  {
    gpiod_set_value(gpio_tx, 1 & (character >> bit_index));
    bit_index++;
    must_restart_timer = true;
  }
  
  // Stop bit.
  else if (bit_index == 8)
  {
    gpiod_set_value(gpio_tx, 1);
    character = 0;
    bit_index = -1;
    must_restart_timer = get_queue_size(&queue_tx) > 0;
  }
  
  // Restarts the TX timer.
  if (must_restart_timer)
  {
    hrtimer_forward(&timer_tx, current_time, period);
    result = HRTIMER_RESTART;
  }
  
  return result;
}

/*
 * Receives a character and sends it to the kernel.
 */
static enum hrtimer_restart handle_rx(struct hrtimer* timer)
{
  ktime_t current_time = ktime_get();
  static unsigned int character = 0;
  int bit_value = gpiod_get_value(gpio_rx);
  enum hrtimer_restart result = HRTIMER_NORESTART;
  bool must_restart_timer = false;
  
  // Start bit.
  if (rx_bit_index == -1)
  {
    rx_bit_index++;
    character = 0;
    must_restart_timer = true;
  }
  
  // Data bits.
  else if (0 <= rx_bit_index && rx_bit_index < 8)
  {
    if (bit_value == 0)
    {
      character &= 0xfeff;
    }
    else
    {
      character |= 0x0100;
    }
    
    rx_bit_index++;
    character >>= 1;
    must_restart_timer = true;
  }
  
  // Stop bit.
  else if (rx_bit_index == 8)
  {
    receive_character(character);
    rx_bit_index = -1;
  }
  
  // Restarts the RX timer.
  if (must_restart_timer)
  {
    hrtimer_forward(&timer_rx, current_time, period);
    result = HRTIMER_RESTART;
  }
  
  return result;
}

/**
 * Adds a given (received) character to the RX buffer, which is managed by the kernel,
 * and then flushes (flip) it.
 * @param character given character
 */
void receive_character(unsigned char character)
{
  mutex_lock(&current_tty_mutex);
  if (rx_callback != NULL) {
	  (*rx_callback)(character);
  } else {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    if (current_tty != NULL && current_tty->port != NULL)
    {
      tty_insert_flip_char(current_tty->port, character, TTY_NORMAL);
      tty_flip_buffer_push(current_tty->port);
    }
#else
    if (tty != NULL)
    {
      tty_insert_flip_char(current_tty, character, TTY_NORMAL);
      tty_flip_buffer_push(tty);
    }
#endif
  }
  mutex_unlock(&current_tty_mutex);
}
