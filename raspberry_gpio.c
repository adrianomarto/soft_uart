
#include "raspberry_gpio.h"
#include <asm/io.h>

// For Raspberry Pi v2 and v3:
//#define BCM2708_BASE  0x3f000000

//For Raspberry Pi v1:
#define BCM2708_BASE  0x20000000

#define GPIO_BASE     (BCM2708_BASE + 0x200000)
#define GPIO_SIZE     4096

#define FIRST_GPIO_PIN  0
#define LAST_GPIO_PIN   53

volatile unsigned int* gpio_address = NULL;

/*
 * Initializes the BCM2708 interface.
 * This function will fail if the interface is already initialized.
 * @return 1 if the operation is successful. 0 otherwize.
 */
int raspberry_gpio_init(void)
{
  int success = 0;
  if (gpio_address == NULL)
  {
    gpio_address = ioremap(GPIO_BASE, GPIO_SIZE);
    success = gpio_address != NULL ? 1 : 0;
  }
  return success;
}

/*
 * Finalizes the BCM2708 interface.
 * This function will fail if the interface has not not initialized.
 * @return 1 if the operation is successful. 0 otherwise.
 */
int raspberry_gpio_finalize(void)
{
  int success = 0;
  if (gpio_address != NULL)
  {
    iounmap(gpio_address);
    gpio_address = NULL;
    success = 1;
  }
  return success;
}

/**
 * Sets a given gpio pin as input or output.
 * This function will fail if the interface has not not initialized.
 * @param gpio_pin given gpio pin
 * @param function input/output
 * @return 1 if the operation is successful. 0 otherwize.
 */
int raspberry_gpio_set_function(const int gpio_pin, const enum gpio_function function)
{
  int success = 0;
  if (gpio_address != NULL
    && gpio_pin >= FIRST_GPIO_PIN
    && gpio_pin <= LAST_GPIO_PIN)
  {
    *(gpio_address + (gpio_pin / 10)) &= ~(7 << ((gpio_pin % 10) * 3));
    if (function == gpio_output)
    {
      *(gpio_address + (gpio_pin / 10)) |= (1 << ((gpio_pin % 10) * 3));
    }
    success = 1;
  }
  return success;
}

/**
 * Gets the value of a given gpio input pin.
 * This function will fail if the interface has not not initialized. In that case, it returns 0.
 * @param gpio_pin given gpio input pin
 * @return gpio input pin value (0 or 1).
 */
int raspberry_gpio_get(const int gpio_pin)
{
  int value = 0;
  if (gpio_address != NULL
    && gpio_pin >= FIRST_GPIO_PIN
    && gpio_pin <= LAST_GPIO_PIN)
  {
    value = (*(gpio_address + 13) & (1 << gpio_pin)) != 0 ? 1 : 0;
  }
  return value;
}

/**
 * Sets the value of a given gpio output pin.
 * This function will fail if the interface has not not initialized.
 * @param gpio_pin given gpio output pin
 * @param value desired value (0 or 1)
 * @return 1 if the operation is successful. 0 otherwize.
 */
int raspberry_gpio_set(const int gpio_pin, const int value)
{
  int success = 0;
  if (gpio_address != NULL
    && gpio_pin >= FIRST_GPIO_PIN
    && gpio_pin <= LAST_GPIO_PIN)
  {
    if (value == 0)
    {
      *(gpio_address + 10) = 1 << gpio_pin;
    }
    else
    {
      *(gpio_address + 7) = 1 << gpio_pin;
    }
    success = 1;
  }
  return success;
}
