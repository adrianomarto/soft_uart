#ifndef RASPBERRY_GPIO_H
#define RASPBERRY_GPIO_H

enum gpio_function
{
  gpio_input,
  gpio_output
};

int raspberry_gpio_init(void);
int raspberry_gpio_finalize(void);
int raspberry_gpio_set_function(const int gpio_pin, const enum gpio_function function);
int raspberry_gpio_get(const int gpio_pin);
int raspberry_gpio_set(const int gpio_pin, const int value);

#endif
