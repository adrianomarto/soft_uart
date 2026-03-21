# soft_uart

Software-based serial port module for Raspberry Pi.

This module creates a software-based serial port using a configurable pair of GPIO pins. The serial port will appear as `/dev/ttySOFT0`.


## Features

* Works exactly as a hardware-based serial port.
* Works with any application, e.g. cat, echo, minicom.
* Configurable baud rate.
* TX buffer of 256 bytes.
* RX buffer managed by the kernel.


## Compiling

Fetch the source:
```
git clone https://github.com/adrianomarto/soft_uart
```

Install the package `raspberrypi-kernel-headers`:
```
sudo apt-get install raspberrypi-kernel-headers
```

Run `make` and `make install`, as usual.
```
cd soft_uart
make
sudo make install
```

I haven't tried cross-compiling this module, but it should work as well.


## Loading

Module parameters:

* gpio_tx: int [default = 529]
* gpio_rx: int [default = 539]

Loading the module with default parameters:
```
sudo insmod soft_uart.ko
```

Look up GPIO number using the following command, for setting up pins other than the default.
```
cat /sys/kernel/debug/gpio
```

Loading module with custom parameters with (GPIO17:529) as TX and (GPIO27:539) as RX:
```
sudo insmod soft_uart.ko gpio_tx=529 gpio_rx=539
```


## Usage

The device will appear as `/dev/ttySOFT0`. Use it as any usual TTY device.

You must be included in the group `dialout`. You can verify in what groups you are included by typing `groups`. To add an user to the group `dialout`, type:
```
sudo usermod -aG dialout <username>
```

Usage examples:
```
minicom -b 4800 -D /dev/ttySOFT0
cat /dev/ttySOFT0
echo "hello" > /dev/ttySOFT0
```

## Baud rate

When choosing the baud rate, take into account that:
* The Raspberry Pi is not very fast.
* You will probably not be running a real-time operating system.
* There will be other processes competing for CPU time.

As a result, you can expect communication errors when using fast baud rates. So I would not try to go any faster than 4800 bps.
