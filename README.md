# i2c-stepper-controller
Dual stepper controller with I2C interface (a4988 + stm8s103f3p6)

This device is a I2C slave and gets 6 bytes of control data:
1-2 = 16-bit signed integer(high part first), represents number of steps per second for the first stepper;
3-4 = 16-bit signed integer(high part first), represents number of steps per second for the second stepper;
5 = 8-bit unsigned integer, represents enable/disable state for both steppers (any non-zero means enabled);
6 = 8-bit unsigned integer, represents microstepping settings for both steppers (takes only 1, 2, 4, 8, 16, others ignored).

It is using 'stm8builder' build tool (http://github.com/grytole/stm8builder). To build this project you need to place 'stm8builder' folder here. 'make' will build it, 'make flash' will flash it.
