# Makefile for stm8builder build tool
# http://github.com/grytole/stm8builder
# You need to place 'stm8builder' folder here to compile

PROJECT = controller
SOURCES = main.c
DEFINES = I2C_ADDRESS=0x0F
CHIP    = STM8S103
CHIPMOD = F3
FLASHER = stlinkv2

include stm8builder/stm8builder.mk
