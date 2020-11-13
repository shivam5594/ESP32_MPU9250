## I2C master code

This code communicated ESP32 with MPU9250 over I2C bus. I am using ESP32-Cam board but code should work on any ESP32 board provided you change the pins for I2C accordingly. This code has parts of basic math and matrix code ported from STM HAL library according to needs.

## Current state

- I2C communication is estrablished and able to read and write registers.
- Working on Calibration, unable to calibrate.
- Wifi logging is enabled for Websocket, TCP and UDP.

## Future work

- Add SPI support

## How to use
`git clone --recurse-submodules https://github.com/shivam5594/ESP32_MPU9250`
