/// This file is for defining options/enabling settings.
#ifndef OPTIONS_H
#define OPTIONS_H
#define USE_POSITION_ESTIMATION_FILTER true
// This sets the speed of I2C/Fifo communication.
#define I2C_SPEED 400000
// I have no knowledge of what this does, but I could infer that this is the speed for how fast it reads from serial.
#define serialBaudRate 115200 
#endif