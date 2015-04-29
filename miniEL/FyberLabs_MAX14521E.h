/*
  Designed by Chris Hamilton
*/

#ifndef FYBERLABS_MAX14521E_H
#define FYBERLABS_MAX14521E_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define MAX14521E_DEBUG 0
#define MAX14521E_I2CADDR 0x78
#define MAX14521E_SUBADR1 0x2
#define MAX14521E_SUBADR2 0x3
#define MAX14521E_SUBADR3 0x4
//Address 0x90-0x9E, 0x30-0x3E, 0xC0-0xDE, 0xF0-0xFE
  // 

class Fyberlabs_ELDriver {
  public:
  	Fyberlabs_ELDriver(uint8_t addr = MAX14521E_I2CADDR);
  	void begin(void);
  	void reset(void);

  private:
  	uint8_t _i2caddr;

  	uint8_t read8(uint8_t addr);
  	void write8(uint8_t addr, uint8_t d);

};

#endif
//0x00 Device ID
//0x01 Power Mode
//  0 shutdown, 1 Operate
//0x02 EL Output Frequency
//  100Hz-800Hz
//0x03 Slope/Shape
//  00 Sine Wave
//  01 Fast Slope
//  10 Faster Slope
//  11 Square Wave
//  00 Full Wave
//  10 Half Wave (pause in half wave)
//  11 Half Wave (pause in full wave)
//0x04 Boost Converter Frequency
//  Spread spectrum modulation disable, 1/8 FSW, 1/32 FSW, 1/128 FSW
//  400KHz-1600KHz
//0x05 Audio Effects
//  Voltage or Frequency (AUX divided by 16,8,4,2)
//  EL 1,2,3,4
//  No Sample
//0x06 Load data into EL1 peak voltage
//  Soft start time: min, 62.5, 125, 250, 500, 750, 1000, 2000msec
//  0-150V
//0x07 EL2 peak voltage
//0x08 EL3 peak voltage
//0x09 EL4 peak voltage
//0x0A Update peak voltage output registers