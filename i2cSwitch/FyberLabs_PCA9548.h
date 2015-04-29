/****************************************************************************************
    This is a library for the Fyber Labs i2c Switch Flex Module

    Copyright (c) 2015 Chris Hamilton


    Permission is hereby granted, free of charge, to any person obtaining a copy of this
    software and associated documentation files (the "Software"), to deal in the Software
    without restriction, including without limitation the rights to use, copy, modify,
    merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
    permit persons to whom the Software is furnished to do so, subject to the following
    conditions:

    The above copyright notice and this permission notice shall be included in all copies
    or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
    INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
    PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
    CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
    OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*****************************************************************************************/

#ifndef FYBERLABS_PCA9548_H
#define FYBERLABS_PCA9548_H

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include "Wire.h"

#define PCA9548_I2CADDR 0x70
#define PCA9548_A2 0x04
#define PCA9548_A1 0x02
#define PCA9548_A0 0x01

class FyberLabs_PCA9548 {
  public:
    FyberLabs_PCA9548(uint8_t address = PCA9548_I2CADDR);
    boolean begin();
    uint8_t readSwitchChannel(void);

    void offAllSwitchChannels(void);
    void onSwitchChannels(uint8_t channels);
    void offSwitchChannel(uint8_t channel);
    void onSwitchChannel(uint8_t channel);
    uint8_t setSolderBridge(uint8_t address);

  private:
    uint8_t _i2c_address;
    
    uint8_t read8();
    void write8(uint8_t data);
};

#endif  //  FYBERLABS_PCA9548_H