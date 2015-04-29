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

#include "FyberLabs_PCA9548.h"

FyberLabs_PCA9548::FyberLabs_PCA9548(uint8_t address) {
  _i2c_address = address;
}

boolean FyberLabs_PCA9548::begin() {
  Wire.begin();
  offAllSwitchChannels();
  return true;
}

uint8_t FyberLabs_PCA9548::readSwitchChannel(void) {
  return read8();
}

//Each channel is a bit, so shift 1 by the channel number
void FyberLabs_PCA9548::onSwitchChannels(uint8_t channels) {
  write8(channels);
}

//Bit
void FyberLabs_PCA9548::onSwitchChannel(uint8_t channel) {
  uint8_t channels;
  channels = readSwitchChannel();
  channels |= (1 << (channel-1));
  onSwitchChannels(channels);
}

void FyberLabs_PCA9548::offSwitchChannel(uint8_t channel) {
  uint8_t channels;
  channels = readSwitchChannel();
  channels &= ~(1 << (channel-1));
  onSwitchChannels(channels);
}

void FyberLabs_PCA9548::offAllSwitchChannels() {
  onSwitchChannels(0x00);
}

//Pass defined solder bridge hex to create the correct address
uint8_t FyberLabs_PCA9548::setSolderBridge(uint8_t address) {
  _i2c_address = _i2c_address | address;
  return _i2c_address;
}

//read/write structure borrowed from Adafruit
uint8_t FyberLabs_PCA9548::read8() {
  uint8_t ret;

  Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)1);// send data n-bytes read
#if (ARDUINO >= 100)
  ret = Wire.read(); // receive DATA
#else
  ret = Wire.receive(); // receive DATA
#endif

  return ret;
}

void FyberLabs_PCA9548::write8(uint8_t data) {
  Wire.beginTransmission(_i2c_address); // start transmission to device 
#if (ARDUINO >= 100)
  Wire.write(data);  // write data
#else
  Wire.send(data);  // write data
#endif
  Wire.endTransmission();
}