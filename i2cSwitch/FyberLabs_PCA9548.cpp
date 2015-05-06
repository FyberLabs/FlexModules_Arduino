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

/*
  Switching for more than one channel.  To turn on only 
  a specific channel then all other channels must be
  turned off.

  Maybe used to broadcast writes to multiple devices with
  the same address on each bus channel.  Only select one
  channel/device to read from at a time.
*/


FyberLabs_PCA9548::FyberLabs_PCA9548(uint8_t address) {
  _i2c_address = address;
}

boolean FyberLabs_PCA9548::begin() {
  Wire.begin();
  //offAllSwitchChannels();
  return true;
}

//Create an instance for each switch in the system
uint8_t FyberLabs_PCA9548::readSwitchChannel(void) {
  return read8();
}

//Enter the raw hex for the channels to be on
void FyberLabs_PCA9548::onSwitchChannels(uint8_t channels) {
  write8(channels);
}

//Turn on a specific channel 1-8
void FyberLabs_PCA9548::onSwitchChannel(uint8_t channel) {
  uint8_t channels;
  channels = readSwitchChannel();
  channels |= (1 << (channel-1)); //Bit ops at 0, so subtract 1 for channels
  onSwitchChannels(channels);
}

//Turn off a specific channel 1-8
void FyberLabs_PCA9548::offSwitchChannel(uint8_t channel) {
  uint8_t channels;
  channels = readSwitchChannel();
  channels &= ~(1 << (channel-1)); //Bit ops at 0, so subtract 1 for channels
  onSwitchChannels(channels);
}

//Turn off all switches
void FyberLabs_PCA9548::offAllSwitchChannels() {
  onSwitchChannels(0x00);
}

//Pass defined solder bridge hex to create the correct address
uint8_t FyberLabs_PCA9548::setSolderBridge(uint8_t address) {
  _i2c_address = _i2c_address | address;
  return _i2c_address;
}

uint8_t FyberLabs_PCA9548::read8() {
  uint8_t ret;

  Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)1);
  ret = Wire.read();

  return ret;
}

void FyberLabs_PCA9548::write8(uint8_t data) {
  Wire.beginTransmission(_i2c_address);
  Wire.write(data);
  Wire.endTransmission();
}