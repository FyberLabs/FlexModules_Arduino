/****************************************************************************************
    This is a library for the Fyber Labs mini EL Flex Module

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

#include "FyberLabs_MAX14521E.h"


FyberLabs_MAX14521E::FyberLabs_MAX14521E(uint8_t address) {
    _i2c_address = address;
}

void FyberLabs_MAX14521E::begin(void) {
    //Should setup a safe default configuration and check for device and complain if serial is up
    Wire.begin();
    reset();
}

//Safe configuration
void FyberLabs_MAX14521E::reset(void){
    //Turn off everything and set voltages low
    outputFrequency(0x00);
    outputWaveShape(0x00);
    outputWaveSlope(0x00);
    boostConvFrequency(0x00);
    boostConvModulation(0x00);
    audioEffectType(0x00);
    audioEffectELOff(1);
    audioEffectELOff(2);
    audioEffectELOff(3);
    audioEffectELOff(4);
    peakVoltage(1, 0x00);
    peakVoltage(2, 0x00);
    peakVoltage(3, 0x00);
    peakVoltage(4, 0x00);
    startTime(1, 0x00);
    startTime(2, 0x00);
    startTime(3, 0x00);
    startTime(4, 0x00);

    shutDown();
}

/*
  Pass address is physical representation of solder bridges as:
  MAX14521E_A0
  MAX14521E_A1
  Run for each bridge shorted
*/
uint8_t FyberLabs_MAX14521E::setSolderBridge(uint8_t address) {
    _i2c_address = _i2c_address | address;
    return _i2c_address;
}

uint8_t FyberLabs_MAX14521E::deviceID(void) {
    return read8(MAX14521E_REGISTER_DEVICE_ID);
}

void FyberLabs_MAX14521E::shutDown(void){
    write8(MAX14521E_REGISTER_POWERMODE, MAX14521E_SHUTDOWN);
}

void FyberLabs_MAX14521E::powerUp(void) {
    write8(MAX14521E_REGISTER_POWERMODE, MAX14521E_OPERATE);
}

/*
    inVariedScaleHz scaled frequencies are as:
        50-100Hz in ~ .5Hz increments from 0x00-0x3F
        100-200Hz in ~ 1Hz increments from 0x40-0x7F
        200-400Hz in ~ 2Hz increments from 0x80-0xBF
        400-800Hz in ~ 5Hz increments from 0xC0-0xFF
*/
void FyberLabs_MAX14521E::outputFrequency(uint8_t inVariedScaleHz) {
    write8(MAX14521E_REGISTER_OUT_FREQ, inVariedScaleHz);
}

/*
    Pass shape as:
        MAX14521E_SHAPE_FULLWAVE
        MAX14521E_SHAPE_HALFWAVE1
        MAX14521E_SHAPE_HALFWAVE2
*/    
void FyberLabs_MAX14521E::outputWaveShape(uint8_t shape) {
    if(shape <= MAX14521E_SHAPE_HALFWAVE2) {
        waveShape = shape;
        write8(MAX14521E_REGISTER_OUT_SHAPE, waveSlope + waveShape);
    }
}

/*
    Pass slope as:
        MAX14521E_SLOPE_SINEWAVE
        MAX14521E_SLOPE_FASTSLOPE
        MAX14521E_SLOPE_FASTERSLOPE
        MAX14521E_SLOPE_SQUAREWAVE
*/
void FyberLabs_MAX14521E::outputWaveSlope(uint8_t slope) {
    if(slope <= MAX14521E_SLOPE_SQUAREWAVE) {
        waveSlope = slope;
        write8(MAX14521E_REGISTER_OUT_SHAPE, waveSlope + waveShape);
    }
}

/*
    inHundredKHz range is
    400KHz-1600KHz - 0x00-0x0F
*/
void FyberLabs_MAX14521E::boostConvFrequency(uint8_t inHundredKHz){
    if(inHundredKHz <= 0x0F) {
        boostFreq = inHundredKHz;
    }
    write8(MAX14521E_REGISTER_BOOST_FREQ, boostFSW + boostFreq);
}

/*
    Pass fsw as:
        MAX14521E_BOOST_DISABLEFSW
        MAX14521E_BOOST_8THFSW
        MAX14521E_BOOST_32NDFSW
        MAX14521E_BOOST_128THFSW
*/
void FyberLabs_MAX14521E::boostConvModulation(uint8_t fsw) {
    if(fsw <= MAX14521E_BOOST_128THFSW) {
        boostFSW = fsw;
    }
    write8(MAX14521E_REGISTER_BOOST_FREQ, boostFSW + boostFreq);
}

/*
    Pass voltOrFreq as:
      MAX14521E_AUDIO_VOLTAGE
      MAX14521E_AUDIO_NOSAMPLE
      MAX14521E_AUDIO_FREQ16TH
      MAX14521E_AUDIO_FREQ8TH
      MAX14521E_AUDIO_FREQ4TH
      MAX14521E_AUDIO_FREQHALF
*/
void FyberLabs_MAX14521E::audioEffectType(uint8_t voltOrFreq) {
    //Merge Type and ELOns    
    audioType = voltOrFreq;
    audioEffectSet();
}

void FyberLabs_MAX14521E::audioEffectELOn(uint8_t EL) {
    if (EL > 0 && EL < 5) {
        audioELOn[EL-1] = 1;
    }
    audioEffectSet();
}

void FyberLabs_MAX14521E::audioEffectELOff(uint8_t EL) {
    if (EL > 0 && EL < 5) {
        audioELOn[EL-1] = 0;
    }
    audioEffectSet();
}

//Updates the audioEffects registers based on stored variables
void FyberLabs_MAX14521E::audioEffectSet(void) {
    uint8_t data;
    data = audioType;
    if(audioELOn[0]) {
        data += MAX14521E_AUDIO_EL1EN;
    }
    if(audioELOn[1]) {
        data += MAX14521E_AUDIO_EL2EN;
    }
    if(audioELOn[2]) {
        data += MAX14521E_AUDIO_EL3EN;
    }
    if(audioELOn[3]) {
        data += MAX14521E_AUDIO_EL4EN;
    }

    write8(MAX14521E_REGISTER_AUDIO_EFF, data);

}

//  voltage 0-150V 0x00-0x1F
void FyberLabs_MAX14521E::peakVoltage(uint8_t EL, uint8_t voltage) {
    uint8_t data;
    uint8_t ELin;

    if (EL > 0 && EL < 5 && voltage <= 0x1F) {
        ELin = EL - 1;
        data = voltage + ELPeakStart[ELin];
        ELPeakVoltage[ELin] = voltage;
        write8(MAX14521E_REGISTER_EL1_PEAK + ELin, data);
        updatePeak();
    }  //else report error
}

/*
    Pass startTime as
        MAX14521E_PEAK_MIN
        MAX14521E_PEAK_62
        MAX14521E_PEAK_125
        MAX14521E_PEAK_250
        MAX14521E_PEAK_500
        MAX14521E_PEAK_750
        MAX14521E_PEAK_1000
        MAX14521E_PEAK_2000
*/
void FyberLabs_MAX14521E::startTime(uint8_t EL, uint8_t timing) {
    uint8_t data;
    uint8_t ELin;

    if (EL > 0 && EL < 5 && timing <= MAX14521E_PEAK_2000) {
        ELin = EL - 1;
        data = timing + ELPeakVoltage[ELin];
        ELPeakStart[ELin] = timing;
        write8(MAX14521E_REGISTER_EL1_PEAK + ELin, data);
        updatePeak();
    }  //else report error
}

void FyberLabs_MAX14521E::updatePeak(void) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(MAX14521E_REGISTER_UPDATE_PEAK);
    Wire.endTransmission();
}

uint8_t FyberLabs_MAX14521E::read8(uint8_t reg) {
    uint8_t ret;

    Wire.beginTransmission(_i2c_address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)1);
    ret = Wire.read();

    return ret;
}

void FyberLabs_MAX14521E::write8(uint8_t reg, uint8_t data) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}