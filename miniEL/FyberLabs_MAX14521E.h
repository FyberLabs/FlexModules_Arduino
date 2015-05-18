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

/*
    TODO
    move argument enum to define?
    move private stuff
    move around documentation better
    ??implement register readers - should we also set internal variables from them?
*/

#ifndef FYBERLABS_MAX14521E_H
#define FYBERLABS_MAX14521E_H


#include "Arduino.h"
#include "Wire.h"

#define MAX14521E_DEBUG 0
#define MAX14521E_I2CADDR 0x78
#define MAX14521E_A0 0x2
#define MAX14521E_A1 0x4

class FyberLabs_MAX14521E {
  public:
  	FyberLabs_MAX14521E(uint8_t address = MAX14521E_I2CADDR);

    typedef enum
    {
      MAX14521E_REGISTER_DEVICE_ID           = 0x00,
      MAX14521E_REGISTER_POWERMODE           = 0x01,
      MAX14521E_REGISTER_OUT_FREQ            = 0x02,
      MAX14521E_REGISTER_OUT_SHAPE           = 0x03,
      MAX14521E_REGISTER_BOOST_FREQ          = 0x04,
      MAX14521E_REGISTER_AUDIO_EFF           = 0x05,
      MAX14521E_REGISTER_EL1_PEAK            = 0x06,
      MAX14521E_REGISTER_EL2_PEAK            = 0x07,
      MAX14521E_REGISTER_EL3_PEAK            = 0x08,
      MAX14521E_REGISTER_EL4_PEAK            = 0x09,
      MAX14521E_REGISTER_UPDATE_PEAK         = 0x0A,
    } MAX14521ERegisters_t;

    //0x00 Device ID
    //0x01 Power Mode
    //  0 shutdown, 1 Operate
    typedef enum {
      MAX14521E_SHUTDOWN    = 0x00,
      MAX14521E_OPERATE     = 0x01,
    } MAX14521EPower_t;

    //0x02 EL Output Frequency - 50Hz-800Hz with 4 scales:
    //  50-100Hz in ~ .5Hz increments from 0x00-0x3F
    //  100-200Hz in ~ 1Hz increments from 0x40-0x7F
    //  200-400Hz in ~ 2Hz increments from 0x80-0xBF
    //  400-800Hz in ~ 5Hz increments from 0xC0-0xFF


    //0x03 Slope/Shape
    //  00 Sine Wave
    //  01 Fast Slope
    //  10 Faster Slope
    //  11 Square Wave
    uint8_t waveSlope;

    typedef enum {
      MAX14521E_SLOPE_SINEWAVE    = 0x00,
      MAX14521E_SLOPE_FASTSLOPE   = 0x01,
      MAX14521E_SLOPE_FASTERSLOPE = 0x02,
      MAX14521E_SLOPE_SQUAREWAVE  = 0x03,
    } MAX14521ESlope_t;
    //  +
    //  00 Full Wave
    //  10 Half Wave (pause in half wave)
    //  11 Half Wave (pause in full wave)
    uint8_t waveShape;

    typedef enum {
      MAX14521E_SHAPE_FULLWAVE    = 0x00,
      MAX14521E_SHAPE_HALFWAVE1   = 0x08,
      MAX14521E_SHAPE_HALFWAVE2   = 0x0C,
    } MAX14521EShape_t;
    
    //0x04 Boost Converter Frequency
    //  Spread spectrum modulation disable, 1/8 FSW, 1/32 FSW, 1/128 FSW
    //  400KHz-1600KHz - 0x00-0x0F + :
    uint8_t boostFreq;
    uint8_t boostFSW;

    typedef enum {
      MAX14521E_BOOST_DISABLEFSW  = 0x00,
      MAX14521E_BOOST_8THFSW      = 0x40,
      MAX14521E_BOOST_32NDFSW     = 0x80,
      MAX14521E_BOOST_128THFSW    = 0xC0,
    } MAX14521EBoost_t;

    //0x05 Audio Effects
    //  Voltage or Frequency (AUX divided by 16,8,4,2)
    //  EL 1,2,3,4
    //  No Sample
    //  Add ELxEN with specific effect
    uint8_t audioELOn[4];
    uint8_t audioType;

    typedef enum {
      MAX14521E_AUDIO_VOLTAGE    = 0x00,
      MAX14521E_AUDIO_NOSAMPLE   = 0x40,
      MAX14521E_AUDIO_FREQ16TH   = 0xC0,
      MAX14521E_AUDIO_FREQ8TH    = 0xD0,
      MAX14521E_AUDIO_FREQ4TH    = 0xE0,
      MAX14521E_AUDIO_FREQHALF   = 0xF0,
      MAX14521E_AUDIO_EL1EN      = 0x01,
      MAX14521E_AUDIO_EL2EN      = 0x02,
      MAX14521E_AUDIO_EL3EN      = 0x04,
      MAX14521E_AUDIO_EL4EN      = 0x08,
    } MAX14521EAudio_t;

    //0x06 Load data into EL1 peak voltage
    //0x07 EL2 peak voltage
    //0x08 EL3 peak voltage
    //0x09 EL4 peak voltage
    //Soft start time: min, 62.5, 125, 250, 500, 750, 1000, 2000msec
    uint8_t ELPeakStart[4];

    typedef enum {
      MAX14521E_PEAK_MIN        = 0x00,
      MAX14521E_PEAK_62         = 0x20,
      MAX14521E_PEAK_125        = 0x40,
      MAX14521E_PEAK_250        = 0x60,
      MAX14521E_PEAK_500        = 0x80,
      MAX14521E_PEAK_750        = 0xA0,
      MAX14521E_PEAK_1000       = 0xC0,
      MAX14521E_PEAK_2000       = 0xE0,
    } MAX14521EPeak_t;

    //+ 0-150V 0x00-0x1F
    uint8_t ELPeakVoltage[4];

    //0x0A Update peak voltage output registers (send after any peak voltage updates)


  	void begin(void);
  	void reset(void);

    uint8_t setSolderBridge(uint8_t bridge);

    uint8_t deviceID(void);
    void shutDown(void);
    void powerUp(void);
    void outputFrequency(uint8_t inVariedScaleHz);
    
    void outputWaveShape(uint8_t shape);
    void outputWaveSlope(uint8_t slope);
    
    void boostConvFrequency(uint8_t inHundredKHz);
    void boostConvModulation(uint8_t fsw);

    void audioEffectType(uint8_t voltOrFreq);
    void audioEffectELOn(uint8_t EL);
    void audioEffectELOff(uint8_t EL); //No sample?


    void peakVoltage(uint8_t EL, uint8_t voltage);
    void startTime(uint8_t EL, uint8_t timing);
    void updatePeak(void);


  private:
  	uint8_t _i2c_address;

    void audioEffectSet(void);

  	uint8_t read8(uint8_t reg);
  	void write8(uint8_t reg, uint8_t data);

};

#endif
