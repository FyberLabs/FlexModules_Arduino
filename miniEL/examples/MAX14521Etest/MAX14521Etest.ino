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


#include <Wire.h>
#include "FyberLabs_MAX14521E.h"

FyberLabs_MAX14521E miniEL = FyberLabs_MAX14521E();

void setup() {
  miniEL.begin();
  

  miniEL.outputFrequency(0xFF);
  miniEL.outputWaveShape(miniEL.MAX14521E_SHAPE_FULLWAVE);
  miniEL.outputWaveSlope(miniEL.MAX14521E_SLOPE_SQUAREWAVE);
  miniEL.boostConvFrequency(0x00);
  miniEL.boostConvModulation(miniEL.MAX14521E_BOOST_32NDFSW);
  
  miniEL.startTime(1, miniEL.MAX14521E_PEAK_MIN);
  miniEL.startTime(2, miniEL.MAX14521E_PEAK_MIN);
  miniEL.startTime(3, miniEL.MAX14521E_PEAK_MIN);
  miniEL.startTime(4, miniEL.MAX14521E_PEAK_MIN);

  
}

//This will create a chasing pattern on a 3 input chasing EL wire
void loop() {
  miniEL.powerUp();

  int i=0;
  int j=0;
  for(j=0; j<10000; j++) {
    for(i=1; i<4; i++) {
      miniEL.peakVoltage(i, 0x1F);
      delay(50);
      miniEL.peakVoltage(i, 0x00);
    }
  }
  //miniEL.shutDown();
  delay(50000);
}
