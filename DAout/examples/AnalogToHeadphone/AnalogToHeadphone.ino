/****************************************************************************************
    This is a library for the Fyber Labs DAout Flex Module AnalogToHeadphone

    Copyright (c) 2015 Fyber Labs Inc.


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

// Enables headphone output only from analog input
// Based on usage examples in http://www.ti.com/lit/ug/slau456/slau456.pdf
//

//Example Register Setup to Play AINL and AINR Through Headphone Output
// I2C Script to Setup the device in Playback Mode
// This script set AINL and AINR inputs routed to only HP Driver
// Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY

#include "FyberLabs_TAS2521.h"

using namespace TAS2521;

setup()
{
    FyberLabs_TAS2521 DAout;

    // Assert Software reset (P0, R1, D0=1)
    DAout.begin();

    // LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
    DAout.setLDOControl(LDO_1_8V);

    // Master Reference Powered on (P1, R1, D4=1)
    DAout.setMasterReferencePowerUp();

    // Enable AINL and AINR (P1, R9, D1-D0=11) w 30 09 03
    DAout.setAINLInputOn();
    DAout.setAINRInputOn();

    // AINL/R to HP driver not via Mixer P (P1, R12, D1-D0=11) w 30 0C 03
    DAout.setHPOUTAINLAttenuator();
    DAout.setHPOUTAINRAttenuator();

    // HP Volume, 0dB Gain (P1, R22, D6-D0=0000000) W 30 16 00
    DAout.setHPOUTVolume(0);

    // Not enable HP Out Mixer, AINL Volume, 0dB Gain (P1, R24, D7=0, D6-D0=0000000)
    //  W 30 18 00
    DAout.setAINLRMixerPandMixerMForceOff();
    DAout.setAINLVolume(0x00);

    // Enable AINL and AINR and Power up HP (P1, R9, D5=1, D1-D0=11) w 30 09 23
    DAout.setOutputHPLPowerUp();
    DAout.setAINLInputOn();
    DAout.setAINRInputOn();

    // Unmute HP with 0dB gain (P1, R16, D4=1) w 30 10 00
    DAout.setHPOUTDriverGain(0x00);
    DAout.setHPOUTDriverUnmuted();

}

loop()
{

}
