/****************************************************************************************
    This is a library for the Fyber Labs DAout Flex Module i2sToHeadphone

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

// Enables headphone output only from i2s digital input
// Based on usage examples in http://www.ti.com/lit/ug/slau456/slau456.pdf
//

//Example Register Setup to Play Digital Data Through DAC and Headphone Output
# I2C Script to Setup the device in Playback Mode
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# This script set DAC output routed to only HP Driver
#

#include "FyberLabs_TAS2521.h"

using namespace TAS2521;

setup()
{
    FyberLabs_TAS2521 DAout;

    // Assert Software reset (P0, R1, D0=1)
    DAout.begin();

    // LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
    DAout.setLDOControl(LDO_1_8V);

    // CODEC_CLKIN=MCLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=00)
    //   W 30 04 00
    DAout.setPLLCLK(PLL_CLKIN_MCLK):
    DAout.setCODECCLK(CODEC_CLKIN_MCLK);

    // DAC NDAC Powered up, NDAC=1 (P0, R11, D7=1, D6-D0=0000001)
    //   W 30 0B 81
    DAout.setNDACCLK(0x01);
    DAout.setNDACPowerUp();

    // DAC MDAC Powered up, MDAC=2 (P0, R12, D7=1, D6-D0=0000010)
    //   W 30 0C 82
    DAout.setMDACCLK(0x02);
    DAout.setMDACPowerUp();

    // DAC OSR(9:0)-> DOSR=128 (P0, R13, D1-D0=00)
    //   W 30 0D 00
    DAout.setDACOSRMSB(0x00);

    // DAC OSR(9:0)-> DOSR=128 (P0, R14, D7-D0=10000000)
    //   W 30 0E 80
    DAout.setDACOSRLSB(0x80);

    // Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode.
    //   (P0, R27, D7-D6=00, D5-D4=00, D3-D2=00) W 30 1B 00
    DAout.setAudioDataWordLength(BIT16);
    DAout.setAudioBCLKin();
    DAout.setAudioWCLKin();
    DAout.setAudioInterfaceSelect(I2S);

    // Data slot offset 00 (P0, R28, D7-D0=0000)
    //   W 30 1C 00
    DAout.setAudioBCLKDataOffset(0x00);

    // Dac Instruction programming PRB #2 for Mono routing. Type interpolation (x8) and 3 programmable
    //   Biquads. (P0, R60, D4-D0=0010) W 30 3C 02
    DAout.setDACInstructionSet(PRB_P2);

    // Master Reference Powered on (P1, R1, D4=1)
    DAout.setMasterReferencePowerUp();

    // Output common mode for DAC set to 0.9V (default) (P1, R10)
    //   W 30 0A 00
    DAout.setFullChipCommonMode09V();

    // DAC output is routed directly to HP driver (P1, R12, D3=1)
    //   w 30 0C 08
    DAout.setHPOUTDACRoutedDirect();

    // HP Volume, 0dB Gain (P1, R22, D6-D0=0000000)
    //   W 30 16 00
    DAout.setHPOUTVolume(0);

    // Power up HP (P1, R9, D5=1)
    //   w 30 09 20
    DAout.setOutputHPLPowerUp();

    // Unmute HP with 0dB gain (P1, R16, D4=1)
    //   w 30 10 00
    DAout.setHPOUTDriverGain(0x00);
    DAout.setHPOUTDriverUnmuted();

    // DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
    //   W 30 3F 90
    DAout.setDACChannelVolumeControl(0x00);
    DAout.setDACChannelSetupDataPath(DATA_FROM_LEFT);
    DAout.setDACChannelSetupPowerUp();

    // DAC digital gain 0dB (P0, R65, D7-D0=00000000)
    //   W 30 41 00
    DAout.setDACChannelDigitalVolumeControl(0x00);

    // DAC volume not muted. (P0, R64, D3=0, D2=1)
    //   W 30 40 04
    DAout.setDACChannelUnmute();

}

loop()
{

}
