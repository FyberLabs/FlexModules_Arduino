/****************************************************************************************
    This is a library for the Fyber Labs DAout Flex Module i2sToSpeaker6biquads

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

// Enables speaker and headphone output from i2s digital input with 6 digital biquad filters
// Based on usage examples in http://www.ti.com/lit/ug/slau456/slau456.pdf
//

//Example Register Setup to Play Digital Data Through DAC and Headphone/Speaker Outputs with 6 programmable Biquads
// I2C Script to Setup the device in Playback Mode #3
// Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
// This script set DAC output routed to HP Driver and Class-
// D driver via Mixer with 6 programmable Biquads.

#include "FyberLabs_TAS2521.h"

using namespace TAS2521;

setup()
{
    FyberLabs_TAS2521 DAout;

    // Assert Software reset (P0, R1, D0=1)
    DAout.begin();

    // LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
    DAout.setLDOControl(LDO_1_8V);

    // PLL_clkin = MCLK, codec_clkin = PLL_CLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=03)
    //   w 30 04 00
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

    // DAC OSR(9:0)-> DOSR=128 (P0, R12, D1-D0=00)
    //   W 30 0D 00
    DAout.setDACOSRMSB(0x00);

    // DAC OSR(9:0)-> DOSR=128 (P0, R13, D7-D0=10000000)
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

    // Dac Instruction programming PRB #2 for Mono routing. Type interpolation
    //   (x8) and 3 programmable Biquads. (P0, R60, D4-D0=0010) W 30 3C 02
    DAout.setDACInstructionSet(PRB_P2);

##########--------------- BEGIN COEFFICIENTS --------------------------------------
# reg 00 - Page Select Register = 46
# sets active page to page 46 for First-Order IIR
w 30 00 2E
#-----------------------------------------------------------------------
#  First-Order IIR = 100Hz HP
#-----------------------------------------------------------------------
# reg 28/29/30 - N0 Coefficient
w 30 1C 7F 18 36
# reg 32/33/34 - N1 Coefficient
w 30 20 80 E7 CA
# reg 36/37/38 - N2 Coefficient
w 30 24 7E 30 6D
# reg 00 - Page Select Register = 44
# sets active page to page 44 for 6-BQs (BQ-A, BQ-B, BQ-C, BQ-D, BQ-E, BQ-F) w 30 00 2C
#
#-----------------------------------------------------------------------
# BQ-A=500HzNotchBW=25
#-----------------------------------------------------------------------
# reg 12/13/14 - N0 Coefficient
w 30 0C 7F C5 BD
# reg 16/17/18 - N1 Coefficient
w 30 10 80 8D 39
# reg 20/21/22 - N2 Coefficient
w 30 14 7F C5 BD
# reg 24/25/26 - D1 Coefficient
w 30 18 7F 72 C7
# reg 28/29/30 - D2 Coefficient
w 30 1C 80 74 84
#-----------------------------------------------------------------------
# BQ-B=1KHzNotchBW=25
#-----------------------------------------------------------------------
# reg 32/33/34 - N0 Coefficient
w 30 20 7F C5 BD
# reg 36/37/38 - N1 Coefficient
w 30 24 81 85 B1
# reg 40/41/42 - N2 Coefficient
w 30 28 7F C5 BD
# reg 44/45/46 - D1 Coefficient
w 30 2C 7E 7A 4F
# reg 48/49/50 - D2 Coefficient
w 30 30 80 74 84
#-----------------------------------------------------------------------
# BQ-C=2KHzNotchBW=25
#-----------------------------------------------------------------------
# reg 52/53/54 - N0 Coefficient
w 30 34 7F C5 BD
# reg 56/57/58 - N1 Coefficient
w 30 38 85 61 46
# reg 60/61/62 - N2 Coefficient
w 30 3C 7F C5 BD
# reg 64/65/66 - D1 Coefficient
w 30 40 7A 9E BA
# reg 68/69/70 - D2 Coefficient
w 30 44 80 74 84
#-----------------------------------------------------------------------
# BQ-D=3KHzNotchBW=25
#-----------------------------------------------------------------------
# reg 72/73/74 - N0 Coefficient
w 30 48 7F C5 BD
# reg 76/77/78 - N1 Coefficient
w 30 4C 8B B8 FD
# reg 80/81/82 - N2 Coefficient
w 30 50 7F C5 BD
# reg 84/85/86 - D1 Coefficient
w 30 54 74 47 03
# reg 88/89/90 - D2 Coefficient
w 30 58 80 74 84
#-----------------------------------------------------------------------
# BQ-E=4KHzNotchBW=25
#-----------------------------------------------------------------------
# reg 92/93/94 - N0 Coefficient
w 30 5C 7F C5 BD
# reg 96/97/98 - N1 Coefficient
w 30 60 94 6B EF
# reg 100/101/102 - N2 Coefficient
w 30 64 7F C5 BD
# reg 104/105/106 - D1 Coefficient
w 30 68 6B 94 11
# reg 108/109/110 - D2 Coefficient
w 30 6C 80 74 84
#-----------------------------------------------------------------------
# BQ-F=5KHzNotchBW=25
#-----------------------------------------------------------------------
# reg 112/113/114 - N0 Coefficient
w 30 70 7F C5 BD
# reg 116/117/118 - N1 Coefficient
w 30 74 9F 4C FB
# reg 120/121/122 - N2 Coefficient
w 30 78 7F C5 BD
# reg 124/125/126 - D1 Coefficient
w 30 7C 60 B3 05
# sets active page to page 45 for BQ-F D2
w 30 00 2D
# reg 8/9/10 - D2 Coefficient
w 30 08 80 74 84
##########--------------- END COEFFICIENTS OF Notch Filters  ------------------------
#######################################################

    // Master Reference Powered on (P1, R1, D4=1)
    DAout.setMasterReferencePowerUp();

    // Output common mode for DAC set to 0.9V (default) (P1, R10)
    //   W 30 0A 00
    DAout.setFullChipCommonMode09V();

    // Mixer P output is connected to HP Out Mixer (P1, R12, D2=1)
    //   w 30 0C 04
    DAout.setHPOUTRouting(NO_ANALOG_ROUTING);
    DAout.setHPOUTMixerPAttenuator();

    // HP Volume, 0dB Gain (P1, R22, D6-D0=0000000) W 30 16 00
    DAout.setHPOUTVolume(0);

    // Power up HP (P1, R9, D5=1)
    //   w 30 09 20
    DAout.setOutputHPLPowerUp();

    // Unmute HP with 0dB gain (P1, R16, D4=1)
    //   w 30 10 00
    DAout.setHPOUTDriverGain(0x00);
    DAout.setHPOUTDriverUnmuted();

    // SPK attn. Gain =0dB (P1, R46, D6-D0=000000) W 30 2E 00
    DAout.setSpeakerVolume(0x00);

    // SPK driver Gain=6.0dB (P1, R48, D6-D4=001) W 30 30 10
    DAout.setSpeakerAmplifierVolume(SPK_VOL_6DB)

    // SPK powered up (P1, R45, D1=1) W 30 2D 02
    DAout.setSpeakerDriverPowerUp();

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
