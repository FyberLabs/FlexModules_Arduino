/****************************************************************************************
    This is a library for the Fyber Labs DAout Flex Module

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

//The TAS2521 contains several pages of 8-bit registers, and each page can contain up to 128 registers. The register pages are divided up based on functional blocks for this device. Page 0 is the default home page after RST. Page control is done by writing a new page value into register 0 of the current page.
//Pages legal: 0, 1, 44-52, 62-70, and 152-169
//All registers are 8 bits in width, with D7 referring to the most-significant bit of each register, and D0 referring to the least-significant b

/*
Summary of Register Map	Page Number	Description
0	Control Registers, Page 0 (Default Page): Clock Multipliers, Dividers, Serial Interfaces, Flags, Interrupts, and GPIOs. See Section 5.1.1.
1	Control Registers, Page 1: DAC Routing, Power-Controls and MISC Logic Related Programmabilities. See Section 5.1.2.
8 - 43	Page 8 - 43: Reserved Registers
44	Page 44: DAC Programmable Coefficients RAM. See Section 5.1.4 and Section 5.1.11.
45 - 52	Page 45 - 52: DAC Programmable Coefficients RAM. See Section 5.1.5 and Section 5.1.11.
53 - 61	Page 53 - 61: Reserved Registers
62 - 70	Page 62 - 70: DAC Programmable Coefficients RAM. See Section 5.1.7 and Section 5.1.11.
71 -151	Page 71 - 151: Reserved Registers
152 -169	Page 152 - 169: DAC Programmable Instruction RAM. See Section 5.1.9Section 5.1.13.
170 -255	Page 170 - 255: Reserved Registers
*/

#include "FyberLabs_TAS2521.h"
namespace TAS2521{

  void FyberLabs_TAS2521::switchPage(uint8_t page) {
    _page = page;
    write8(0x00,page);
  }


  uint8_t FyberLabs_TAS2521::read8(uint8_t reg) {
    Wire.beginTransmission(_i2c_address);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)1);
    return Wire.read();
  }

  void FyberLabs_TAS2521::write8(uint8_t reg, uint8_t data) {
      Wire.beginTransmission(_i2c_address);
      Wire.write(reg);
      Wire.write(data);
      Wire.endTransmission();
  }

  void FyberLabs_TAS2521::begin(void) {
    P0R4_t P0R4;
    P0R5_t P0R5;

    Wire.begin();

    switchPage(0);

    //Soft reset
    write8(0x01,0x01);

    //use default configuration
    write8(P0R4.GetAddress(),P0R4.toByte());
    write8(P0R5.GetAddress(),P0R5.toByte());
  }

  void FyberLabs_TAS2521::setPLLCLKRangeLow(void) {
    if(_page !=0)
      switchPage(0);

    P0R4_t P0R4(read8(4));
    if(P0R4.getPLLRange()!=PLL_RANGE_LOW)
    {
      P0R4.setPLLRange(PLL_RANGE_LOW);
      write8(P0R4.GetAddress(),P0R4.toByte());
    }
  }

  void FyberLabs_TAS2521::setPLLCLKRangeHigh(void) {
    if(_page !=0)
          switchPage(0);

    P0R4_t P0R4(read8(4));
    if(P0R4.getPLLRange()!=PLL_RANGE_HIGH)
    {
      P0R4.setPLLRange(PLL_RANGE_HIGH);
      write8(P0R4.GetAddress(),P0R4.toByte());
    }
  }

  /*
	  Select PLL Input Clock
  00: MCLK pin is input to PLL
  01: BCLK pin is input to PLL
  10: GPIO pin is input to PLL
  11: DIN pin is input to PLL
  */
  void FyberLabs_TAS2521::setPLLCLK(PLL_CLKIN_t clk) {
    if(_page !=0)
      switchPage(0);

    P0R4_t P0R4(read8(4));
    if(P0R4.getPLLClk()!=clk)
    {
      P0R4.setPLLClk(clk);
      write8(P0R4.GetAddress(),P0R4.toByte());
    }
  }
  /*
	  Select CODEC_CLKIN
  00: MCLK pin is CODEC_CLKIN
  01: BCLK pin is CODEC_CLKIN
  10: GPIO pin is CODEC_CLKIN
  11: PLL Clock is CODEC_CLKIN
  */
  void FyberLabs_TAS2521::setCODECCLK(CODEC_CLKIN_t clk) {
    if(_page !=0)
      switchPage(0);

    P0R4_t P0R4(read8(P0R4.GetAddress()));
    if(P0R4.getCodecClk()!=clk)
    {
      P0R4.setCodecClk(clk);
      write8(P0R4.GetAddress(),P0R4.toByte());
    }
  }

  void FyberLabs_TAS2521::setPLLPowerDown(void) {
    if(_page !=0)
      switchPage(0);

    P0R5_t P0R5(read8(P0R5.GetAddress()));
    P0R5.setPLLPower(POWER_DOWN);
    write8(P0R5.GetAddress(),P0R5.toByte());
  }

  void FyberLabs_TAS2521::setPLLPowerUp(void) {
    if(_page !=0)
      switchPage(0);

    P0R5_t P0R5(read8(P0R5.GetAddress()));
    P0R5.setPLLPower(POWER_UP);
    write8(P0R5.GetAddress(),P0R5.toByte());
  }

  /*
  000: PLL divider P = 8
  001: PLL divider P = 1
  010: PLL divider P = 2
  ...
  110: PLL divider P = 6
  111: PLL divider P = 7
  */
  void FyberLabs_TAS2521::setPLLDividerP(PLL_DIV_P_t P) {
    if(_page !=0)
      switchPage(0);

    P0R5_t P0R5(read8(P0R5.GetAddress()));
    P0R5.setPLLDivP(P);
    write8(P0R5.GetAddress(),P0R5.toByte());
  }

  /*
  0000: Reserved. Do not use
  0001: PLL multiplier R = 1
  0010: PLL multiplier R = 2
  0011: PLL multipler R = 3
  0100: PLL multipler R = 4
  ...
  0101...0111: Reserved. Do not use
  */
  void FyberLabs_TAS2521::setPLLDividerR(PLL_MULT_R_t R) {
    if(_page !=0)
      switchPage(0);

    P0R5_t P0R5(read8(P0R5.GetAddress()));
    P0R5.setPLLMultR(R);
    write8(P0R5.GetAddress(),P0R5.toByte());
  }

  /*
	  PLL divider J value
  00 0000...00 0011: Do not use 00 0100: J = 4
  00 0101: J = 5
  ...
  11 1110: J = 62
  11 1111: J = 63
  */
  void FyberLabs_TAS2521::setPLLDividerJ(uint8_t J) {
    if(_page !=0)
      switchPage(0);
    P0R6_t P0R6(read8(P0R6.GetAddress()));
    P0R6.setPLLDivJ(J);
    write8(P0R6.GetAddress(),P0R6.toByte());
  }

  /*
	  PLL divider D value (MSB)
  PLL divider D value(MSB) and PLL divider D value(LSB)
  00 0000 0000 0000: D=0000
  00 0000 0000 0001: D=0001
  ...
  10 0111 0000 1110: D=9998
  10 0111 0000 1111: D=9999
  10 0111 0001 0000...11 1111 1111 1111: Do not use
  Note: This register will be updated only when the Page-0, Reg-8 is written immediately after Page-0, Reg-7.
  */
  void FyberLabs_TAS2521::setPLLDividerDMSB(uint8_t DMSB) {
    if(_page !=0)
      switchPage(0);
    P0R7_t P0R7(read8(P0R7.GetAddress()));
    P0R7.setPLLDivD_MSB(DMSB);
    write8(P0R7.GetAddress(),P0R7.toByte());
  }

  /*
	  PLL divider D value (LSB)
  PLL divider D value(MSB) and PLL divider D value(LSB)
  00 0000 0000 0000: D=0000
  00 0000 0000 0001: D=0001
  ...
  10 0111 0000 1110: D=9998
  10 0111 0000 1111: D=9999
  10 0111 0001 0000...11 1111 1111 1111: Do not use
  Note: Page-0, Reg-8 should be written immediately after Page-0, Reg-7.
  */
  void FyberLabs_TAS2521::setPLLDividerDLSB(uint8_t DLSB) {
    if(_page !=0)
      switchPage(0);
    P0R8_t P0R8(read8(P0R8.GetAddress()));
    P0R8.setPLLDivD_LSB(DLSB);
    write8(P0R8.GetAddress(),P0R8.toByte());
  }

  void FyberLabs_TAS2521::setNDACPowerDown(void) {
    if(_page !=0)
      switchPage(0);
    P0R11_t reg(read8(reg.GetAddress()));
    reg.setNDACPower(POWER_DOWN);
    write8(reg.GetAddress(),reg.toByte());
  }

  void FyberLabs_TAS2521::setNDACPowerUp(void) {
    if(_page !=0)
      switchPage(0);
    P0R11_t P0R11(read8(P0R11.GetAddress()));
    P0R11.setNDACPower(POWER_UP);
    write8(P0R11.GetAddress(),P0R11.toByte());
  }
  /*
	  NDAC Value
  000 0000: NDAC=128
  000 0001: NDAC=1
  000 0010: NDAC=2
  ...
  111 1110: NDAC=126
  111 1111: NDAC=127
  Note: Please check the clock frequency requirements in the Overview section.
  */
  void FyberLabs_TAS2521::setNDACCLK(uint8_t clk) {
    if(_page !=0)
      switchPage(0);
    P0R11_t P0R11(read8(P0R11.GetAddress()));
    P0R11.setNDACValue(clk);
    write8(P0R11.GetAddress(),P0R11.toByte());
  }

  void FyberLabs_TAS2521::setMDACPowerDown(void) {
    if(_page !=0)
      switchPage(0);
    P0R12_t P0R12(read8(P0R12.GetAddress()));
    P0R12.setMDACPower(POWER_DOWN);
    write8(P0R12.GetAddress(),P0R12.toByte());
  }
  void FyberLabs_TAS2521::setMDACPowerUp(void) {
    if(_page !=0)
      switchPage(0);
    P0R12_t P0R12(read8(P0R12.GetAddress()));
    P0R12.setMDACPower(POWER_UP);
    write8(P0R12.GetAddress(),P0R12.toByte());
  }
  /*
	  MDAC Value
  000 0000: MDAC=128
  000 0001: MDAC=1
  000 0010: MDAC=2
  ...
  111 1110: MDAC=126
  111 1111: MDAC=127
  Note: Please check the clock frequency requirements in the Overview section.
  */
  void FyberLabs_TAS2521::setMDACCLK(uint8_t clk) {
    if(_page !=0)
      switchPage(0);
    P0R12_t P0R12(read8(P0R12.GetAddress()));
    P0R12.setMDACValue(clk);
    write8(P0R12.GetAddress(),P0R12.toByte());
  }

  /*
	  DAC OSR (DOSR) MSB Setting
  DAC OSR(MSB) and DAC OSR (LSB)
  00 0000 0000: DOSR=1024
  00 0000 0001: DOSR=1
  00 0000 0010: DOSR=2
  ...
  11 1111 1110: DOSR=1022
  11 1111 1111: DOSR=1023
  Note: This register is updated when Page-0, Reg-14 is written to immediately after Page-0, Reg-13.
  */
  void FyberLabs_TAS2521::setDACOSRMSB(uint8_t DOSRMSB) {
    if(_page !=0)
      switchPage(0);
    P0R13_t P0R13(read8(P0R13.GetAddress()));
    P0R13.setDacOsrMsb(DOSRMSB);
    write8(P0R13.GetAddress(),P0R13.toByte());
  }

  /*
	  DAC OSR (DOSR) LSB Setting
  DAC OSR(MSB) and DAC OSR (LSB)
  00 0000 0000: DOSR=1024
  00 0000 0001: DOSR=1
  00 0000 0010: DOSR=2
  ...
  11 1111 1110: DOSR=1022
  11 1111 1111: DOSR=1023
  Note: This register should be written immediately after Page-0, Reg-13.
  */
  void FyberLabs_TAS2521::setDACOSRLSB(uint8_t DOSRLSB) {
    if(_page !=0)
      switchPage(0);
    P0R14_t P0R14(read8(P0R14.GetAddress()));
    P0R14.setDacOsrLsb(DOSRLSB);
    write8(P0R14.GetAddress(),P0R14.toByte());
  }

  /*
	  miniDSP_D IDAC (14:8) setting. Use when miniDSP_D is in use for signal processing (page 0,Reg 60) miniDSP_D IDAC(14:0)
  000 0000 0000 0000: miniDSP_D IDAC = 32768
  000 0000 0000 0001: miniDSP_D IDAC = 1
  000 0000 0000 0010: miniDSP_D IDAC = 2
  ...
  111 1111 1111 1110: miniDSP_D IDAC = 32766
  111 1111 1111 1111: miniDSP_D IDAC = 32767
  Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 )
  Note: Page-0, Reg-15 takes effect after programming Page-0, Reg-16 in the immediate next control command.
  */
  void FyberLabs_TAS2521::setminiDSP_DMSB(uint8_t IDACMSB) {
    if(_page !=0)
      switchPage(0);
    P0R15_t P0R15(read8(P0R15.GetAddress()));
    P0R15.setIDACValue(IDACMSB);
    write8(P0R15.GetAddress(),P0R15.toByte());
  }
  /*
	  miniDSP_D IDAC (7:0) setting. Use when miniDSP_D is in use for signal processing (page 0,Reg 60) miniDSP_D IDAC(14:0)
  000 0000 0000 0000: miniDSP_D 000 0000 0000 0001: miniDSP_D 000 0000 0000 0010: miniDSP_D ......
  111 1111 1111 1110: miniDSP_D
  111 1111 1111 1111: miniDSP_D
  Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 ) Note: Page-0, Reg-16 should be programmed immediately after Page-0, Reg-15.
  */
  void FyberLabs_TAS2521::setminiDSP_DLSB(uint8_t IDACLSB) {
    if(_page !=0)
      switchPage(0);
    P0R16_t P0R16(read8(P0R16.GetAddress()));
    P0R16.setIDACValue(IDACLSB);
    write8(P0R16.GetAddress(),P0R16.toByte());
  }

  /*
	  miniDSP_D interpolation factor setting. Used when miniDSP_D is in use for signal processing (page 0,Reg 60)
  0000 : Interpolation factor in miniDSP_D(INTERP) = 16
  0001: Interpolation factor in miniDSP_D(INTERP)= 1
  0010: Interpolation factor in miniDSP_D(INTERP) = 2 ...
  1110: Interpolation factor in miniDSP_D(INTERP) = 14
  */
  void FyberLabs_TAS2521::setminiDSP_DIntFactor(uint8_t IntFactor) {
    if(_page !=0)
      switchPage(0);
    P0R17_t P0R17(read8(P0R17.GetAddress()));
    P0R17.setInterpFactor(IntFactor);
    write8(P0R17.GetAddress(),P0R17.toByte());
  }

/*
	CDIV_CLKIN Clock Selection
000: CDIV_CLKIN = MCLK
001: CDIV_CLKIN = BCLK
010: CDIV_CLKIN = DIN
011: CDIV_CLKIN = PLL_CLK
100: CDIV_CLKIN = DAC_CLK
101: CDIV_CLKIN = DAC_MOD_CLK
*/
void FyberLabs_TAS2521::setCDIV_CLKIN(CDIV_CLKIN_t clkin) {
  if(_page !=0)
    switchPage(0);
  P0R25_t P0R25(read8(P0R25.GetAddress()));
  P0R25.setCDIVClk(clkin);
  write8(P0R25.GetAddress(),P0R25.toByte());
}

void FyberLabs_TAS2521::setCLKOUTMPowerDown(void) {
  if(_page !=0)
    switchPage(0);
  P0R26_t P0R26(read8(P0R26.GetAddress()));
  P0R26.setCLKoutMdivPower(POWER_DOWN);
  write8(P0R26.GetAddress(),P0R26.toByte());
}
void FyberLabs_TAS2521::setCLKOUTMPowerUp(void) {
  if(_page !=0)
    switchPage(0);
  P0R26_t P0R26(read8(P0R26.GetAddress()));
  P0R26.setCLKoutMdivPower(POWER_UP);
  write8(P0R26.GetAddress(),P0R26.toByte());
}

/*
	CLKOUT M divider value
000 0000: CLKOUT divider M = 128
000 0001: CLKOUT divider M = 1
000 0010: CLKOUT divider M = 2
...
111 1110: CLKOUT divider M = 126
111 1111: CLKOUT divider M = 127
Note: Check the clock frequency requirements in the application overview section.
*/
void FyberLabs_TAS2521::setCLKOUTMDivider(uint8_t divider) {
  if(_page !=0)
    switchPage(0);
  P0R26_t P0R26(read8(P0R26.GetAddress()));
  P0R26.setCLKoutMdivValue(divider);
  write8(P0R26.GetAddress(),P0R26.toByte());
}

/*
	Audio Interface Selection
	00: Audio Interface = I2S
	01: Audio Interface = DSP
	10: Audio Interface = RJF
	11: Audio Interface = LJF
*/
void FyberLabs_TAS2521::setAudioInterfaceSelect(INTERFACE_t interface) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setInterface(interface);
  write8(P0R27.GetAddress(),P0R27.toByte());
}
/*
	Audio Data Word length
00: Data Word length = 16 bits
01: Data Word length = 20 bits
10: Data Word length = 24 bits
11: Data Word length = 32 bits
*/
void FyberLabs_TAS2521::setAudioDataWordLength(WORD_LENGTH_t wordlength) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setWordLength(wordlength);
  write8(P0R27.GetAddress(),P0R27.toByte());
}
void FyberLabs_TAS2521::setAudioBCLKin(void) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setBclkDir(IN);
  write8(P0R27.GetAddress(),P0R27.toByte());
}

void FyberLabs_TAS2521::setAudioBCLKout(void) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setBclkDir(OUT);
  write8(P0R27.GetAddress(),P0R27.toByte());
}
void FyberLabs_TAS2521::setAudioWCLKin(void) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setWclkDir(IN);
  write8(P0R27.GetAddress(),P0R27.toByte());
}
void FyberLabs_TAS2521::FyberLabs_TAS2521::setAudioWCLKout(void) {
  if(_page !=0)
    switchPage(0);
  P0R27_t P0R27(read8(P0R27.GetAddress()));
  P0R27.setWclkDir(OUT);
  write8(P0R27.GetAddress(),P0R27.toByte());
}

/*
	Data Offset Value
0000 0000: Data Offset = 0 BCLK's
0000 0001: Data Offset = 1 BCLK's
...
1111 1110: Data Offset = 254 BCLK's
1111 1111: Data Offset = 255 BCLK's
*/
void FyberLabs_TAS2521::setAudioBCLKDataOffset(uint8_t offset) {
  if(_page !=0)
    switchPage(0);
  P0R28_t P0R28(read8(P0R28.GetAddress()));
  P0R28.setOffset(offset);
  write8(P0R28.GetAddress(),P0R28.toByte());
}

void FyberLabs_TAS2521::FyberLabs_TAS2521::setAudioBCLKPolarityDefault(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setAudioBitClockPolarity(DEFAULT);
  write8(P0R29.GetAddress(),P0R29.toByte());
}
void FyberLabs_TAS2521::setAudioBCLKPolarityInverted(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setAudioBitClockPolarity(INVERTED);
  write8(P0R29.GetAddress(),P0R29.toByte());
}

void FyberLabs_TAS2521::setAudioBWCLKPowerAlways(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setPrimaryBCLK_WCLKPower(false);
  write8(P0R29.GetAddress(),P0R29.toByte());
}
void FyberLabs_TAS2521::setAudioBWCLKPowerCodec(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setPrimaryBCLK_WCLKPower(true);
  write8(P0R29.GetAddress(),P0R29.toByte());
}

void FyberLabs_TAS2521::setAudioBDIV_CLKINDAC(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setBDIVCLKINMultiplexer(DAC_CLK);
  write8(P0R29.GetAddress(),P0R29.toByte());
}
void FyberLabs_TAS2521::setAudioBDIV_CLKINDAC_MOD(void) {
  if(_page !=0)
    switchPage(0);
  P0R29_t P0R29(read8(P0R29.GetAddress()));
  P0R29.setBDIVCLKINMultiplexer(DAC_MOD_CLK);
  write8(P0R29.GetAddress(),P0R29.toByte());
}

void FyberLabs_TAS2521::setAudioBCLKNDividerPowerUp(void) {
  if(_page !=0)
    switchPage(0);
  P0R30_t P0R30(read8(P0R30.GetAddress()));
  P0R30.seBclkNDIVPower(POWER_UP);
  write8(P0R30.GetAddress(),P0R30.toByte());
}
void FyberLabs_TAS2521::setAudioBCLKNDividerPowerDown(void) {
  if(_page !=0)
    switchPage(0);
  P0R30_t P0R30(read8(P0R30.GetAddress()));
  P0R30.seBclkNDIVPower(POWER_DOWN);
  write8(P0R30.GetAddress(),P0R30.toByte());
}
/*
	BCLK N Divider value
000 0000: BCLK divider N = 128
000 0001: BCLK divider N = 1
...
111 1110: BCLK divider N = 126
111 1111: BCLK divider N = 127
*/
void FyberLabs_TAS2521::setAudioBCLKNDivider(uint8_t divider) {
  if(_page !=0)
    switchPage(0);
  P0R30_t P0R30(read8(P0R30.GetAddress()));
  P0R30.seBclkNDIVValue(divider);
  write8(P0R30.GetAddress(),P0R30.toByte());
}

/*
	Secondary Bit Clock Multiplexer
	00: Secondary Bit Clock = GPIO
	01: Secondary Bit Clock = SCLK
	10: Secondary Bit Clock = MISO
	11: Secondary Bit Clock = DOUT
*/
void FyberLabs_TAS2521::setAudioSecondaryBCLK(SEC_CLK_MUX_t clk) {
  if(_page !=0)
    switchPage(0);
  P0R31_t P0R31(read8(P0R31.GetAddress()));
  P0R31.setBitClkMux(clk);
  write8(P0R31.GetAddress(),P0R31.toByte());
}
/*
	Secondary Word Clock Multiplexer
	00: Secondary Word Clock = GPIO
	01: Secondary Word Clock = SCLK
	10: Secondary Word Clock = MISO
	11: Secondary Word Clock = DOUT
*/
void FyberLabs_TAS2521::setAudioSecondaryWCLK(SEC_CLK_MUX_t clk) {
  if(_page !=0)
    switchPage(0);
  P0R31_t P0R31(read8(P0R31.GetAddress()));
  P0R31.setWrdClkMux(clk);
  write8(P0R31.GetAddress(),P0R31.toByte());
}
/*
	Secondary Data Input Multiplexer
	0: Secondary Data Input = GPIO
	1: Secondary Data Input = SCLK
*/
void FyberLabs_TAS2521::setAudioSecondaryMuxInput(DATA_IN_MUX_t input) {
  if(_page !=0)
    switchPage(0);
  P0R31_t P0R31(read8(P0R31.GetAddress()));
  P0R31.setDataInMux(input);
  write8(P0R31.GetAddress(),P0R31.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryBCLK(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setBitClkCtrl(0);
  write8(P0R32.GetAddress(),P0R32.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryBCLK(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setBitClkCtrl(1);
  write8(P0R32.GetAddress(),P0R32.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryWCLK(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setWrdClkCtrl(0);
  write8(P0R32.GetAddress(),P0R32.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryWCLK(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setWrdClkCtrl(1);
  write8(P0R32.GetAddress(),P0R32.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryDIN(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setDataInCtrl(0);
  write8(P0R32.GetAddress(),P0R32.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryDIN(void) {
  if(_page !=0)
    switchPage(0);
  P0R32_t P0R32(read8(P0R32.GetAddress()));
  P0R32.setDataInCtrl(1);
  write8(P0R32.GetAddress(),P0R32.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceBCLKOut_GeneratedPrimaryBitClock(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setBclkOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceBCLKOut_SecondaryBitClockInput(void) {
  if(_page !=0)
    switchPage(1);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setBclkOutCtrl(1);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceSecondaryBitClockOut_BCLKInput(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setSecBitClkOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceSecondaryBitClockOut_GeneratedPrimaryBitClock(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setSecBitClkOutCtrl(1);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceWCLKOut_GeneratedDAC_FS(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setWclkOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceWCLKOut_SecWrdClkIn(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setWclkOutCtrl(2);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceSecondaryWCLKOut_WCLKIn(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setWrdClkOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceSecondaryWCLKOut_GeneratedDAC_FS(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setWrdClkOutCtrl(1);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfacePrimaryDOUTOutput_SerialInterface(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setPriDataOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfacePrimaryDOUTOutput_LoopBack(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setPriDataOutCtrl(1);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::setAudioInterfaceSecondaryDOUTOutput_LoopBack(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setSecDataOutCtrl(0);
  write8(P0R33.GetAddress(),P0R33.toByte());
}
void FyberLabs_TAS2521::setAudioInterfaceSecondaryDOUTOutput_SerialInterface(void) {
  if(_page !=0)
    switchPage(0);
  P0R33_t P0R33(read8(P0R33.GetAddress()));
  P0R33.setSecDataOutCtrl(1);
  write8(P0R33.GetAddress(),P0R33.toByte());
}

void FyberLabs_TAS2521::I2CGeneralCallIgnored(void) {
  if(_page !=0)
    switchPage(0);
  P0R34_t P0R34(read8(P0R34.GetAddress()));
  P0R34.setI2CCallAddr(0);
  write8(P0R34.GetAddress(),P0R34.toByte());
}
void FyberLabs_TAS2521::I2CGeneralCallAccepted(void) {
  if(_page !=0)
    switchPage(0);
  P0R34_t P0R34(read8(P0R34.GetAddress()));
  P0R34.setI2CCallAddr(1);
  write8(P0R34.GetAddress(),P0R34.toByte());
}

bool FyberLabs_TAS2521::getDACPower(void) {
  if(_page !=0)
    switchPage(0);
  P0R37_t P0R37(read8(P0R37.GetAddress()));
  return P0R37.getDacPowerState();
}

bool FyberLabs_TAS2521::getHPOUTPower(void) {
  if(_page !=0)
    switchPage(0);
  P0R37_t P0R37(read8(P0R37.GetAddress()));
  return P0R37.getHpoutPowerState();
}

bool FyberLabs_TAS2521::getDACPGA(void) {
  if(_page !=0)
    switchPage(0);
  P0R38_t P0R38(read8(P0R38.GetAddress()));
  return P0R38.getPGAStat();
}

bool FyberLabs_TAS2521::getDACOverflowSticky(void) {
  if(_page !=0)
    switchPage(0);
  P0R42_t P0R42(read8(P0R42.GetAddress()));
  return P0R42.getDacOverflow();
}

bool FyberLabs_TAS2521::getminiDSP_DBarrelShifterOverflowSticky(void) {
  if(_page !=0)
    switchPage(0);
  P0R42_t P0R42(read8(P0R42.GetAddress()));
  return P0R42.getShifterOveflow();
}

bool FyberLabs_TAS2521::getDACOverflow(void) {
  if(_page !=0)
    switchPage(0);
  P0R43_t P0R43(read8(P0R43.GetAddress()));
  return P0R43.getDacOverflow();
}

bool FyberLabs_TAS2521::getminiDSP_DBarrelShifterOverflow(void) {
  if(_page !=0)
    switchPage(0);
  P0R43_t P0R43(read8(P0R43.GetAddress()));
  return P0R43.getShifterOveflow();
}

bool FyberLabs_TAS2521::getHPOUTOverCurrentSticky(void) {
  if(_page !=0)
    switchPage(0);
  P0R44_t P0R44(read8(P0R44.GetAddress()));
  return P0R44.getHpoutOverCurrent();
}

bool FyberLabs_TAS2521::getminiDSP_DStdInterruptSticky(void) {
  if(_page !=0)
    switchPage(0);
  P0R44_t P0R44(read8(P0R44.GetAddress()));
  return P0R44.getMinDspStdInt();
}

bool FyberLabs_TAS2521::getminiDSP_DAuxInterruptSticky(void) {
  if(_page !=0)
    switchPage(0);
  P0R44_t P0R44(read8(P0R44.GetAddress()));
  return P0R44.getMinDspAuxInt();
}

bool FyberLabs_TAS2521::getHPOUTOverCurrent(void) {
  if(_page !=0)
    switchPage(0);
  P0R46_t P0R46(read8(P0R46.GetAddress()));
  return P0R46.getHpoutOverCurrent();
}

bool FyberLabs_TAS2521::getminiDSP_DStdInterrupt(void) {
  if(_page !=0)
    switchPage(0);
  P0R46_t P0R46(read8(P0R46.GetAddress()));
  return P0R46.getMinDspStdInt();
}

bool FyberLabs_TAS2521::getminiDSP_DAuxInterrupt(void) {
  if(_page !=0)
    switchPage(0);
  P0R46_t P0R46(read8(P0R46.GetAddress()));
  return P0R46.getMinDspAuxInt();
}

bool FyberLabs_TAS2521::getINT1HPOUTOverCurrent(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  return P0R48.getOverCurrentIntEn();
}


void FyberLabs_TAS2521::setINT1HPOUTOverCurrentOn(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setOverCurrentIntEn(1);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

void FyberLabs_TAS2521::setINT1HPOUTOverCurrentOff(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setOverCurrentIntEn(0);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

bool FyberLabs_TAS2521::FyberLabs_TAS2521::getINT1miniDSP_DInterrupt(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  return P0R48.getOverflowIntEn();
}

void FyberLabs_TAS2521::setINT1miniDSP_DInterruptOn(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setOverflowIntEn(1);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

void FyberLabs_TAS2521::setINT1miniDSP_DInterruptOff(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setOverflowIntEn(0);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

bool FyberLabs_TAS2521::getINT1PulseControl(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  return P0R48.getPulseCtrl();
}

void FyberLabs_TAS2521::setINT1PulseControlSingle(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setPulseCtrl(0);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

void FyberLabs_TAS2521::setINT1PulseControlMultiple(void) {
  if(_page !=0)
    switchPage(0);
  P0R48_t P0R48(read8(P0R48.GetAddress()));
  P0R48.setPulseCtrl(1);
  write8(P0R48.GetAddress(),P0R48.toByte());
}

bool FyberLabs_TAS2521::getINT2HPOUTOverCurrent(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  return P0R49.getOverCurrentIntEn();
}


void FyberLabs_TAS2521::setINT2HPOUTOverCurrentOn(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setOverCurrentIntEn(1);
  write8(P0R49.GetAddress(),P0R49.toByte());
}

void FyberLabs_TAS2521::setINT2HPOUTOverCurrentOff(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setOverCurrentIntEn(0);
  write8(P0R49.GetAddress(),P0R49.toByte());
}

bool FyberLabs_TAS2521::getINT2miniDSP_DInterrupt(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  return P0R49.getOverflowIntEn();
}
void FyberLabs_TAS2521::setINT2miniDSP_DInterruptOn(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setOverflowIntEn(1);
  write8(P0R49.GetAddress(),P0R49.toByte());
}

void FyberLabs_TAS2521::setINT2miniDSP_DInterruptOff(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setOverflowIntEn(0);
  write8(P0R49.GetAddress(),P0R49.toByte());
}

bool FyberLabs_TAS2521::getINT2PulseControl(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  return P0R49.getPulseCtrl();
}
void FyberLabs_TAS2521::setINT2PulseControlSingle(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setPulseCtrl(0);
  write8(P0R49.GetAddress(),P0R49.toByte());
}
void FyberLabs_TAS2521::setINT2PulseControlMultiple(void) {
  if(_page !=0)
    switchPage(0);
  P0R49_t P0R49(read8(P0R49.GetAddress()));
  P0R49.setPulseCtrl(1);
  write8(P0R49.GetAddress(),P0R49.toByte());
}

/*
	GPIO Control
0000: GPIO input/output disabled.
0001: GPIO input is used for secondary audio interface or clock input. Configure other registers to choose the functionality of GPIO input.
0010: GPIO is general purpose input
0011: GPIO is general purpose output
0100: GPIO output is CLKOUT
0101: GPIO output is INT1
0110: GPIO output is INT2
0111: GPIO output is 0
1000: GPIO output is secondary bit-clock for Audio Interface.
1001: GPIO output is secondary word-clock for Audio Interface.
1010: GPIO output is 0
1011-1101: Reserved. Do not use.
1110: GPIO output is DOUT for Audio Interface according to Register 53 programming.
1111: Reserved. Do not use.
*/
void FyberLabs_TAS2521::setGPIOControl(uint8_t control) {
  if(_page !=0)
    switchPage(0);
  P0R52_t reg(read8(reg.GetAddress()));
  reg.setGpioCtrl(control);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setGPIOOutZero(void) {
  if(_page !=0)
    switchPage(0);
  P0R52_t reg(read8(reg.GetAddress()));
  reg.setGpioOutState(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setGPIOOutOne(void) {
  if(_page !=0)
    switchPage(0);
  P0R52_t reg(read8(reg.GetAddress()));
  reg.setGpioOutState(true);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::GetGPIOInState(void) {
  if(_page !=0)
    switchPage(0);
  P0R52_t reg(read8(reg.GetAddress()));
  return reg.getGpioInState();
}

void FyberLabs_TAS2521::setDOUTBusKeeperEnabled(void) {
  if(_page !=0)
    switchPage(0);
  P0R53_t reg(read8(reg.GetAddress()));
  reg.setBusKeepEn(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDOUTBusKeeperDisabled(void) {
  if(_page !=0)
    switchPage(0);
  P0R53_t reg(read8(reg.GetAddress()));
  reg.setBusKeepEn(false);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	DOUT MUX Control
000: DOUT disabled
001: DOUT is Primary DOUT
010: DOUT is General Purpose Output 011: DOUT is CLKOUT
100: DOUT is INT1
101: DOUT is INT2
110: DOUT is Secondary BCLK
111: DOUT is Secondary WCLK
*/
void FyberLabs_TAS2521::setDOUTMuxControl(DOUT_MUX_CTRL_t mux) {
  if(_page !=0)
    switchPage(0);
  P0R53_t reg(read8(reg.GetAddress()));
  reg.setDoutMuxCtrl(mux);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDOUTGPIOHigh(void) {
  if(_page !=0)
    switchPage(0);
  P0R53_t reg(read8(reg.GetAddress()));
  reg.setDoutGpio(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDOUTGPIOLow(void) {
  if(_page !=0)
    switchPage(0);
  P0R53_t reg(read8(reg.GetAddress()));
  reg.setDoutGpio(false);
  write8(reg.GetAddress(),reg.toByte());
}

//DIN function control
//00: DIN pin is disabled
//01: DIN is enabled for Primary Data Input or General Purpose Clock input 10: DIN is used as General Purpose Input
//11: Reserved. Do not use
void FyberLabs_TAS2521::setDINFunctionControl(DIN_FUNC_CTRL_t control) {
  if(_page !=0)
    switchPage(0);
  P0R54_t reg(read8(reg.GetAddress()));
  reg.setDinFunc(control);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::getDINGPIOInput(void) {
  if(_page !=0)
    switchPage(0);
  P0R54_t reg(read8(reg.GetAddress()));
  return reg.getDinGPIOState();
}

//MISO function control
//0000: MISO buffer disabled
//0001: MISO is used for data output in SPI interface, is disabled for I2C interface 0010: MISO is General Purpose Output
//0011: MISO is CLKOUT output
//0100: MISO is INT1 output
//0101: MISO is INT2 output
//0110: Reserved
//0111: Reserved
//1000: MISO is Secondary Data Output for Audio Interface
//1001: MISO is Secondary Bit Clock for Audio Interface
//1010: MISO is Secondary Word Clock for Audio Interface
//1011-1111: Reserved. Do not use
void FyberLabs_TAS2521::setMISOFunctionControl(MISO_FUNC_CTRL_t control) {
  if(_page !=0)
    switchPage(0);
  P0R55_t reg(read8(reg.GetAddress()));
  reg.setMisoFunc(control);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setMISOGPIOOutput(bool bit) {
  if(_page !=0)
    switchPage(0);
  P0R55_t reg(read8(reg.GetAddress()));
  reg.setMISOGPIOState(bit);
  write8(reg.GetAddress(),reg.toByte());
}

/*
SCLK function control
00: SCLK pin is disabled
01: SCLK pin is enabled for SPI clock in SPI Interface mode or when in I2C Interface enabled for Secondary Data Input or Secondary Bit Clock Input or Secondary Word Clock. 10: SCLK is enabled as General Purpose Input
11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setSCLKFunctionControl(SCLK_FUNC_CTRL_t control) {
  if(_page !=0)
    switchPage(0);
  P0R56_t reg(read8(reg.GetAddress()));
  reg.setSclkFunc(control);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::getSCLKGPIOInput(void) {
  if(_page !=0)
    switchPage(0);
  P0R56_t reg(read8(reg.GetAddress()));
  return reg.getSclkGPIOState();
}

/*
	0 0000: The miniDSP_D will be used for signal processing 0 0001: DAC Signal Processing Block PRB_P1
0 0010: DAC Signal Processing Block PRB_P2
0 0011: DAC Signal Processing Block PRB_P3
0 0100-1 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setDACInstructionSet(DAC_INST_t ins) {
  if(_page !=0)
    switchPage(0);
  P0R60_t reg(read8(reg.GetAddress()));
  reg.setDacInstruction(ins);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setminiDSP_DConfigurationBitA(bool bit) {
  if(_page !=0)
    switchPage(0);
  P0R62_t reg(read8(reg.GetAddress()));
  reg.setAuxBitA(bit);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setminiDSP_DConfigurationBitB(bool bit) {
  if(_page !=0)
    switchPage(0);
  P0R62_t reg(read8(reg.GetAddress()));
  reg.setAuxBitB(bit);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setminiDSP_DConfigurationResetCounter(void) {
  if(_page !=0)
    switchPage(0);
  P0R62_t reg(read8(reg.GetAddress()));
  reg.setResetInstCnt(false);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setminiDSP_DConfigurationNoResetCounter(void) {
  if(_page !=0)
    switchPage(0);
  P0R62_t reg(read8(reg.GetAddress()));
  reg.setResetInstCnt(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACChannelSetupPowerUp(void) {
  if(_page !=0)
    switchPage(0);
  P0R63_t reg(read8(reg.GetAddress()));
  reg.setDacChanPwr(POWER_UP);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDACChannelSetupPowerDown(void) {
  if(_page !=0)
    switchPage(0);
  P0R63_t reg(read8(reg.GetAddress()));
  reg.setDacChanPwr(POWER_DOWN);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	DAC Data path Control
00: DAC data is disabled
01: DAC data is picked from Left Channel Audio Interface Data
10: DAC data is picked from Right Channel Audio Interface Data
11: DAC data is picked from Mono Mix of Left and Right Channel Audio Interface Data
*/
void FyberLabs_TAS2521::setDACChannelSetupDataPath(DAC_DATA_PATH_t path) {
  if(_page !=0)
    switchPage(0);
  P0R63_t reg(read8(reg.GetAddress()));
  reg.setDacDataPath(path);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	DAC Channel Volume Control's Soft-Step control
	00: Soft-Stepping is 1 step per 1 DAC Word Clock
	01: Soft-Stepping is 1 step per 2 DAC Word Clocks
	10: Soft-Stepping is disabled
11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setDACChannelVolumeControl(uint8_t volume) {
  if(_page !=0)
    switchPage(0);
  P0R63_t reg(read8(reg.GetAddress()));
  reg.setDacChanVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	DAC Auto Mute Control
	000: Auto Mute disabled
	001: DAC is auto muted if input data is DC for more than 100 consecutive inputs
	010: DAC is auto muted if input data is DC for more than 200 consecutive inputs
	011: DAC is auto muted if input data is DC for more than 400 consecutive inputs
	100: DAC is auto muted if input data is DC for more than 800 consecutive inputs
	101: DAC is auto muted if input data is DC for more than 1600 consecutive inputs
	110: DAC is auto muted if input data is DC for more than 3200 consecutive inputs
	111: DAC is auto muted if input data is DC for more than 6400 consecutive inputs
*/
void FyberLabs_TAS2521::setDACChannelAutoMuteControl(DAC_AUTO_MUTE_t mute) {
  if(_page !=0)
    switchPage(0);
  P0R64_t reg(read8(reg.GetAddress()));
  reg.setDacAutoMuteCtrl(mute);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDACChannelMute(void) {
  if(_page !=0)
    switchPage(0);
  P0R64_t reg(read8(reg.GetAddress()));
  reg.setDacChanMuteCtrl(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDACChannelUnmute(void) {
  if(_page !=0)
    switchPage(0);
  P0R64_t reg(read8(reg.GetAddress()));
  reg.setDacChanMuteCtrl(false);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	DAC Channel Digital Volume Control Setting
	0111 1111-0011 0001: Reserved. Do not use
	0011 0000: Digital Volume Control = +24dB
	0010 1111: Digital Volume Control = +23.5dB
	...
	0000 0001: Digital Volume Control = +0.5dB
	0000 0000: Digital Volume Control = 0.0dB
	1111 1111: Digital Volume Control = -0.5dB
	...
	1000 0010: Digital Volume Control = -63dB
	1000 0001: Digital Volume Control = -63.5dB
	1000 0000: Reserved. Do not use"
*/
void FyberLabs_TAS2521::setDACChannelDigitalVolumeControl(uint8_t volume) {
  if(_page !=0)
    switchPage(0);
  P0R65_t reg(read8(reg.GetAddress()));
  reg.setDacChanVol(volume);
  write8(reg.GetAddress(),reg.toByte());
}

//Page1
void FyberLabs_TAS2521::setPage(uint8_t page) {
  switchPage(page);
}

void FyberLabs_TAS2521::setMasterReferencePowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setMastRefCtrl(POWER_UP);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setMasterReferencePowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setMastRefCtrl(POWER_DOWN);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setPORPowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setPorPwrCtrl(false);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setPORPowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setPorPwrCtrl(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setLDOBandGapPowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setLdoPwrCtrl(false);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setLDOBandGapPowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R1_t reg(read8(reg.GetAddress()));
  reg.setLdoPwrCtrl(true);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	AVDD LDO Control
	00: AVDD LDO output is nominally 1.8V
	01: AVDD LDO output is nominally 1.6V
	10: AVDD LDO output is nominally 1.7V
	11: AVDD LDO output is nominally 1.5V
*/
void FyberLabs_TAS2521::setLDOControl(AVDD_LDO_CTRL_t voltage) {
  if(_page !=1)
    switchPage(1);
  P1R2_t reg(read8(reg.GetAddress()));
  reg.setAvddLdoVoltage(voltage);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setLDOPLLHPPowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R2_t reg(read8(reg.GetAddress()));
  reg.setLvlShiftPwr(false);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setLDOPLLHPPowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R2_t reg(read8(reg.GetAddress()));
  reg.setLvlShiftPwr(true);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::getLDOShortCircuitDetection(void) {
  if(_page !=1)
    switchPage(1);
  P1R2_t reg(read8(reg.GetAddress()));
  return reg.getAvddLdoShort();
}

bool FyberLabs_TAS2521::getLDOSelect(void) {
  if(_page !=1)
    switchPage(1);
  P1R2_t reg(read8(reg.GetAddress()));
  return reg.getLdoSelState();
}

void FyberLabs_TAS2521::setPlaybackConfigurationDACLowPower(void) {
  if(_page !=1)
    switchPage(1);
  P1R3_t reg(read8(reg.GetAddress()));
  reg.setDacMode(LOW_POWER);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setPlaybackConfigurationDACHighPerformance(void) {
  if(_page !=1)
    switchPage(1);
  P1R3_t reg(read8(reg.GetAddress()));
  reg.setDacMode(HIGH_PERFORMANCE);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	DAC PTM Control
	000: DAC in mode PTM_P3, PTM_P4
	001: DAC in mode PTM_P2
	010: DAC in mode PTM_P1
	011-111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setPlaybackConfigurationDACPTMControl(uint8_t control) {
  if(_page !=1)
    switchPage(1);
  P1R3_t reg(read8(reg.GetAddress()));
  reg.setDacPtmCtrl(control);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACPGAControlSoftSteppingOn(void) {
  if(_page !=1)
    switchPage(1);
  P1R8_t reg(read8(reg.GetAddress()));
  reg.setSoftStepEn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACPGAControlSoftSteppingOff(void) {
  if(_page !=1)
    switchPage(1);
  P1R8_t reg(read8(reg.GetAddress()));
  reg.setSoftStepEn(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACPGAControlSoftSteppingNormal(void) {
  if(_page !=1)
    switchPage(1);
  P1R8_t reg(read8(reg.GetAddress()));
  reg.setSoftStepMode(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACPGAControlSoftSteppingDouble(void) {
  if(_page !=1)
    switchPage(1);
  P1R8_t reg(read8(reg.GetAddress()));
  reg.setSoftStepMode(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setOutputHPLPowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setHplPwr(POWER_UP);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setOutputHPLPowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setHplPwr(POWER_DOWN);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setAINLInputOn(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setAinlEn(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setAINLInputOff(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setAinlEn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setAINRInputOn(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setAinrEn(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setAINRInputOff(void) {
  if(_page !=1)
    switchPage(1);
  P1R9_t reg(read8(reg.GetAddress()));
  reg.setAinrEn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setFullChipCommonMode09V(void) {
  if(_page !=1)
    switchPage(1);
  P1R10_t reg(read8(reg.GetAddress()));
  reg.setCommonMode(COMMON_MODE_0_9V);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setFullChipCommonMode075V(void) {
  if(_page !=1)
    switchPage(1);
  P1R10_t reg(read8(reg.GetAddress()));
  reg.setCommonMode(COMMON_MODE_0_75V);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTFullDrive(void) {
  if(_page !=1)
    switchPage(1);
  P1R10_t reg(read8(reg.GetAddress()));
  reg.setHpDrive(FULL_DRIVE);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setHPOUTHalfDrive(void) {
  if(_page !=1)
    switchPage(1);
  P1R10_t reg(read8(reg.GetAddress()));
  reg.setHpDrive(HALF_DRIVE);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	000: No debounce is used for Over Current detection
	001: Over Current detection is debounced by 8ms
	010: Over Current detection is debounce by 16ms
	011: Over Current detection is debounced by 32ms
	100: Over Current detection is debounced by 64ms
	101: Over Current detection is debounced by 128ms
	110: Over Current detection is debounced by 256ms
	111: Over Current detection is debounced by 512ms
*/
void FyberLabs_TAS2521::setHPOUTOverCurrentDebounce(OVER_CURRENT_DETECTION_DEBOUNCE_t debounce) {
  if(_page !=1)
    switchPage(1);
  P1R11_t reg(read8(reg.GetAddress()));
  reg.setOvrCurrDebounce(debounce);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTOverCurrentLimiting(void) {
  if(_page !=1)
    switchPage(1);
  P1R11_t reg(read8(reg.GetAddress()));
  reg.setOvrCurrCondition(LIMIT_OUT_CURRENT);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTOverCurrentPowerOff(void) {
  if(_page !=1)
    switchPage(1);
  P1R11_t reg(read8(reg.GetAddress()));
  reg.setOvrCurrCondition(POWER_DOWN_DRIVER);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	0000: No analog routing to SPK driver and HP driver
	0001 - 0011 : Do not use
	0100: AINR routed to Mixer P
	0101: Do not use
	0110: AINL/R differential routed to SPK driver through Mixer A and Mixer B 0111: Do no use
	1000: AINL routed to Mixer A
	1001: AINL/R differential routed to SPK driver through Mixer A and Mixer B 1010 -1011: Do not use
	1100: AINL and AINR routed to Mixer A to HP driver
	1101 - 1111: Do not use
*/
void FyberLabs_TAS2521::setHPOUTRouting(ANALOG_ROUTING_t routing) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setAnalogRouting(routing);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTDACRoutedDirect(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setDacRout(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::FyberLabs_TAS2521::setHPOUTDACRoutedIndirect(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setDacRout(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTMixerPAttenuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setMixerPtoHpAttn(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTMixerPNotAttentuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setMixerPtoHpAttn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTAINLAttenuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setAinlAttn(true);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTAINLNotAttenuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setAinlAttn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTAINRAttenuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setAinrAttn(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setHPOUTAINRNotAttenuator(void) {
  if(_page !=1)
    switchPage(1);
  P1R12_t reg(read8(reg.GetAddress()));
  reg.setAinrAttn(false);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setHPOUTDriverMuted(void) {
  if(_page !=1)
    switchPage(1);
  P1R16_t reg(read8(reg.GetAddress()));
  reg.setHpDrvMute(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setHPOUTDriverUnmuted(void) {
  if(_page !=1)
    switchPage(1);
  P1R16_t reg(read8(reg.GetAddress()));
  reg.setHpDrvMute(false);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	10 0000 - 11 1001: Reserved. Do not use
	11 1010: HP driver gain is -6dB (Note: It is not possible to mute HPR while programmed to - 6dB)
	11 1011: HP driver gain is -5dB 11 1100: HP driver gain is -4dB
	11 1101: HP driver gain is -3dB
	...
	00 0000: HP driver gain is 0dB
	...
	00 0011: HP driver gain is 3dB
	00 0100: HP driver gain is 4dB
	00 0101: Hp driver gain is 5dB
	00 0110: HP driver gain is 6dB
	...
	00 1100: HP driver gain is 12dB
	...
	01 0010: HP driver gain is 18dB
	...
	01 1000: HP driver gain is 24dB
	...
	01 1100: HP driver gain is 28dB
	01 1101: HP driver gain is 29dB
	...
	00 1110 - 01 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setHPOUTDriverGain(uint8_t gain) {
  if(_page !=1)
    switchPage(1);
  P1R16_t reg(read8(reg.GetAddress()));
  reg.setHpDrvGain(gain);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	00: Soft-routing step time = 0ms
	01: Soft-routing step time = 50ms
	10: Soft-routing step time = 100ms
	11: Soft-routing step time = 200ms
*/
void FyberLabs_TAS2521::setHPOOUTDriverStartupSoftRoutingStepTime(SOFT_ROUTING_STEP_TIME_t time) {
  if(_page !=1)
    switchPage(1);
  P1R20_t reg(read8(reg.GetAddress()));
  reg.setsoftRoutingStepTime(time);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	0000: Slow power up of headphone amp's is disabled
	0001: Headphone amps power up slowly in 0.5 time constants
	0010: Headphone amps power up slowly in 0.625 time constants
	0011{

} Headphone amps power up slowly in 0.725 time constants
	0100: Headphone amps power up slowly in 0.875 time constants
	0101: Headphone amps power up slowly in 1.0 time constants
	0110: Headphone amps power up slowly in 2.0 time constants
	0111: Headphone amps power up slowly in 3.0 time constants
	1000: Headphone amps power up slowly in 4.0 time constants
	1001: Headphone amps power up slowly in 5.0 time constants
	1010: Headphone amps power up slowly in 6.0 time constants
	1011: Headphone amps power up slowly in 7.0 time constants
	1100: Headphone amps power up slowly in 8.0 time constants
	1101: Headphone amps power up slowly in 16.0 time constants ( do not use for Rchg=25K)
	1110: Headphone amps power up slowly in 24.0 time constants (do not use for Rchg=25K)
	1111: Headphone amps power up slowly in 32.0 time constants (do not use for Rchg=25K)
	Note: Time constants assume 47μF decoupling cap
*/
void FyberLabs_TAS2521::setHPOUTDriverStartupPowerUpTime(HP_AMP_SLOW_POWERUP_TIME_t time) {
  if(_page !=1)
    switchPage(1);
  P1R20_t reg(read8(reg.GetAddress()));
  reg.setSlowHpAmpPwrUp(time);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	00: Headphone amps power up time is determined with 25K resistance
	01: Headphone amps power up time is determined with 6K resistance
	10: Headphone amps power up time is determined with 2K resistance
	11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setHPOUTDriverResistance(HP_AMP_POWERUP_RESISTOR_t resistance) {
  if(_page !=1)
    switchPage(1);
  P1R20_t reg(read8(reg.GetAddress()));
  reg.setHpAmpPwrUpResistor(resistance);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	000 0000: Volume Control = 0.0dB
	000 0001: Volume Control = -0.5dB
	000 0010: Volume Control = -1.0dB
	000 0011: Volume Control = -1.5dB
	000 0100: Volume Control = -2.0dB
	000 0101: Volume Control = -2.5dB
	000 0110: Volume Control = -3.0dB
	000 0111: Volume Control = -3.5dB
	000 1000: Volume Control = -4.0dB
	000 1001: Volume Control = -4.5dB
	...
	110 1101: Volume Control = -56.7dB
	110 1110: Volume Control = -58.3dB
	110 1111: Volume Control = -60.1dB
	111 0000: Volume Control = -62.7dB
	111 0001: Volume Control = -64.3dB
	111 0010: Volume Control = -66.2dB
	111 0011: Volume Control = -66.7dB
	111 0100: Volume Control = -72.3dB
	111 0101: Volume Control = Mute
	111 0110-111 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setHPOUTVolume(uint8_t volume) {
  if(_page !=1)
    switchPage(1);
  P1R22_t reg(read8(reg.GetAddress()));
  reg.setVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}

//See note
void FyberLabs_TAS2521::setAINLRMixerPandMixerMForceOn(void) {
  if(_page !=1)
    switchPage(1);
  P1R24_t reg(read8(reg.GetAddress()));
  reg.setMixerPMForceEn(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setAINLRMixerPandMixerMUnforcedOn(void) {
  if(_page !=1)
    switchPage(1);
  P1R24_t reg(read8(reg.GetAddress()));
  reg.setMixerPMForceEn(false);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	000 0000: Volume Control = 0.0dB
	000 0001: Volume Control = -0.5dB
	000 0010: Volume Control = -1.0dB
	000 0011: Volume Control = -1.5dB
	000 0100: Volume Control = -2.0dB
	000 0101: Volume Control = -2.5dB
	000 0110: Volume Control = -3.0dB
	000 0111: Volume Control = -3.5dB
	000 1000: Volume Control = -4.0dB
	000 1001: Volume Control = -4.5dB
	...
	110 1101: Volume Control = -56.7dB
	110 1110: Volume Control = -58.3dB
	110 1111: Volume Control = -60.1dB
	111 0000: Volume Control = -62.7dB
	111 0001: Volume Control = -64.3dB
	111 0010: Volume Control = -66.2dB
	111 0011: Volume Control = -66.7dB
	111 0100: Volume Control = -72.3dB
	111 0101: Volume Control = Mute
	111 0110-111 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setAINLVolume(uint8_t volume) {
  if(_page !=1)
    switchPage(1);
  P1R24_t reg(read8(reg.GetAddress()));
  reg.setVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}
/*
	000 0000: Volume Control = 0.0dB
	000 0001: Volume Control = -0.5dB
	000 0010: Volume Control = -1.0dB
	000 0011: Volume Control = -1.5dB
	000 0100: Volume Control = -2.0dB
	000 0101: Volume Control = -2.5dB
	000 0110: Volume Control = -3.0dB
	000 0111: Volume Control = -3.5dB
	000 1000: Volume Control = -4.0dB
	000 1001: Volume Control = -4.5dB
	...
	110 1101: Volume Control = -56.7dB
	110 1110: Volume Control = -58.3dB
	110 1111: Volume Control = -60.1dB
	111 0000: Volume Control = -62.7dB
	111 0001: Volume Control = -64.3dB
	111 0010: Volume Control = -66.2dB
	111 0011: Volume Control = -66.7dB
	111 0100: Volume Control = -72.3dB
	111 0101: Volume Control = Mute
	111 0110-111 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setAINRVolume(uint8_t volume) {
  if(_page !=1)
    switchPage(1);
  P1R25_t reg(read8(reg.GetAddress()));
  reg.setVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setSpeakerDriverPowerUp(void) {
  if(_page !=1)
    switchPage(1);
  P1R45_t reg(read8(reg.GetAddress()));
  reg.setSpkDrvPwr(POWER_UP);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setSpeakerDriverPowerDown(void) {
  if(_page !=1)
    switchPage(1);
  P1R45_t reg(read8(reg.GetAddress()));
  reg.setSpkDrvPwr(POWER_DOWN);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	000 0000: Volume Control = 0.0dB
	000 0001: Volume Control = -0.5dB
	000 0010: Volume Control = -1.0dB
	000 0011: Volume Control = -1.5dB
	000 0100: Volume Control = -2.0dB
	000 0101: Volume Control = -2.5dB
	000 0110: Volume Control = -3.0dB
	000 0111: Volume Control = -3.5dB
	000 1000: Volume Control = -4.0dB
	000 1001: Volume Control = -4.5dB
	...
	110 1101: Volume Control = -56.7dB
	110 1110: Volume Control = -58.3dB
	110 1111: Volume Control = -60.1dB
	111 0000: Volume Control = -62.7dB
	111 0001: Volume Control = -64.3dB
	111 0010: Volume Control = -66.2dB
	111 0011: Volume Control = -66.7dB
	111 0100: Volume Control = -72.3dB
	111 0101-111 1110: Reserved. Do not use
	*111 1111: Volume Control = Mute
*/
void FyberLabs_TAS2521::setSpeakerVolume(uint8_t volume) {
  if(_page !=1)
    switchPage(1);
  P1R46_t reg(read8(reg.GetAddress()));
  reg.setVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}

/*
	Left Speaker Amplifier (SPK) Volume Control:
	000: SPK Driver is Muted (Default)
	001: SPK Driver Volume = 6 dB
	010: SPK Driver Volume = 12 dB
	011: SPK Driver Volume = 18 dB
	100: SPK Driver Volume = 24 dB
	101: SPK Driver Volume = 32 dB
	110 - 111: Reserved
*/
void FyberLabs_TAS2521::setSpeakerAmplifierVolume(SPK_AMP_VOL_t volume) {
  if(_page !=1)
    switchPage(1);
  P1R48_t reg(read8(reg.GetAddress()));
  reg.setVolume(volume);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::getHPOUTAppliedGain(void) {
  if(_page !=1)
    switchPage(1);
  P1R63_t reg(read8(reg.GetAddress()));
  return reg.getHPGainFlag();
}

bool FyberLabs_TAS2521::getAINLMixPGAHPOUTAppliedGain(void) {
  if(_page !=1)
    switchPage(1);
  P1R63_t reg(read8(reg.GetAddress()));
  return reg.getAIN1LMixAppliedGainFlag();
}

bool FyberLabs_TAS2521::getAINLMixPGAAppliedVolume(void) {
  if(_page !=1)
    switchPage(1);
  P1R63_t reg(read8(reg.GetAddress()));
  return reg.getLeftMixerVolFlag();
}

bool FyberLabs_TAS2521::getAINRMixPGAAppliedVolume(void) {
  if(_page !=1)
    switchPage(1);
  P1R63_t reg(read8(reg.GetAddress()));
  return reg.getRightMixerVolFlag();
}

/*
	Reference Power Up configuration
	000: Reference will power up slowly when analog blocks are powered up
	001: Reference will power up in 40ms when analog blocks are powered up
	010: Reference will power up in 80ms when analog blocks are powered up
	011: Reference will power up in 120ms when analog blocks are powered up
	100: Force power up of reference. Power up will be slow
	101: Force power up of reference. Power up time will be 40ms
	110: Force power up of reference. Power up time will be 80ms
	111: Force power up of reference. Power up time will be 120ms
*/
void FyberLabs_TAS2521::setReferencePowerUpDelay(REF_PWRUP_DELAY_t delay) {
  if(_page !=1)
    switchPage(1);
  P1R122_t reg(read8(reg.GetAddress()));
  reg.setPwrUpDelay(delay);
  write8(reg.GetAddress(),reg.toByte());
}

void FyberLabs_TAS2521::setDACAdaptiveFilterOn(void) {
  if(_page !=44)
    switchPage(44);
  P44R1_t reg(read8(reg.GetAddress()));
  reg.setDacAdapFiltEn(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDACAdaptiveFilterOff(void) {
  if(_page !=44)
    switchPage(44);
  P44R1_t reg(read8(reg.GetAddress()));
  reg.setDacAdapFiltEn(false);
  write8(reg.GetAddress(),reg.toByte());
}

bool FyberLabs_TAS2521::getDACAdaptiveFilterControlFlag(void) {
  if(_page !=44)
    switchPage(44);
  P44R1_t reg(read8(reg.GetAddress()));
  return reg.getDacAdapFiltBufFlag();
}
void FyberLabs_TAS2521::setDACAdaptiveFilterSwitch(void) {
  if(_page !=44)
    switchPage(44);
  P44R1_t reg(read8(reg.GetAddress()));
  reg.setDacAdapFiltBufSwitch(true);
  write8(reg.GetAddress(),reg.toByte());
}
void FyberLabs_TAS2521::setDACAdaptiveFilterNotSwitch(void) {
  if(_page !=44)
    switchPage(44);
  P44R1_t reg(read8(reg.GetAddress()));
  reg.setDacAdapFiltBufSwitch(false);
  write8(reg.GetAddress(),reg.toByte());
}

//Page 44 / Register 8 - 127: DAC Coefficient Buffer-A C(0:29) - 0x2C / 0x08 - 0x7F
//Page 45 - 52 / Register 8 - 127: DAC Coefficients Buffer-A C(30:255) - 0x2D - 0x34 / 0x08 -0x7F
void FyberLabs_TAS2521::setDACCoefficientBufferA(uint8_t coeff, uint32_t value) {
  uint8_t b1,b2,b3,page,reg;

  b3=(value&0x00FF0000)>>16;
  b2=(value&0x0000FF00)>>8;
  b1=(value&0x000000FF);

  page= 44+(coeff/30);
  reg=8+(coeff%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.write(b3);
  Wire.write(b2);
  Wire.write(b1);
  Wire.endTransmission();
}

uint32_t FyberLabs_TAS2521::getDACCoefficientBufferA(uint8_t coeff) {
  uint8_t b1,b2,b3,page,reg;

  page= 44+(coeff/30);
  reg=8+(coeff%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)3);
  b3=Wire.read();
  b2=Wire.read();
  b1=Wire.read();

  return (b3<<2)|(b2<<1)|(b1);
}


//Page 62 - 70 / Register 8 -127: DAC Coefficients Buffer-B C(0:255) - 0x3E - 0x46 / 0x08 - 0x7F
void FyberLabs_TAS2521::setDACCoefficientBufferB(uint8_t coeff, uint32_t value) {
  uint8_t b1,b2,b3,page,reg;

  b3=(value&0x00FF0000)>>16;
  b2=(value&0x0000FF00)>>8;
  b1=(value&0x000000FF);

  page= 62+(coeff/30);
  reg=8+(coeff%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.write(b3);
  Wire.write(b2);
  Wire.write(b1);
  Wire.endTransmission();
}

uint32_t FyberLabs_TAS2521::getDACCoefficientBufferB(uint8_t coeff) {
  uint8_t b1,b2,b3,page,reg;

  page= 44+(coeff/30);
  reg=8+(coeff%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)3);
  b3=Wire.read();
  b2=Wire.read();
  b1=Wire.read();

  return (b3<<2)|(b2<<1)|(b1);
}

//Page 152 - 169 / Register 8 - 127: DAC Instruction Registers - 0x98 - 0xA9 / 0x08 - 0x7F
void FyberLabs_TAS2521::setDACminiDSPInstruction(uint8_t Inst, uint32_t value) {
  uint8_t b1,b2,b3,page,reg;

  b3=(value&0x00FF0000)>>16;
  b2=(value&0x0000FF00)>>8;
  b1=(value&0x000000FF);

  page= 62+(Inst/30);
  reg=8+(Inst%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.write(b3);
  Wire.write(b2);
  Wire.write(b1);
  Wire.endTransmission();
}

uint32_t FyberLabs_TAS2521::getDACminiDSPInstruction(uint8_t Inst) {
  uint8_t b1,b2,b3,page,reg;

  page= 152+(Inst/30);
  reg=8+(Inst%30)*4;
  if(_page !=page)
      switchPage(page);

  Wire.beginTransmission(_i2c_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)_i2c_address, (uint8_t)3);
  b3=Wire.read();
  b2=Wire.read();
  b1=Wire.read();

  return (b3<<2)|(b2<<1)|(b1);
}


}
