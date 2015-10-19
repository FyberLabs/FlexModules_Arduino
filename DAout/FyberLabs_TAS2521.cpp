//TODO make class methods based on usage examples in http://www.ti.com/lit/ug/slau456/slau456.pdf

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
    return Wire.read();;
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

    P0R4_t P0R4(read8(4));
    if(P0R4.getCodecClk()!=clk)
    {
      P0R4.setCodecClk(clk);
      write8(P0R4.GetAddress(),P0R4.toByte());
    }
  }

  void FyberLabs_TAS2521::setPLLPowerDown(void) {

  }
  void FyberLabs_TAS2521::setPLLPowerUp(void) {

  }

  /*
	  000: PLL divider P = 8
	  001: PLL divider P = 1
	  010: PLL divider P = 2
	  ...
  110: PLL divider P = 6
  111: PLL divider P = 7
  */
  void FyberLabs_TAS2521::setPLLDividerP(uint8_t P) {

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
  void FyberLabs_TAS2521::setPLLDividerR(uint8_t R) {

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

  }

  void FyberLabs_TAS2521::setNDACPowerDown(void) {

  }
  void FyberLabs_TAS2521::setNDACPowerUp(void) {

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

  }

  void FyberLabs_TAS2521::setMDACPowerDown(void) {

  }
  void FyberLabs_TAS2521::setMDACPowerUp(void) {

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

  }
  /*
	  miniDSP_D IDAC (7:0) setting. Use when miniDSP_D is in use for signal processing (page 0,Reg 60) miniDSP_D IDAC(14:0)
  000 0000 0000 0000: miniDSP_D 000 0000 0000 0001: miniDSP_D 000 0000 0000 0010: miniDSP_D ......
  111 1111 1111 1110: miniDSP_D
  111 1111 1111 1111: miniDSP_D
  Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 ) Note: Page-0, Reg-16 should be programmed immediately after Page-0, Reg-15.
  */
  void FyberLabs_TAS2521::setminiDSP_DLSB(uint8_t IDACLSB) {

  }

  /*
	  miniDSP_D interpolation factor setting. Used when miniDSP_D is in use for signal processing (page 0,Reg 60)
  0000 : Interpolation factor in miniDSP_D(INTERP) = 16
  0001: Interpolation factor in miniDSP_D(INTERP)= 1
  0010: Interpolation factor in miniDSP_D(INTERP) = 2 ...
  1110: Interpolation factor in miniDSP_D(INTERP) = 14
  */
  void FyberLabs_TAS2521::setminiDSP_DIntFactor(uint8_t IntFactor) {

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
void FyberLabs_TAS2521::setCDIV_CLKIN(uint8_t clkin) {

}

void FyberLabs_TAS2521::setCLKOUTMPowerDown(void) {

}
void FyberLabs_TAS2521::setCLKOUTMPowerUp(void) {

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

}

/*
	Audio Interface Selection
	00: Audio Interface = I2S
	01: Audio Interface = DSP
	10: Audio Interface = RJF
	11: Audio Interface = LJF
*/
void FyberLabs_TAS2521::setAudioInterfaceSelect(uint8_t interface) {

}
/*
	Audio Data Word length
00: Data Word length = 16 bits
01: Data Word length = 20 bits
10: Data Word length = 24 bits
11: Data Word length = 32 bits
*/
void FyberLabs_TAS2521::setAudioDateWordLength(uint8_t wordlength) {

}
void FyberLabs_TAS2521::setAudioBCLKin(void) {

}
void FyberLabs_TAS2521::setAudioBCLKout(void) {

}
void FyberLabs_TAS2521::setAudioWCLKin(void) {

}
void FyberLabs_TAS2521::FyberLabs_TAS2521::setAudioWCLKout(void) {

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

}

void FyberLabs_TAS2521::FyberLabs_TAS2521::setAudioBCLKPolarityDefault(void) {

}
void FyberLabs_TAS2521::setAudioBCLKPolarityInverted(void) {

}

void FyberLabs_TAS2521::setAudioBWCLKPowerAlways(void) {

}
void FyberLabs_TAS2521::setAudioBWCLKPowerCodec(void) {

}

void FyberLabs_TAS2521::setAudioBDIV_CLKINDAC(void) {

}
void FyberLabs_TAS2521::setAudioBDIV_CLKINDAC_MOD(void) {

}

void FyberLabs_TAS2521::setAudioBCLKNDividerPowerUp(void) {

}
void FyberLabs_TAS2521::setAudioBCLKNDividerPowerDown(void) {

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

}

/*
	Secondary Bit Clock Multiplexer
	00: Secondary Bit Clock = GPIO
	01: Secondary Bit Clock = SCLK
	10: Secondary Bit Clock = MISO
	11: Secondary Bit Clock = DOUT
*/
void FyberLabs_TAS2521::setAudioSecondaryBCLK(uint8_t clk) {

}
/*
	Secondary Word Clock Multiplexer
	00: Secondary Word Clock = GPIO
	01: Secondary Word Clock = SCLK
	10: Secondary Word Clock = MISO
	11: Secondary Word Clock = DOUT
*/
void FyberLabs_TAS2521::setAudioSecondaryWCLK(uint8_t clk) {

}
/*
	Secondary Data Input Multiplexer
	0: Secondary Data Input = GPIO
	1: Secondary Data Input = SCLK
*/
void FyberLabs_TAS2521::setAudioSecondaryMuxInput(uint8_t input) {

}
/*
void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryBCLK(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryBCLK(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryWCLK(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryWCLK(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceUsePrimaryDIN(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceUseSecondaryDIN(void) {

}

void FyberLabs_TAS2521::setAudioInterfaceBCLKPrimaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceBCLKPrimaryOutputSecondary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceBCLKSecondaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceBCLKSecondaryOutputSecondary(void) {

}

	void FyberLabs_TAS2521::setAudioInterfaceWCLKPrimaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceWCLKPrimaryOutputSecondary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceWCLKSecondaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceWCLKSecondaryOutputSecondary(void) {

}

void FyberLabs_TAS2521::setAudioInterfaceDOUTPrimaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceDOUTPrimaryOutputSecondary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceDOUTSecondaryOutputPrimary(void) {

}
void FyberLabs_TAS2521::setAudioInterfaceDOUTSecondaryOutputSecondary(void) {

}

void FyberLabs_TAS2521::I2CGeneralCallIgnored(void) {

}
void FyberLabs_TAS2521::I2CGeneralCallAccepted(void) {

}

bool FyberLabs_TAS2521::getDACPower(void) {

}
bool FyberLabs_TAS2521::getHPOUTPower(void) {

}

bool FyberLabs_TAS2521::getDACPGA(void) {

}
bool FyberLabs_TAS2521::getDACOverflow(void) {

}
bool FyberLabs_TAS2521::getminiDSP_DOverflow(void) {

}
bool FyberLabs_TAS2521::getHPOUTOverCurrentSticky(void) {

}
bool FyberLabs_TAS2521::getminiDSP_DStdInterruptSticky(void) {

}
bool FyberLabs_TAS2521::getminiDSP_DAuxInterruptSticky(void) {

}
bool FyberLabs_TAS2521::getHPOUTOverCurrent(void) {

}
bool FyberLabs_TAS2521::getminiDSP_DStdInterrupt(void) {

}
bool FyberLabs_TAS2521::getminiDSP_DAuxInterrupt(void) {

}

bool FyberLabs_TAS2521::getINT1HPOUTOverCurrent(void) {

}
void FyberLabs_TAS2521::setINT1HPOUTOverCurrentOn(void) {

}
void FyberLabs_TAS2521::setINT1HPOUTOverCurrentOff(void) {

}

bool FyberLabs_TAS2521::FyberLabs_TAS2521::getINT1miniDSP_DInterrupt(void) {

}
void FyberLabs_TAS2521::setINT1miniDSP_DInterruptOn(void) {

}
void FyberLabs_TAS2521::setINT1miniDSP_DInterruptOff(void) {

}

bool FyberLabs_TAS2521::getINT1PulseControl(void) {

}
void FyberLabs_TAS2521::setINT1PulseControlSingle(void) {

}
void FyberLabs_TAS2521::setINT1PulseControlMultiple(void) {

}

	bool FyberLabs_TAS2521::getINT2miniDSP_DInterrupt(void) {

}
void FyberLabs_TAS2521::setINT2miniDSP_DInterruptOn(void) {

}
void FyberLabs_TAS2521::setINT2miniDSP_DInterruptOff(void) {

}

bool FyberLabs_TAS2521::getINT2PulseControl(void) {

}
void FyberLabs_TAS2521::setINT2PulseControlSingle(void) {

}
void FyberLabs_TAS2521::setINT2PulseControlMultiple(void) {

}
*/
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

}
void FyberLabs_TAS2521::setGPIOOutZero(void) {

}
void FyberLabs_TAS2521::setGPIOOutOne(void) {

}

void FyberLabs_TAS2521::setDOUTBusKeeperEnabled(void) {

}
void FyberLabs_TAS2521::setDOUTBusKeeperDisabled(void) {

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
void FyberLabs_TAS2521::setDOUTMuxControl(uint8_t mux) {

}
void FyberLabs_TAS2521::setDOUTIsGPIO(void) {

}
void FyberLabs_TAS2521::setDOUTNotGPIO(void) {

}

/*
	DIN function control
00: DIN pin is disabled
01: DIN is enabled for Primary Data Input or General Purpose Clock input 10: DIN is used as General Purpose Input
11: Reserved. Do not use
*/
/*
void FyberLabs_TAS2521::setDINFunctionControl(uint8_t control) {

}
bool FyberLabs_TAS2521::getDINGPIOInput(void) {

}
*/
/*
	MISO function control
0000: MISO buffer disabled
0001: MISO is used for data output in SPI interface, is disabled for I2C interface 0010: MISO is General Purpose Output
0011: MISO is CLKOUT output
0100: MISO is INT1 output
0101: MISO is INT2 output
0110: Reserved
0111: Reserved
1000: MISO is Secondary Data Output for Audio Interface
1001: MISO is Secondary Bit Clock for Audio Interface
1010: MISO is Secondary Word Clock for Audio Interface
1011-1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setMISOFunctionControl(uint8_t control) {

}
void FyberLabs_TAS2521::setMISOGPIOOutput(bool bit) {

}

/*
	SCLK function control
00: SCLK pin is disabled
01: SCLK pin is enabled for SPI clock in SPI Interface mode or when in I2C Interface enabled for Secondary Data Input or Secondary Bit Clock Input or Secondary Word Clock. 10: SCLK is enabled as General Purpose Input
11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setSCLKFunctionControl(uint8_t control) {

}
/*
bool FyberLabs_TAS2521::getSCLKGPIOInput(void) {

}
*/
/*
	0 0000: The miniDSP_D will be used for signal processing 0 0001: DAC Signal Processing Block PRB_P1
0 0010: DAC Signal Processing Block PRB_P2
0 0011: DAC Signal Processing Block PRB_P3
0 0100-1 1111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setDACInstructionSet(uint8_t) {

}

void FyberLabs_TAS2521::setminiDSP_DConfigurationBitA(bool bit) {

}
void FyberLabs_TAS2521::setminiDSP_DConfigurationBitB(bool bit) {

}
void FyberLabs_TAS2521::setminiDSP_DConfigurationResetCounter(void) {

}
void FyberLabs_TAS2521::setminiDSP_DConfigurationNoResetCounter(void) {

}

void FyberLabs_TAS2521::setDACChannelSetupPowerUp(void) {

}
void FyberLabs_TAS2521::setDACChannelSetupPowerDown(void) {

}
/*
	DAC Data path Control
00: DAC data is disabled
01: DAC data is picked from Left Channel Audio Interface Data
10: DAC data is picked from Right Channel Audio Interface Data
11: DAC data is picked from Mono Mix of Left and Right Channel Audio Interface Data
*/
void FyberLabs_TAS2521::setDACChannelSetupDataPath(uint8_t path) {

}
/*
	DAC Channel Volume Control's Soft-Step control
	00: Soft-Stepping is 1 step per 1 DAC Word Clock
	01: Soft-Stepping is 1 step per 2 DAC Word Clocks
	10: Soft-Stepping is disabled
11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setDACChannelVolumeControl(uint8_t volume) {

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
void FyberLabs_TAS2521::setDACChannelAutoMuteControl(uint8_t mute) {

}
void FyberLabs_TAS2521::setDACChannelMute(void) {

}
void FyberLabs_TAS2521::setDACChannelUnmute(void) {

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

}

//Page1
void FyberLabs_TAS2521::setPage(uint8_t page) {

}

void FyberLabs_TAS2521::setMasterReferencePowerUp(void) {

}
void FyberLabs_TAS2521::setMasterReferencePowerDown(void) {

}
void FyberLabs_TAS2521::setPORPowerUp(void) {

}
void FyberLabs_TAS2521::setPORPowerDown(void) {

}
void FyberLabs_TAS2521::setLDOBandGapPowerUp(void) {

}
void FyberLabs_TAS2521::setLDOBandGapPowerDown(void) {

}

/*
	AVDD LDO Control
	00: AVDD LDO output is nominally 1.8V
	01: AVDD LDO output is nominally 1.6V
	10: AVDD LDO output is nominally 1.7V
	11: AVDD LDO output is nominally 1.5V
*/
void FyberLabs_TAS2521::setLDOControl(uint8_t voltage) {

}
void FyberLabs_TAS2521::setLDOPLLHPPowerUp(void) {

}
void FyberLabs_TAS2521::setLDOPLLHPPowerDown(void) {

}
void FyberLabs_TAS2521::setLDOShortCircuitDetectionOn(void) {

}
void FyberLabs_TAS2521::setLDOShortCircuitDetectionOff(void) {

}
void FyberLabs_TAS2521::setLDOSelectLow(void) {

}
void FyberLabs_TAS2521::setLDOSelectHigh(void) {

}

void FyberLabs_TAS2521::setPlaybackConfigurationDACLowPower(void) {

}
void FyberLabs_TAS2521::setPlaybackConfigurationDACHighPerformance(void) {

}
/*
	DAC PTM Control
	000: DAC in mode PTM_P3, PTM_P4
	001: DAC in mode PTM_P2
	010: DAC in mode PTM_P1
	011-111: Reserved. Do not use
*/
void FyberLabs_TAS2521::setPlaybackConfigurationDACPTMControl(uint8_t control) {

}

void FyberLabs_TAS2521::setDACPGAControlSoftSteppingOn(void) {

}
void FyberLabs_TAS2521::setDACPGAControlSoftSteppingOff(void) {

}
void FyberLabs_TAS2521::setDACPGAControlSoftSteppingNormal(void) {

}
void FyberLabs_TAS2521::setDACPGAControlSoftSteppingDouble(void) {

}

void FyberLabs_TAS2521::setOutputHPLPowerUp(void) {

}
void FyberLabs_TAS2521::setOutputHPLPowerDown(void) {

}
void FyberLabs_TAS2521::setAINLInputOn(void) {

}
void FyberLabs_TAS2521::setAINLInputOff(void) {

}
void FyberLabs_TAS2521::setAINRInputOn(void) {

}
void FyberLabs_TAS2521::setAINRInputOff(void) {

}

void FyberLabs_TAS2521::setFullChipCommonMode09V(void) {

}
void FyberLabs_TAS2521::setFullChipCommonMode075V(void) {

}
void FyberLabs_TAS2521::setHPOUTFullDrive(void) {

}
void FyberLabs_TAS2521::setHPOUTHalfDrive(void) {

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
void setHPOUTOverCurrentDebounce(uint8_t debounce) {

}
void setHPOUTOverCurrentLimiting(void) {

}
void setHPOUTOverCurrentPowerOff(void) {

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
void FyberLabs_TAS2521::setHPOUTRouting(uint8_t routing) {

}
void FyberLabs_TAS2521::setHPOUTDACRoutedDirect(void) {

}
void FyberLabs_TAS2521::setHPOUTDACRoutedIndirect(void) {

}
void FyberLabs_TAS2521::setHPOUTMixerPAttenuator(void) {

}
void FyberLabs_TAS2521::setHPOUTMixerPNotAttentuator(void) {

}
void FyberLabs_TAS2521::setHPOUTAINLAttenuator(void) {

}
void FyberLabs_TAS2521::setHPOUTAINLNotAttenuator(void) {

}
void FyberLabs_TAS2521::setHPOUTAINRAttenuator(void) {

}
void FyberLabs_TAS2521::setHPOUTAINRNotAttenuator(void) {

}

void FyberLabs_TAS2521::setHPOUTDriverMuted(void) {

}
void FyberLabs_TAS2521::setHPOUTDriverUnmuted(void) {

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

}

/*
	00: Soft-routing step time = 0ms
	01: Soft-routing step time = 50ms
	10: Soft-routing step time = 100ms
	11: Soft-routing step time = 200ms
*/
void FyberLabs_TAS2521::setHPOOUTDriverStartupSoftRoutingStepTime(uint8_t time) {

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
void FyberLabs_TAS2521::setHPOUTDriverStartupPowerUpTime(uint8_t time) {

}
/*
	00: Headphone amps power up time is determined with 25K resistance
	01: Headphone amps power up time is determined with 6K resistance
	10: Headphone amps power up time is determined with 2K resistance
	11: Reserved. Do not use
*/
void FyberLabs_TAS2521::setHPOUTDriverResistance(uint8_t resistance) {

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

}

//See note
void FyberLabs_TAS2521::setAINLRMixerPandMixerMForceOn(void) {

}
void FyberLabs_TAS2521::setAINLRMixerPandMixerMUnforcedOn(void) {

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

}

void FyberLabs_TAS2521::setSpeakerDriverPowerUp(void) {

}
void FyberLabs_TAS2521::setSpeakerDriverPowerDown(void) {

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
void FyberLabs_TAS2521::setSpeakerAmplifierVolume(uint8_t volume) {

}
/*
bool FyberLabs_TAS2521::getHPOUTAppliedGain(void) {

}
bool FyberLabs_TAS2521::getAINLMixPGAHPOUTAppliedGain(void) {

}
bool FyberLabs_TAS2521::getAINLMixPGAAppliedVolume(void) {

}
bool FyberLabs_TAS2521::getAINRMixPGAAppliedVolume(void) {

}
*/
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
void FyberLabs_TAS2521::setReferencePowerUpDelay(uint8_t delay) {

}
/*
//Page 44 / Register 0: Page Select Register - 0x2C / 0x00
//Page 45 - 52 / Register 0: Page Select Register - 0x2D - 0x34 / 0x00
//9 Pages DAC coeff A
//256 coeffs x 24bits
//30 coeffs per page
//4 reg per coeff
void FyberLabs_TAS2521::setDACProgrammableCoefficientsPageA(uint8_t page) {

}

void FyberLabs_TAS2521::setDACAdaptiveFilterOn(void) {

}
void FyberLabs_TAS2521::setDACAdaptiveFilterOff(void) {

}
uint8_t FyberLabs_TAS2521::getDACAdaptiveFilterControlFlag(void) {

}
void FyberLabs_TAS2521::setDACAdaptiveFilterSwitch(void) {

}
void FyberLabs_TAS2521::setDACAdaptiveFilterNotSwitch(void) {

}

//Page 44 / Register 8 - 127: DAC Coefficient Buffer-A C(0:29) - 0x2C / 0x08 - 0x7F
//Page 45 - 52 / Register 8 - 127: DAC Coefficients Buffer-A C(30:255) - 0x2D - 0x34 / 0x08 -0x7F
void FyberLabs_TAS2521::setDACCoefficientBufferA(uint8_t reg, uint8_t value) {

}
uint8_t FyberLabs_TAS2521::getDACCoefficientBufferA(uint8_t reg) {

}

//Page 62 - 70 / Register 0: Page Select Register - 0x3E - 0x46 / 0x00
//9 Pages DAC coeff B
//256 coeffs x 24bits
//30 coeffs per page
//4 reg per coeff
void FyberLabs_TAS2521::setDACProgrammableCoefficientsPageB(uint8_t page) {

}

//Page 62 - 70 / Register 8 -127: DAC Coefficients Buffer-B C(0:255) - 0x3E - 0x46 / 0x08 - 0x7F
void FyberLabs_TAS2521::setDACCoefficientBufferB(uint8_t reg, uint8_t value) {

}
uint8_t FyberLabs_TAS2521::getDACCoefficientBufferB(uint8_t reg) {

}

//Page 152 - 169 / Register 0: Page Select Register - 0x98 - 0xA9 / 0x00
//18 Pages DAC Inst
//1024 coeffs x 24bits
//30 insts per page
//4 reg per coeff
void FyberLabs_TAS2521::setDACminiDSPInstructionPage(uint8_t page) {

}

//Page 152 - 169 / Register 8 - 127: DAC Instruction Registers - 0x98 - 0xA9 / 0x08 - 0x7F
void FyberLabs_TAS2521::setDACminiDSPInstruction(uint8_t reg, uint8_t value) {

}
uint8_t FyberLabs_TAS2521::getDACminiDSPInstruction(uint8_t reg) {

}

*/

}
