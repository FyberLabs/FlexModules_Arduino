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

#ifndef FYBERLABS_TAS2521_H
#define FYBERLABS_TAS2521_H


#include "Arduino.h"
#include "Wire.h"

#define TAS2521_DEBUG 0
#define TAS2521_I2CADDR 0x18

class FyberLabs_TAS2521 {
  public:
  	FyberLabs_TAS2521();

  	void begin(void);
  	void reset(void);

  	typedef enum {
  		TAS2521_MCLK	= 0x00,
  		TAS2521_BCLK	= 0x01,
  		TAS2521_GPIO	= 0x02,
  		TAS2521_PLL		= 0x03,
  		TAS2521_DAC		= 0x04,  //CDIV
  		TAS2521_DAC_MOD	= 0x05,  //CDIV
  	} TAS2521clkmux_t;


  	void setPLLCLKRangeLow(void);
  	void setPLLCLKRangeHigh(void);
  	/*
  		Select PLL Input Clock
		00: MCLK pin is input to PLL
		01: BCLK pin is input to PLL
		10: GPIO pin is input to PLL
		11: DIN pin is input to PLL
  	*/
  	void setPLLCLK(uint8_t clkmux);
  	/*
  		Select CODEC_CLKIN
		00: MCLK pin is CODEC_CLKIN
		01: BCLK pin is CODEC_CLKIN
		10: GPIO pin is CODEC_CLKIN
		11: PLL Clock is CODEC_CLKIN
  	*/
  	void setCODECCLK(uint8_t clkmux);

  	void setPLLPowerDown(void);
  	void setPLLPowerUp(void);

  	/*
  		000: PLL divider P = 8
  		001: PLL divider P = 1
  		010: PLL divider P = 2
  		...
		110: PLL divider P = 6
		111: PLL divider P = 7
  	*/
  	void setPLLDividerP(uint8_t P);

  	/*
  		0000: Reserved. Do not use
  		0001: PLL multiplier R = 1
  		0010: PLL multiplier R = 2
  		0011: PLL multipler R = 3
  		0100: PLL multipler R = 4
		...
		0101...0111: Reserved. Do not use
  	*/
  	void setPLLDividerR(uint8_t R);

  	/*
  		PLL divider J value
		00 0000...00 0011: Do not use 00 0100: J = 4
		00 0101: J = 5
		...
		11 1110: J = 62
		11 1111: J = 63
  	*/
  	void setPLLDividerJ(uint8_t J);

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
  	void setPLLDividerDMSB(uint8_t DMSB);

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
  	void setPLLDividerDLSB(uint8_t DLSB);

	  void setNDACPowerDown(void);
  	void setNDACPowerUp(void);
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
  	void setNDACCLK(uint8_t clk);

	  void setMDACPowerDown(void);
  	void setMDACPowerUp(void);
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
  	void setMDACCLK(uint8_t clk);

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
  	void setDACOSRMSB(uint8_t DOSRMSB);

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
  	void setDACOSRLSB(uint8_t DOSRLSB);

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
  	void setminiDSP_DMSB(uint8_t IDACMSB);
  	/*
  		miniDSP_D IDAC (7:0) setting. Use when miniDSP_D is in use for signal processing (page 0,Reg 60) miniDSP_D IDAC(14:0)
		000 0000 0000 0000: miniDSP_D 000 0000 0000 0001: miniDSP_D 000 0000 0000 0010: miniDSP_D ......
		111 1111 1111 1110: miniDSP_D
		111 1111 1111 1111: miniDSP_D
		Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 ) Note: Page-0, Reg-16 should be programmed immediately after Page-0, Reg-15.
  	*/
  	void setminiDSP_DLSB(uint8_t IDACLSB);

  	/*
  		miniDSP_D interpolation factor setting. Used when miniDSP_D is in use for signal processing (page 0,Reg 60)
		0000 : Interpolation factor in miniDSP_D(INTERP) = 16
		0001: Interpolation factor in miniDSP_D(INTERP)= 1
		0010: Interpolation factor in miniDSP_D(INTERP) = 2 ...
		1110: Interpolation factor in miniDSP_D(INTERP) = 14
  	*/
  	void setminiDSP_DIntFactor(uint8_t IntFactor);

  	/*
  		CDIV_CLKIN Clock Selection
		000: CDIV_CLKIN = MCLK
		001: CDIV_CLKIN = BCLK
		010: CDIV_CLKIN = DIN
		011: CDIV_CLKIN = PLL_CLK
		100: CDIV_CLKIN = DAC_CLK
		101: CDIV_CLKIN = DAC_MOD_CLK
  	*/
  	void setCDIV_CLKIN(uint8_t clkin);

	  void setCLKOUTMPowerDown(void);
  	void setCLKOUTMPowerUp(void);

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
  	void setCLKOUTMDivider(uint8_t divider);

  	/*
  		Audio Interface Selection
  		00: Audio Interface = I2S
  		01: Audio Interface = DSP
  		10: Audio Interface = RJF
  		11: Audio Interface = LJF
  	*/
  	void setAudioInterfaceSelect(uint8_t interface);
  	/*
  		Audio Data Word length
		00: Data Word length = 16 bits
		01: Data Word length = 20 bits
		10: Data Word length = 24 bits
		11: Data Word length = 32 bits
  	*/
  	void setAudioDateWordLength(uint8_t wordlength);
  	void setAudioBCLKin(void);
  	void setAudioBCLKout(void);
  	void setAudioWCLKin(void);
  	void setAudioWCLKout(void);

  	/*
  		Data Offset Value
		0000 0000: Data Offset = 0 BCLK's
		0000 0001: Data Offset = 1 BCLK's
		...
		1111 1110: Data Offset = 254 BCLK's
		1111 1111: Data Offset = 255 BCLK's
  	*/
  	void setAudioBCLKDataOffset(uint8_t offset);

  	void setAudioBCLKPolarityDefault(void);
  	void setAudioBCLKPolarityInverted(void);

  	void setAudioBWCLKPowerAlways(void);
  	void setAudioBWCLKPowerCodec(void);

  	void setAudioBDIV_CLKINDAC(void);
  	void setAudioBDIV_CLKINDAC_MOD(void);

  	void setAudioBCLKNDividerPowerUp(void);
  	void setAudioBCLKNDividerPowerDown(void);
  	/*
  		BCLK N Divider value
		000 0000: BCLK divider N = 128
		000 0001: BCLK divider N = 1
		...
		111 1110: BCLK divider N = 126
		111 1111: BCLK divider N = 127
  	*/
  	void setAudioBCLKNDivider(uint8_t divider);

  	/*
  		Secondary Bit Clock Multiplexer
  		00: Secondary Bit Clock = GPIO
  		01: Secondary Bit Clock = SCLK
  		10: Secondary Bit Clock = MISO
  		11: Secondary Bit Clock = DOUT
  	*/
  	void setAudioSecondaryBCLK(uint8_t clk);
  	/*
  		Secondary Word Clock Multiplexer
  		00: Secondary Word Clock = GPIO
  		01: Secondary Word Clock = SCLK
  		10: Secondary Word Clock = MISO
  		11: Secondary Word Clock = DOUT
  	*/
  	void setAudioSecondaryWCLK(uint8_t clk);
  	/*
  		Secondary Data Input Multiplexer
  		0: Secondary Data Input = GPIO
  		1: Secondary Data Input = SCLK
  	*/
  	void setAudioSecondaryMuxInput(uint8_t input);

  	void setAudioInterfaceUsePrimaryBCLK(void);
  	void setAudioInterfaceUseSecondaryBCLK(void);
  	void setAudioInterfaceUsePrimaryWCLK(void);
  	void setAudioInterfaceUseSecondaryWCLK(void);
  	void setAudioInterfaceUsePrimaryDIN(void);
  	void setAudioInterfaceUseSecondaryDIN(void);

  	void setAudioInterfaceBCLKPrimaryOutputPrimary(void);
  	void setAudioInterfaceBCLKPrimaryOutputSecondary(void);
	  void setAudioInterfaceBCLKSecondaryOutputPrimary(void);
  	void setAudioInterfaceBCLKSecondaryOutputSecondary(void);

   	void setAudioInterfaceWCLKPrimaryOutputPrimary(void);
  	void setAudioInterfaceWCLKPrimaryOutputSecondary(void);
	  void setAudioInterfaceWCLKSecondaryOutputPrimary(void);
  	void setAudioInterfaceWCLKSecondaryOutputSecondary(void);

  	void setAudioInterfaceDOUTPrimaryOutputPrimary(void);
  	void setAudioInterfaceDOUTPrimaryOutputSecondary(void);
  	void setAudioInterfaceDOUTSecondaryOutputPrimary(void);
  	void setAudioInterfaceDOUTSecondaryOutputSecondary(void);

  	void I2CGeneralCallIgnored(void);
  	void I2CGeneralCallAccepted(void);

  	bool getDACPower(void);
  	bool getHPOUTPower(void);

  	bool getDACPGA(void);
  	bool getDACOverflow(void);
  	bool getminiDSP_DOverflow(void);
  	bool getHPOUTOverCurrentSticky(void);
  	bool getminiDSP_DStdInterruptSticky(void);
  	bool getminiDSP_DAuxInterruptSticky(void);
  	bool getHPOUTOverCurrent(void);
  	bool getminiDSP_DStdInterrupt(void);
  	bool getminiDSP_DAuxInterrupt(void);

  	bool getINT1HPOUTOverCurrent(void);
  	void setINT1HPOUTOverCurrentOn(void);
  	void setINT1HPOUTOverCurrentOff(void);

  	bool getINT1miniDSP_DInterrupt(void);
  	void setINT1miniDSP_DInterruptOn(void);
  	void setINT1miniDSP_DInterruptOff(void);

  	bool getINT1PulseControl(void);
  	void setINT1PulseControlSingle(void);
  	void setINT1PulseControlMultiple(void);

   	bool getINT2miniDSP_DInterrupt(void);
  	void setINT2miniDSP_DInterruptOn(void);
  	void setINT2miniDSP_DInterruptOff(void);

  	bool getINT2PulseControl(void);
  	void setINT2PulseControlSingle(void);
  	void setINT2PulseControlMultiple(void);

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
  	void setGPIOControl(uint8_t control);
  	void setGPIOOutZero(void);
  	void setGPIOOutOne(void);

  	void setDOUTBusKeeperEnabled(void);
  	void setDOUTBusKeeperDisabled(void);
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
  	void setDOUTMuxControl(uint8_t mux);
  	void setDOUTIsGPIO(void);
  	void setDOUTNotGPIO(void);

  	/*
  		DIN function control
		00: DIN pin is disabled
		01: DIN is enabled for Primary Data Input or General Purpose Clock input 10: DIN is used as General Purpose Input
		11: Reserved. Do not use
  	*/
  	void setDINFunctionControl(uint8_t control);
  	bool getDINGPIOInput(void);

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
  	void setMISOFunctionControl(uint8_t control);
  	void setMISOGPIOOutput(bool bit);

  	/*
  		SCLK function control
		00: SCLK pin is disabled
		01: SCLK pin is enabled for SPI clock in SPI Interface mode or when in I2C Interface enabled for Secondary Data Input or Secondary Bit Clock Input or Secondary Word Clock. 10: SCLK is enabled as General Purpose Input
		11: Reserved. Do not use
  	*/
  	void setSCLKFunctionControl(uint8_t control);
  	bool getSCLKGPIOInput(void);

  	/*
  		0 0000: The miniDSP_D will be used for signal processing 0 0001: DAC Signal Processing Block PRB_P1
		0 0010: DAC Signal Processing Block PRB_P2
		0 0011: DAC Signal Processing Block PRB_P3
		0 0100-1 1111: Reserved. Do not use
  	*/
  	void setDACInstructionSet(uint8_t);

  	void setminiDSP_DConfigurationBitA(bool bit);
  	void setminiDSP_DConfigurationBitB(bool bit);
  	void setminiDSP_DConfigurationResetCounter(void);
  	void setminiDSP_DConfigurationNoResetCounter(void);

  	void setDACChannelSetupPowerUp(void);
  	void setDACChannelSetupPowerDown(void);
  	/*
  		DAC Data path Control
		00: DAC data is disabled
		01: DAC data is picked from Left Channel Audio Interface Data
		10: DAC data is picked from Right Channel Audio Interface Data
		11: DAC data is picked from Mono Mix of Left and Right Channel Audio Interface Data
  	*/
  	void setDACChannelSetupDataPath(uint8_t path);
  	/*
  		DAC Channel Volume Control's Soft-Step control
  		00: Soft-Stepping is 1 step per 1 DAC Word Clock
  		01: Soft-Stepping is 1 step per 2 DAC Word Clocks
  		10: Soft-Stepping is disabled
		11: Reserved. Do not use
  	*/
	  void setDACChannelVolumeControl(uint8_t volume);

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
  	void setDACChannelAutoMuteControl(uint8_t mute);
  	void setDACChannelMute(void);
  	void setDACChannelUnmute(void);
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
  	void setDACChannelDigitalVolumeControl(uint8_t volume);

  	//Page1
  	void setPage(uint8_t page);

  	void setMasterReferencePowerUp(void);
  	void setMasterReferencePowerDown(void);
  	void setPORPowerUp(void);
  	void setPORPowerDown(void);
  	void setLDOBandGapPowerUp(void);
  	void setLDOBandGapPowerDown(void);

  	/*
  		AVDD LDO Control
  		00: AVDD LDO output is nominally 1.8V
  		01: AVDD LDO output is nominally 1.6V
  		10: AVDD LDO output is nominally 1.7V
  		11: AVDD LDO output is nominally 1.5V
  	*/
  	void setLDOControl(uint8_t voltage);
  	void setLDOPLLHPPowerUp(void);
  	void setLDOPLLHPPowerDown(void);
  	void setLDOShortCircuitDetectionOn(void);
  	void setLDOShortCircuitDetectionOff(void);
  	void setLDOSelectLow(void);
  	void setLDOSelectHigh(void);

  	void setPlaybackConfigurationDACLowPower(void);
  	void setPlaybackConfigurationDACHighPerformance(void);
  	/*
  		DAC PTM Control
  		000: DAC in mode PTM_P3, PTM_P4
  		001: DAC in mode PTM_P2
  		010: DAC in mode PTM_P1
  		011-111: Reserved. Do not use
  	*/
  	void setPlaybackConfigurationDACPTMControl(uint8_t control);

  	void setDACPGAControlSoftSteppingOn(void);
  	void setDACPGAControlSoftSteppingOff(void);
  	void setDACPGAControlSoftSteppingNormal(void);
  	void setDACPGAControlSoftSteppingDouble(void);

  	void setOutputHPLPowerUp(void);
  	void setOutputHPLPowerDown(void);
  	void setAINLInputOn(void);
  	void setAINLInputOff(void);
  	void setAINRInputOn(void);
  	void setAINRInputOff(void);

  	void setFullChipCommonMode09V(void);
  	void setFullChipCommonMode075V(void);
  	void setHPOUTFullDrive(void);
  	void setHPOUTHalfDrive(void);

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
  	void setHPOUTOverCurrentDebounce(uint8_t debounce);
  	void setHPOUTOverCurrentLimiting(void);
  	void setHPOUTOverCurrentPowerOff(void);

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
  	void setHPOUTRouting(uint8_t routing);
  	void setHPOUTDACRoutedDirect(void);
  	void setHPOUTDACRoutedIndirect(void);
  	void setHPOUTMixerPAttenuator(void);
  	void setHPOUTMixerPNotAttentuator(void);
  	void setHPOUTAINLAttenuator(void);
  	void setHPOUTAINLNotAttenuator(void);
  	void setHPOUTAINRAttenuator(void);
  	void setHPOUTAINRNotAttenuator(void);

  	void setHPOUTDriverMuted(void);
  	void setHPOUTDriverUnmuted(void);
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
  	void setHPOUTDriverGain(uint8_t gain);

  	/*
  		00: Soft-routing step time = 0ms
  		01: Soft-routing step time = 50ms
  		10: Soft-routing step time = 100ms
  		11: Soft-routing step time = 200ms
  	*/
  	void setHPOOUTDriverStartupSoftRoutingStepTime(uint8_t time);

  	/*
  		0000: Slow power up of headphone amp's is disabled
  		0001: Headphone amps power up slowly in 0.5 time constants
  		0010: Headphone amps power up slowly in 0.625 time constants
  		0011; Headphone amps power up slowly in 0.725 time constants
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
  	void setHPOUTDriverStartupPowerUpTime(uint8_t time);
  	/*
  		00: Headphone amps power up time is determined with 25K resistance
  		01: Headphone amps power up time is determined with 6K resistance
  		10: Headphone amps power up time is determined with 2K resistance
  		11: Reserved. Do not use
  	*/
  	void setHPOUTDriverResistance(uint8_t resistance);

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
  	void setHPOUTVolume(uint8_t volume);

  	//See note
  	void setAINLRMixerPandMixerMForceOn(void);
  	void setAINLRMixerPandMixerMUnforcedOn(void);
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
  	void setAINLVolume(uint8_t volume);
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
  	void setAINRVolume(uint8_t volume);

  	void setSpeakerDriverPowerUp(void);
  	void setSpeakerDriverPowerDown(void);

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
  	void setSpeakerVolume(uint8_t volume);

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
  	void setSpeakerAmplifierVolume(uint8_t volume);

  	bool getHPOUTAppliedGain(void);
  	bool getAINLMixPGAHPOUTAppliedGain(void);
  	bool getAINLMixPGAAppliedVolume(void);
  	bool getAINRMixPGAAppliedVolume(void);

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
  	void setReferencePowerUpDelay(uint8_t delay);

  	//Page 44 / Register 0: Page Select Register - 0x2C / 0x00
  	//Page 45 - 52 / Register 0: Page Select Register - 0x2D - 0x34 / 0x00
  	//9 Pages DAC coeff A
  	//256 coeffs x 24bits
  	//30 coeffs per page
  	//4 reg per coeff
  	void setDACProgrammableCoefficientsPageA(uint8_t page);

  	void setDACAdaptiveFilterOn(void);
  	void setDACAdaptiveFilterOff(void);
  	uint8_t getDACAdaptiveFilterControlFlag(void);
  	void setDACAdaptiveFilterSwitch(void);
  	void setDACAdaptiveFilterNotSwitch(void);

  	//Page 44 / Register 8 - 127: DAC Coefficient Buffer-A C(0:29) - 0x2C / 0x08 - 0x7F
  	//Page 45 - 52 / Register 8 - 127: DAC Coefficients Buffer-A C(30:255) - 0x2D - 0x34 / 0x08 -0x7F
  	void setDACCoefficientBufferA(uint8_t reg, uint8_t value);
  	uint8_t getDACCoefficientBufferA(uint8_t reg);

  	//Page 62 - 70 / Register 0: Page Select Register - 0x3E - 0x46 / 0x00
  	//9 Pages DAC coeff B
  	//256 coeffs x 24bits
  	//30 coeffs per page
  	//4 reg per coeff
  	void setDACProgrammableCoefficientsPageB(uint8_t page);

  	//Page 62 - 70 / Register 8 -127: DAC Coefficients Buffer-B C(0:255) - 0x3E - 0x46 / 0x08 - 0x7F
  	void setDACCoefficientBufferB(uint8_t reg, uint8_t value);
  	uint8_t getDACCoefficientBufferB(uint8_t reg);

  	//Page 152 - 169 / Register 0: Page Select Register - 0x98 - 0xA9 / 0x00
  	//18 Pages DAC Inst
  	//1024 coeffs x 24bits
  	//30 insts per page
  	//4 reg per coeff
  	void setDACminiDSPInstructionPage(uint8_t page);

  	//Page 152 - 169 / Register 8 - 127: DAC Instruction Registers - 0x98 - 0xA9 / 0x08 - 0x7F
  	void setDACminiDSPInstruction(uint8_t reg, uint8_t value);
  	uint8_t getDACminiDSPInstruction(uint8_t reg);



  private:
  	uint8_t _i2c_address;
  	uint8_t _page;

  	void switchPage(uint8_t page);
  	uint8_t read8(uint8_t reg);
  	void write8(uint8_t reg, uint8_t data);

};

#endif

//TODO: Turn each of these recipes into public methods that run them
/*
The following example EVM I2C register control scripts can be taken directly for the TAS2521 EVM setup. The # marks a comment line, w marks an I2C write command followed by the device address, the I2C register address and the value. The EVM I2C register control scripts follows to show how to set up the TAS2521 in playback mode with fS = 44.1 kHz and MCLK = 11.2896 MHz.
*/

//Example Register Setup to Play Digital Data Through DAC and Headphone/Speaker Outputs
# I2C Script to Setup the device in Playback Mode
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# This script set DAC output routed to HP Driver and Class-D driver via Mixer
# # ==> comment delimiter

# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
# Page switch to Page 0
W 30 00 00
#         PLL_clkin = MCLK, codec_clkin = PLL_CLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=03)
w 30 04 03
# Power up PLL, set P=1, R=1, (Page-0, Reg-5)
w 30 05 91
# Set J=4, (Page-0, Reg-6)
w 30 06 04
# D = 0000, D(13:8) = 0, (Page-0, Reg-7)
w 30 07 00
#           D(7:0) = 0,  (Page-0, Reg-8)
w 30 08 00
# add delay of 15 ms for PLL to lock
d 15
#        DAC NDAC Powered up, NDAC=4 (P0, R11, D7=1, D6-D0=0000100)
W 30 0B 84
#        DAC MDAC Powered up, MDAC=2 (P0, R12, D7=1, D6-D0=0000010)
W 30 0C 82
#        DAC OSR(9:0)-> DOSR=128 (P0, R12, D1-D0=00)
W 30 0D 00
#        DAC OSR(9:0)-> DOSR=128 (P0, R13, D7-D0=10000000)
W 30 0E 80
# Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode. (P0, R27, D7-
D6=00, D5-D4=00, D3-D2=00)
W 30 1B 00
# Data slot offset 00 (P0, R28, D7-D0=0000)
W 30 1C 00
# Dac Instruction programming PRB #2 for Mono routing. Type interpolation (x8) and 3 programmable
Biquads. (P0, R60, D4-D0=0010)
W 30 3C 02
# Page Switch to Page 1
W 30 00 01
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Output common mode for DAC set to 0.9V (default) (P1, R10)
W 30 0A 00
# Mixer P output is connected to HP Out Mixer (P1, R12, D2=1)
w 30 0C 04
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# No need to enable Mixer M and Mixer P, AINL Voulme, 0dB Gain (P1, R24, D7=1, D6-D0=0000000)
W 30 18 00
# Power up HP (P1, R9, D5=1)
w 30 09 20
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
#          SPK attn. Gain =0dB (P1, R46, D6-D0=000000)
W 30 2E 00
#          SPK driver Gain=6.0dB (P1, R48, D6-D4=001)
W 30 30 10
#          SPK powered up (P1, R45, D1=1)
W 30 2D 02
# Page switch to Page 0
W 30 00 00
# DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
W 30 3F 90
# DAC digital gain 0dB (P0, R65, D7-D0=00000000)
W 30 41 00
# DAC volume not muted. (P0, R64, D3=0, D2=1)
W 30 40 04
#

//Example Register Setup to Play Digital Data Through DAC and Headphone Output
# I2C Script to Setup the device in Playback Mode
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# This script set DAC output routed to only HP Driver
# # ==> comment delimiter
#
# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
Page switch to Page 0
W 30 00 00
#         CODEC_CLKIN=MCLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=00)
W 30 04 00
#        DAC NDAC Powered up, NDAC=1 (P0, R11, D7=1, D6-D0=0000001)
W 30 0B 81
#        DAC MDAC Powered up, MDAC=2 (P0, R12, D7=1, D6-D0=0000010)
W 30 0C 82
#        DAC OSR(9:0)-> DOSR=128 (P0, R12, D1-D0=00)
W 30 0D 00
#        DAC OSR(9:0)-> DOSR=128 (P0, R13, D7-D0=10000000)
W 30 0E 80
# Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode. (P0, R27, D7-
D6=00, D5-D4=00, D3-D2=00)
W 30 1B 00
# Data slot offset 00 (P0, R28, D7-D0=0000)
W 30 1C 00
# Dac Instruction programming PRB #2 for Mono routing. Type interpolation (x8) and 3 programmable
Biquads. (P0, R60, D4-D0=0010)
W 30 3C 02
# Page Switch to Page 1
W 30 00 01
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Output common mode for DAC set to 0.9V (default) (P1, R10)
W 30 0A 00
# DAC output is routed directly to HP driver (P1, R12, D3=1)
w 30 0C 08
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# Power up HP (P1, R9, D5=1)
w 30 09 20
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
# Page switch to Page 0
W 30 00 00
# DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
W 30 3F 90
# DAC digital gain 0dB (P0, R65, D7-D0=00000000)
W 30 41 00
# DAC volume not muted. (P0, R64, D3=0, D2=1)
W 30 40 04
#

//Example Register Setup to Play AINL and AINR Through Headphone/Speaker Outputs
# I2C Script to Setup the device in Playback Mode
# This script set AINL and AINR inputs routed to HP Driver and Class-D driver via Mixer
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# # ==> comment delimiter
#
# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Enable AINL and AINR (P1, R9, D1-D0=11)
w 30 09 03
# AINL/R to HP driver via Mixer P (P1, R12, D7-D6=11, D2=1)
w 30 0C C4
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# Enable Mixer P and Mixer M, AINL Voulme, 0dB Gain (P1, R24, D7=1, D6-D0=0000000)
W 30 18 80
# Enable AINL and AINR and Power up HP (P1, R9, D5=1, D1-D0=11)
w 30 09 23
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
#          SPK attn. Gain =0dB (P1, R46, D6-D0=000000)
W 30 2E 00
#          SPK driver Gain=6.0dB (P1, R48, D6-D4=001)
W 30 30 10
#          SPK powered up (P1, R45, D1=1)
W 30 2D 02
#

//Example Register Setup to Play AINL and AINR Through Headphone Output
# I2C Script to Setup the device in Playback Mode
# This script set AINL and AINR inputs routed to only HP Driver
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# # ==> comment delimiter
#
# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Enable AINL and AINR (P1, R9, D1-D0=11)
w 30 09 03
# AINL/R to HP driver not via Mixer P (P1, R12, D1-D0=11)
w 30 0C 03
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# Not enable HP Out Mixer, AINL Voulme, 0dB Gain (P1, R24, D7=0, D6-D0=0000000)
W 30 18 00
# Enable AINL and AINR and Power up HP (P1, R9, D5=1, D1-D0=11)
w 30 09 23
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
#

//Example Register Setup to Play Digital Data Through DAC and Headphone/Speaker Outputs with 3 programmable Biquads
# I2C Script to Setup the device in Playback Mode #2
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# This script set DAC output routed to HP Driver and Class-
D driver via Mixer with 3 programmable Biquads.
# # ==> comment delimiter
#
# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
# Page switch to Page 0
W 30 00 00
#         CODEC_CLKIN=MCLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=00)
W 30 04 00
#        DAC NDAC Powered up, NDAC=1 (P0, R11, D7=1, D6-D0=0000001)
W 30 0B 81
#        DAC MDAC Powered up, MDAC=2 (P0, R12, D7=1, D6-D0=0000010)
W 30 0C 82
#        DAC OSR(9:0)-> DOSR=128 (P0, R12, D1-D0=00)
W 30 0D 00
#        DAC OSR(9:0)-> DOSR=128 (P0, R13, D7-D0=10000000)
W 30 0E 80
# Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode. (P0, R27, D7-
D6=00, D5-D4=00, D3-D2=00)
W 30 1B 00
# Data slot offset 00 (P0, R28, D7-D0=0000)
W 30 1C 00
# Dac Instruction programming PRB #2 for Mono routing. Type interpolation (x8) and 3 programmable
Biquads. (P0, R60, D4-D0=0010)
W 30 3C 02
##########--------------- BEGIN COEFFICIENTS --------------------------------------
# reg 00 - Page Select Register = 44
# sets active page to page 44 for 3-BQs (BQ-A, BQ-B, BQ-C)
w 30 00 2C
#
#-----------------------------------------------------------------------
#  BQ-A = 100Hz HP
#-----------------------------------------------------------------------
# reg 12/13/14 - N0 Coefficient
w 30 0C 7E B7 7B
# reg 16/17/18 - N1 Coefficient
w 30 10 81 48 85
# reg 20/21/22 - N2 Coefficient
w 30 14 7E B7 7B
# reg 24/25/26 - D1 Coefficient
w 30 18 7E B5 D5
# reg 28/29/30 - D2 Coefficient
w 30 1C 82 8D BE
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
# BQ-C=5KHzNotchBW=125
#-----------------------------------------------------------------------
# reg 52/53/54 - N0 Coefficient
w 30 34 7E DE C5
# reg 56/57/58 - N1 Coefficient
w 30 38 9F FB C8
# reg 60/61/62 - N2 Coefficient
w 30 3C 7E DE C5
# reg 64/65/66 - D1 Coefficient
w 30 40 60 04 38
# reg 68/69/70 - D2 Coefficient
w 30 44 82 42 74
##########--------------- END COEFFICIENTS OF Notch Filters  ------------------------
#######################################################
# Page Switch to Page 1
W 30 00 01
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Output common mode for DAC set to 0.9V (default) (P1, R10)
W 30 0A 00
# Mixer P output is connected to HP Out Mixer (P1, R12, D2=1)
w 30 0C 04
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# Power up HP (P1, R9, D5=1)
w 30 09 20
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
#          SPK attn. Gain =0dB (P1, R46, D6-D0=000000)
W 30 2E 00
#          SPK driver Gain=6.0dB (P1, R48, D6-D4=001)
W 30 30 10
#          SPK powered up (P1, R45, D1=1)
W 30 2D 02
# Page switch to Page 0
W 30 00 00
# DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
W 30 3F 90
# DAC digital gain 0dB (P0, R65, D7-D0=00000000)
W 30 41 00
# DAC volume not muted. (P0, R64, D3=0, D2=1)
W 30 40 04
#

//Example Register Setup to Play Digital Data Through DAC and Headphone/Speaker Outputs with 6 programmable Biquads
# I2C Script to Setup the device in Playback Mode #3
# Key: w 30 XX YY ==> write to I2C address 0x30, to register 0xXX, data 0xYY
# This script set DAC output routed to HP Driver and Class-
D driver via Mixer with 6 programmable Biquads.
# # ==> comment delimiter
#
# Page switch to Page 0
W 30 00 00
# Assert Software reset (P0, R1, D0=1)
W 30 01 01
# Page Switch to Page 1
W 30 00 01
# LDO output programmed as 1.8V and Level shifters powered up. (P1, R2, D5-D4=00, D3=0)
W 30 02 00
# Page switch to Page 0
W 30 00 00
#         CODEC_CLKIN=MCLK, MCLK should be 11.2896MHz (P0, R4, D1-D0=00)
W 30 04 00
#        DAC NDAC Powered up, NDAC=1 (P0, R11, D7=1, D6-D0=0000001)
W 30 0B 81
#        DAC MDAC Powered up, MDAC=2 (P0, R12, D7=1, D6-D0=0000010)
W 30 0C 82
#        DAC OSR(9:0)-> DOSR=128 (P0, R12, D1-D0=00)
W 30 0D 00
#        DAC OSR(9:0)-> DOSR=128 (P0, R13, D7-D0=10000000)
W 30 0E 80
# Codec Interface control Word length = 16bits, BCLK&WCLK inputs, I2S mode. (P0, R27, D7-
D6=00, D5-D4=00, D3-D2=00)
W 30 1B 00
# Data slot offset 00 (P0, R28, D7-D0=0000)
W 30 1C 00
# Dac Instruction programming PRB #3 for Mono routing. Type B nterpolation (x4) and 6
programmable Biquads. (P0, R60, D4-D0=0011)
W 30 3C 03
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
# Page Switch to Page 1
W 30 00 01
# Master Reference Powered on (P1, R1, D4=1)
W 30 01 10
# Output common mode for DAC set to 0.9V (default) (P1, R10)
W 30 0A 00
# Mixer P output is connected to HP Out Mixer (P1, R12, D2=1)
w 30 0C 04
# HP Voulme, 0dB Gain (P1, R22, D6-D0=0000000)
W 30 16 00
# Power up HP (P1, R9, D5=1)
w 30 09 20
# Unmute HP with 0dB gain (P1, R16, D4=1)
w 30 10 00
#          SPK attn. Gain =0dB (P1, R46, D6-D0=000000)
W 30 2E 00
#          SPK driver Gain=6.0dB (P1, R48, D6-D4=001)
W 30 30 10
#          SPK powered up (P1, R45, D1=1)
W 30 2D 02
# Page switch to Page 0
W 30 00 00
# DAC powered up, Soft step 1 per Fs. (P0, R63, D7=1, D5-D4=01, D3-D2=00, D1-D0=00)
W 30 3F 90
# DAC digital gain 0dB (P0, R65, D7-D0=00000000)
W 30 41 00
# DAC volume not muted. (P0, R64, D3=0, D2=1)
W 30 40 04
#
