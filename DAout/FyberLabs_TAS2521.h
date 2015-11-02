//
//TODO make class methods based on usage examples in http://www.ti.com/lit/ug/slau456/slau456.pdf

//The TAS2521 contains several pages of 8-bit registers, and each page can contain up to 128 registers.
//The register pages are divided up based on functional blocks for this device. Page 0 is the default home page after RST. 
//Page control is done by writing a new page value into register 0 of the current page.
//Pages legal: 0, 1, 44-52, 62-70, and 152-169
//All registers are 8 bits in width, with D7 referring to the most-significant bit of each register, 
//and D0 referring to the least-significant b

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

#include <iostream>
//#include "Arduino.h"
#include "wire.h"
#include <stdint.h>

namespace TAS2521{

#define TAS2521_DEBUG

enum CODEC_CLKIN_t{
  CODEC_CLKIN_MCLK	= 0x00,
  CODEC_CLKIN_BCLK	= 0x01,
  CODEC_CLKIN_GPIO	= 0x02,
  CODEC_CLKIN_PLL	= 0x03,
};

enum PLL_CLKIN_t{
  PLL_CLKIN_MCLK	= 0x00,
  PLL_CLKIN_BCLK	= 0x01,
  PLL_CLKIN_GPIO	= 0x02,
  PLL_CLKIN_DIN		= 0x03,
};

enum PLL_RANGE_t{
  PLL_RANGE_LOW		= 0x00,
  PLL_RANGE_HIGH	= 0x01,
};

enum PLL_DIV_P_t{
  PLL_DIV_1	=0x01,
  PLL_DIV_2	=0x02,
  PLL_DIV_3	=0x03,
  PLL_DIV_4	=0x04,
  PLL_DIV_5	=0x05,
  PLL_DIV_6	=0x06,
  PLL_DIV_7	=0x07,
  PLL_DIV_8	=0x00,
};

enum PLL_MULT_R_t{
  PLL_MULT_1	=0x01,
  PLL_MULT_2	=0x02,
  PLL_MULT_3	=0x03,
  PLL_MULT_4	=0x04,
};

enum POWER_STATE_t{
  POWER_DOWN	=0x00,
  POWER_UP	=0x01,
};

enum CDIV_CLKIN_t{
  CDIV_CLKIN_MCLK	= 0x00,
  CDIV_CLKIN_BCLK	= 0x01,
  CDIV_CLKIN_DIN	= 0x02,
  CDIV_CLKIN_PLL_CLK	= 0x03,
  CDIV_CLKIN_DAC_CLK	= 0x04,
  CDIV_CLKIN_DAC_MOD_CLK= 0x05,
};

enum INTERFACE_t{
  I2S	= 0x00,
  DSP	= 0x01,
  RJF	= 0x02,
  LJF	= 0x03,
};

enum WORD_LENGTH_t{
  BIT16	= 0x00,
  BIT20	= 0x01,
  BIT24	= 0x02,
  BIT32	= 0x03,
};

enum DIRECTION_T{
  IN	= 0x00,
  OUT	= 0x01,
};

enum BDIV_CLKIN_t{
  DAC_CLK = 0,
  DAC_MOD_CLK = 1
};

enum BIT_POLARITY_t{
  DEFAULT = 0,
  INVERTED = 1
};

enum SEC_CLK_MUX_t{
  CLK_MUX_GPIO = 0,
  CLK_MUX_SCLK = 1,
  CLK_MUX_MISO = 2,
  CLK_MUX_DOUT = 3
};

enum DATA_IN_MUX_t{
  DATA_IN_MUX_GPIO = 0,
  DATA_IN_MUX_SCLK = 1,
};

enum DOUT_MUX_CTRL_t{
  DOUT_DISABLED = 0,
  DOUT_PRIM_DOUT = 1,
  DOUT_GP_OUT = 2,
  DOUT_CLKOUT = 3,
  DOUT_INT1 = 4,
  DOUT_INT2 = 5,
  DOUT_SEC_BCLK = 6,
  DOUT_SEC_WCLK = 7,
};

enum DIN_FUNC_CTRL_t{
  DIN_DISABLED = 0,
  DIN_PRIM_DATA_IN = 1,
  DIN_GP_CLK_IN = 1,
  DIN_GP_IN = 2,
};

enum MISO_FUNC_CTRL_t{
  MISO_DISABLED = 0,
  MISO_SPI_DOUT =1, //MISO is used for data output in SPI interface, is disabled for I2C interface
  MISO_GPIO = 2, //MISO is General Purpose Output
  MISO_CLKOUT = 3, //MISO is CLKOUT output
  MISO_INT1_OUT = 4, //MISO is INT1 output
  MISO_INT2_OUT = 5, //MISO is INT2 output
  MISO_SEC_DATA_OUT = 8, //MISO is Secondary Data Output for Audio Interface
  MISO_SEC_BIT_CLK = 9, //MISO is Secondary Bit Clock for Audio Interface
  MISO_SEC_WRD_CLK = 10, //MISO is Secondary Word Clock for Audio Interface
};

enum SCLK_FUNC_CTRL_t{
  SCLK_DISABLED = 0,//SCLK pin is disabled
  SCLK_SPI_CLK = 1, //SCLK pin is enabled for SPI clock in SPI Interface mode or when in I2C Interface
		    //enabled for Secondary Data Input or Secondary Bit Clock Input or Secondary Word Clock.
  SCLK_GPIO = 2, // SCLK is enabled as General Purpose Input
};

enum DAC_INST_t{
  MINIDSP = 0,
  PRB_P1 = 1,
  PRB_P2 = 2,
  PRB_P3 = 3,
};

enum DAC_DATA_PATH_t{
  DATA_DISABLED = 0,
  DATA_FROM_LEFT = 1, //01: DAC data is picked from Left Channel Audio Interface Data
  DATA_FROM_RIGHT = 2, //10: DAC data is picked from Right Channel Audio Interface Data
  DATA_FROM_MONO = 3, //11: DAC data is picked from Mono Mix of Left and Right Channel Audio Interface Data
};

enum VOLUME_SOFT_STEP_t{
  SOFT_STEP1 = 0, //00: Soft-Stepping is 1 step per 1 DAC Word Clock
  SOFT_STEP2 = 1, //01: Soft-Stepping is 1 step per 2 DAC Word Clocks
  SOFT_STEP_DISABLE = 2, //10: Soft-Stepping is disabled
};

enum DAC_AUTO_MUTE_t{
  AUTO_MUTE_DISABLED = 0,
  AUTO_MUTE_100DC = 1,//001: DAC is auto muted if input data is DC for more than 100 consecutive inputs
  AUTO_MUTE_200DC = 2,//010: DAC is auto muted if input data is DC for more than 200 consecutive inputs
  AUTO_MUTE_400DC0 = 3,//11: DAC is auto muted if input data is DC for more than 400 consecutive inputs
  AUTO_MUTE_800DC = 4,//100: DAC is auto muted if input data is DC for more than 800 consecutive inputs
  AUTO_MUTE_1600DC = 5,//101: DAC is auto muted if input data is DC for more than 1600 consecutive inputs
  AUTO_MUTE_3200DC = 6,//110: DAC is auto muted if input data is DC for more than 3200 consecutive inputs
  AUTO_MUTE_6400DC = 7,//111: DAC is auto muted if input data is DC for more than 6400 consecutive inputs
};

enum AVDD_LDO_CTRL_t{
  LDO_1_8V = 0,
  LDO_1_6V = 1,
  LDO_1_7V = 2,
  LDO_1_5V = 3,
};

enum DAC_MODE_t{
  LOW_POWER = 0,
  HIGH_PERFORMANCE = 1
};

enum CHIP_COMMON_MODE_t{
  COMMON_MODE_0_9V = 0,
  COMMON_MODE_0_75V = 1,
};

enum HEADPHONE_DRIVE_ABILITY_t{
  FULL_FRIVE = 0,
  HALF_DRIVE = 1,
};

enum OVER_CURRENT_DETECTION_DEBOUNCE_t{
  NO_DEBOUNCE = 0, //000: No debounce is used for Over Current detection
  DEBOUNCE_8MS = 1, //001: Over Current detection is debounced by 8ms
  DEBOUNCE_16MS = 2, //010: Over Current detection is debounce by 16ms
  DEBOUNCE_32MS = 3, //011: Over Current detection is debounced by 32ms
  DEBOUNCE_64MS = 4, //100: Over Current detection is debounced by 64ms
  DEBOUNCE_128MS = 5, //101: Over Current detection is debounced by 128ms
  DEBOUNCE_256MS = 6, //110: Over Current detection is debounced by 256ms
  DEBOUNCE_512MS = 7, //111: Over Current detection is debounced by 512ms
};

enum OVER_CURRENT_CONDITION_t{
  LIMIT_OUT_CURRENT = 0, //Output current will be limited if over current condition is detected
  POWER_DOWN_DRIVER = 1 //Output driver will be powered down if over current condition is detected
};

enum ANALOG_ROUTING_t{
  NO_ANALOG_ROUTING = 0, //0000: No analog routing to SPK driver and HP driver
  AINR_TO_MIXER_P = 4, //0100: AINR routed to Mixer P
  AIN_LR_TO_SPK = 6, //0110: AINL/R differential routed to SPK driver through Mixer A and Mixer B
  AINL_TO_MIXA = 8, //1000: AINL routed to Mixer A
  AIN_LR_TO_SPK_ = 9, //1001: AINL/R differential routed to SPK driver through Mixer A and Mixer B
  AINL_AINR_TO_MIXA_HP = 12, //1100: AINL and AINR routed to Mixer A to HP driver
};

enum SOFT_ROUTING_STEP_TIME_t{
  STEP_TIME_0MS = 0, //00: Soft-routing step time = 0ms
  STEP_TIME_50MS = 1, //01: Soft-routing step time = 50ms
  STEP_TIME_100MS = 2, //10: Soft-routing step time = 100ms
  STEP_TIME_200MS = 3, //11: Soft-routing step time = 200ms
};

enum HP_AMP_SLOW_POWERUP_TIME_t{
  SLOW_POWERUP_DISABLED = 0, //0000: Slow power up of headphone amp's is disabled
  SLOW_POWERUP_0_5T = 1, //0001: Headphone amps power up slowly in 0.5 time constants
  SLOW_POWERUP_0_625T = 2, //0010: Headphone amps power up slowly in 0.625 time constants
  SLOW_POWERUP_0_725T = 3, //0011; Headphone amps power up slowly in 0.725 time constants
  SLOW_POWERUP_0_875T= 4, //0100: Headphone amps power up slowly in 0.875 time constants
  SLOW_POWERUP_1T= 5, //0101: Headphone amps power up slowly in 1.0 time constants
  SLOW_POWERUP_2T= 6, //0110: Headphone amps power up slowly in 2.0 time constants
  SLOW_POWERUP_3T= 7, //0111: Headphone amps power up slowly in 3.0 time constants
  SLOW_POWERUP_4T= 8, //1000: Headphone amps power up slowly in 4.0 time constants
  SLOW_POWERUP_5T= 9, //1001: Headphone amps power up slowly in 5.0 time constants
  SLOW_POWERUP_6T= 10, //1010: Headphone amps power up slowly in 6.0 time constants
  SLOW_POWERUP_7T= 11, //1011: Headphone amps power up slowly in 7.0 time constants
  SLOW_POWERUP_8T= 12, //1100: Headphone amps power up slowly in 8.0 time constants
  SLOW_POWERUP_16T= 13, //1101: Headphone amps power up slowly in 16.0 time constants ( do not use for Rchg=25K)
  SLOW_POWERUP_32T= 14, //1110: Headphone amps power up slowly in 24.0 time constants (do not use for Rchg=25K)
  SLOW_POWERUP_64T= 15, //1111: Headphone amps power up slowly in 32.0 time constants (do not use for Rchg=25K)
};

enum HP_AMP_POWERUP_RESISTOR_t{
  RES_25K = 0, //00: Headphone amps power up time is determined with 25K resistance
  RES_6K = 1, //01: Headphone amps power up time is determined with 6K resistance
  RES_2K = 2, //10: Headphone amps power up time is determined with 2K resistance
};

enum SPK_AMP_VOL_t{
  SPK_MUTED = 0, //000: SPK Driver is Muted (Default)
  SPK_VOL_6DB = 0, //001: SPK Driver Volume = 6 dB
  SPK_VOL_12DB = 0, //010: SPK Driver Volume = 12 dB
  SPK_VOL_18DB = 0, //011: SPK Driver Volume = 18 dB
  SPK_VOL_24DB = 0, //100: SPK Driver Volume = 24 dB
  SPK_VOL_32DB = 0, //101: SPK Driver Volume = 32 dB
};

enum REF_PWRUP_DELAY_t{
  REF_PWRUP_SLOW = 0 ,//000: Reference will power up slowly when analog blocks are powered up
  REF_PWRUP_40MS = 1 ,//001: Reference will power up in 40ms when analog blocks are powered up
  REF_PWRUP_80MS = 2 ,//010: Reference will power up in 80ms when analog blocks are powered up
  REF_PWRUP_120MS = 3 ,//011: Reference will power up in 120ms when analog blocks are powered up
  FORCE_REF_PWRUP_SLOW = 4 ,//100: Force power up of reference. Power up will be slow
  FORCE_REF_PWRUP_40MS = 5 ,//101: Force power up of reference. Power up time will be 40ms
  FORCE_REF_PWRUP_80MS = 6 ,//110: Force power up of reference. Power up time will be 80ms
  FORCE_REF_PWRUP_120MS = 7 ,//111: Force power up of reference. Power up time will be 120ms
};

//--------------------------- default configuration------------------------------
//Page 0 / Register 4: Clock Setting Register 1, Multiplexers - 0x00 / 0x04
const PLL_RANGE_t 	DEFAULT_PLL_RANGE = PLL_RANGE_LOW;
const PLL_CLKIN_t 	DEFAULT_PLL_CLKIN = PLL_CLKIN_MCLK;
const CODEC_CLKIN_t 	DEFAULT_CODEC_CLKIN = CODEC_CLKIN_MCLK;
//Page 0 / Register 5: Clock Setting Register 2, PLL P and R Values - 0x00 / 0x05
const POWER_STATE_t	DEFAULT_PLL_ON = POWER_DOWN;
const PLL_DIV_P_t	DEFAULT_PLL_DIV_P = PLL_DIV_1;
const PLL_MULT_R_t	DEFAULT_PLL_MULT_R = PLL_MULT_1;
//Page 0 / Register 6: Clock Setting Register 3, PLL J Values - 0x00 / 0x06
const uint8_t 		DEFAULT_PLL_DIV_J = 4;
//Page 0 / Register 7: Clock Setting Register 4, PLL D Values (MSB) - 0x00 / 0x07
//Page 0 / Register 8: Clock Setting Register 5, PLL D Values (LSB) - 0x00 / 0x08
const uint16_t		DEFAULT_PLL_DIV_D = 0;
//Page 0 / Register 11: Clock Setting Register 6, NDAC Values - 0x00 / 0x0B
const uint8_t		DEFAULT_NDAC_VALUE = 1;
const POWER_STATE_t	DEFAULT_NDAC_POWER = POWER_DOWN;
//Page 0 / Register 12: Clock Setting Register 7, MDAC Values - 0x00 / 0x0C
const uint8_t		DEFAULT_MDAC_VALUE = 1;
const POWER_STATE_t	DEFAULT_MDAC_POWER = POWER_DOWN;
//Page 0 / Register 14: DAC OSR Setting Register 2, LSB Value - 0x00 / 0x0E
//Page 0 / Register 15: miniDSP_D Instruction Control Register 1 - 0x00 / 0x0F
const uint16_t		DEFAULT_DAC_OSR = 128;
//Page 0 / Register 16: miniDSP_D Instruction Control Register 2 - 0x00 / 0x10
const uint16_t		DEFAULT_MINIDSP_IDAC = 512;
//Page 0 / Register 17: miniDSP_D Interpolation Factor Setting Register - 0x00 / 0x11
const uint8_t		DEFAULT_MINIDSP_INTERP = 8;
//Page 0 / Registers 25: Clock Setting Register 10, Multiplexers - 0x00 / 0x19
const CDIV_CLKIN_t	DEFAULT_CDIV_CLKIN = CDIV_CLKIN_MCLK;
//Page 0 / Registers 26: Clock Setting Register 11, CLKOUT M divider value - 0x00 / 0x1A
const uint8_t		DEFAULT_CLKOUT_MDIV_VALUE = 1;
const POWER_STATE_t	DEFAULT_CLKOUT_MDIV_POWER = POWER_DOWN;
//Page 0 / Register 27: Audio Interface Setting Register 1 - 0x00 / 0x1B
const INTERFACE_t	DEFAULT_AUDIO_INTERFACE = I2S;
const WORD_LENGTH_t	DEFAULT_WORD_LENGTH = BIT16;
const DIRECTION_T	DEFAULT_BCLK_DIR = IN;
const DIRECTION_T	DEFAULT_WCLK_DIR = IN;
//Page 0 / Register 28: Audio Interface Setting Register 2, Data offset setting - 0x00 / 0x1C
const uint8_t		DEFAULT_DATA_OFFSET = 0;
//Page 0 / Register 29: Audio Interface Setting Register 3 - 0x00 / 0x1D
const BIT_POLARITY_t	DEFAULT_CLK_POLARITY = DEFAULT;
const POWER_STATE_t	DEFAULT_PRIM_WCLK_BCLK_POWER = POWER_UP;
const BDIV_CLKIN_t	DEFAULT_BDIV_CLKIN = DAC_CLK;
//Page 0 / Register 30: Clock Setting Register 12, BCLK N Divider- 0x00 / 0x1E
const POWER_STATE_t	DEFAULT_BCLK_NDIV_POWER	= POWER_DOWN;
const uint8_t		DEFAULT_BCLK_NDIV_VALUE = 1;
//Page 0 / Register 31: Audio Interface Setting Register 4, Secondary Audio Interface - 0x00 / 0x1F
const SEC_CLK_MUX_t	DEFAULT_SEC_BIT_CLK_MUX	= CLK_MUX_GPIO;
const SEC_CLK_MUX_t	DEFAULT_SEC_WRD_CLK_MUX	= CLK_MUX_GPIO;
const DATA_IN_MUX_t	DEFAULT_SEC_DATA_IN_MUX = DATA_IN_MUX_GPIO;
//Page 0 / Register 32: Audio Interface Setting Register 5 - 0x00 / 0x20
const bool		DEFAULT_BIT_CLK_CTRL = 0;
const bool		DEFAULT_WRD_CLK_CTRL = 0;
const bool		DEFAULT_DATA_IN_CTRL = 0;
//Page 0 / Register 33: Audio Interface Setting Register 6 - 0x00 / 0x21
const bool		DEFAULT_BCLK_OUT_CTRL = 0;
const bool		DEFAULT_SEC_BIT_CLK_OUT_CTRL = 0;
const uint8_t		DEFAULT_WCLK_OUT_CTRL = 0;
const uint8_t		DEFAULT_SEC_WRD_CLK_OUT_CTRL = 0;
const bool 		DEFAULT_PRIM_DATA_OUT_CTRL = 0;
const bool 		DEFAULT_SEC_DATA_OUT_CTRL = 0;
//Page 0 / Register 34: Digital Interface Misc. Setting Register - 0x00 / 0x22
const bool		DEFAULT_I2C_CALL_ADDR_CONF = 0;
// Page 0 / Register 48: INT1 Control Register - 0x00 / 0x30
const bool 		DEFAULT_OVER_CURRENT_INT1_EN = false;
const bool 		DEFAULT_MINIDSP_OVERFLOW_INT1_EN = false;
const bool 		DEFAULT_INT1_PULSE_CTRL = false;
//Page 0 / Register 49: INT2 Interrupt Control Register - 0x00 / 0x31
const bool 		DEFAULT_OVER_CURRENT_INT2_EN = false;
const bool 		DEFAULT_MINIDSP_OVERFLOW_INT2_EN = false;
const bool 		DEFAULT_INT2_PULSE_CTRL = false;
//Page 0 / Register 52: GPIO/DOUT Control Register - 0x00 / 0x34
const uint8_t 		DEFAULT_GPIO_CTRL = 0;
const bool		DEFAULT_GPIO_OUT_STATE = 0;
//Page 0 / Register 53: DOUT Function Control Register - 0x00 / 0x35
const bool		DEFAULT_DOUT_BUS_KEEPER_EN = false;
const DOUT_MUX_CTRL_t	DEFAULT_DOUT_MUX_CTRL = DOUT_PRIM_DOUT;
const bool		DEFAULT_DOUT_GPIO_VAL = false;
//Page 0 / Register 54: DIN Function Control Register - 0x00 / 0x36
const DIN_FUNC_CTRL_t	DEFAULT_DIN_FUNC = DIN_PRIM_DATA_IN;
//Page 0 / Register 55: MISO Function Control Register - 0x00 / 0x37
const MISO_FUNC_CTRL_t	DEFAULT_MISO_FUNC = MISO_SPI_DOUT;
const bool		DEFAULT_MISO_GPIO_VAL = false;
//Page 0 / Register 56: SCLK/DMDIN2 Function Control Register- 0x00 / 0x38
const SCLK_FUNC_CTRL_t	DEFAULT_SCLK_FUNC = SCLK_SPI_CLK;
//Page 0 / Register 60: DAC Instruction Set - 0x00 / 0x3C
const DAC_INST_t	DEFAULT_DAC_INSTRUCIOTN = PRB_P1;
//Page 0 / Register 62: miniDSP_D Configuration Register - 0x00 / 0x3E
const bool		DEFAULT_MINIDSP_AUX_BIT_A = false;
const bool		DEFAULT_MINIDSP_AUX_BIT_B = false;
const bool		DEFAULT_MINIDSP_RESET_INST_COUNTER = 0;
//Page 0 / Register 63: DAC Channel Setup Register 1 - 0x00 / 0x3F
const POWER_STATE_t	DEFAULT_DAC_CHAN_POWER = POWER_DOWN;
const DAC_DATA_PATH_t	DEFAULT_DAC_DATA_PATH = DATA_FROM_LEFT;
const VOLUME_SOFT_STEP_t DEFAULT_SOFT_STEP = SOFT_STEP1;
//Page 0 / Register 64: DAC Channel Setup Register 2 - 0x00 / 0x40
const DAC_AUTO_MUTE_t	DEFAULT_AUTO_MUTE = AUTO_MUTE_DISABLED;
const bool 		DEFAULT_AUTO_MUTE_EN = true;
//Page 0 / Register 65: DAC Channel Digital Volume Control Register - 0x00 / 0x41
const uint8_t		DEFAULT_DAC_CHAN_VOLUME = 0;

//Page 1 / Registers 1: REF, POR and LDO BGAP Control Register - 0x01 / 0x01
const POWER_STATE_t	DEFAULT_MAST_REF_CTRL = POWER_DOWN;
const bool		DEFAULT_POR_PWR_CTRL = false;
const POWER_STATE_t	DEFAULT_LDO_PWR_CTRL = POWER_DOWN;
//Page 1 / Register 2: LDO Control Register - 0x01 / 0x02
const AVDD_LDO_CTRL_t	DEFAULT_AVDD_LDO_CTRL = LDO_1_8V;
const bool		DEFAULT_PLL_HP_LVL_SHIFT_PWR = false;
//Page 1 / Register 3: Playback Configuration Register 1 - 0x01 / 0x03
const DAC_MODE_t	DEFAULT_DAC_MODE = LOW_POWER;
const uint8_t		DEFAULT_DAC_PTM_CTRL = 0;
//Page 1 / Register 8: DAC PGA Control Register- 0x01 / 0x08
const bool		DEFAULT_SOFT_STEPPING_ENABLE = false;
const bool		DEFAULT_SOF_STEPPING_MODE = false;
//Page 1 / Register 9: Output Drivers, AINL, AINR, Control Register - 0x01 / 0x09
const POWER_STATE_t	DEFAULT_HPL_PWR = POWER_DOWN;
const bool		DEFAULT_AINL_INPUT_EN = false;
const bool		DEFAULT_AINR_INPUT_EN = false;
//Page 1 / Register 10: Common Mode Control Register - 0x01 / 0x0A
const CHIP_COMMON_MODE_t DEFAULT_CHIP_COMMON_MODE = COMMON_MODE_0_9V;
const HEADPHONE_DRIVE_ABILITY_t DEFAULT_DRIVE_ABILITY = FULL_FRIVE;
//Page 1 / Register 11: HP Over Current Protection Configuration Register - 0x01 / 0x0B
const OVER_CURRENT_DETECTION_DEBOUNCE_t DEFAULT_OVER_CURRENT_DEBOUNCE = NO_DEBOUNCE;
const OVER_CURRENT_CONDITION_t	DEFAULT_OVER_CURRENT_COND = LIMIT_OUT_CURRENT;
//Page 1 / Register 12: HP Routing Selection Register - 0x01 / 0x0C
const ANALOG_ROUTING_t DEFAULT_ANALOG_ROUTING = NO_ANALOG_ROUTING;
const bool	DEFAULT_DAC_TO_HP_ROUTING = false;
const bool	DEFAULT_CONNECT_MIX_P_TO_HP_ATTN = false;
const bool	DEFAULT_ROUT_AINL_ATTN_TO_HP = false;
const bool	DEFAULT_ROUT_AINR_ATTN_TO_HP = false;
//Page 1 / Register 16: HP Driver Gain Setting Register - 0x01 / 0x10
const bool	DEFAULT_MUTE_HP	= false;
const uint8_t 	DEFAULT_HP_DRIVER_GAIN = 0;
//Page 1 / Registers 20: Headphone Driver Startup Control Register - 0x01 / 0x14
const SOFT_ROUTING_STEP_TIME_t	DEFAULT_SOFT_ROUT_STEP_TIME = STEP_TIME_0MS;
const HP_AMP_SLOW_POWERUP_TIME_t DEFAULT_HP_SLOW_PWRUP = SLOW_POWERUP_DISABLED;
const HP_AMP_POWERUP_RESISTOR_t  DEFAULT_HP_PWRUP_RES = RES_25K;
//Page 1 / Register 22: HP Volume Control Register - 0x01 / 0x16
const uint8_t	DEFAULT_HP_VOLUME = 0;
//Page 1 / Register 24: AINL Volume Control Register - 0x01 / 0x18
const bool 	DEFAULT_MIXER_P_M_FORCE_EN = false;
const uint8_t	DEFAULT_AINL_VOLUME = 0;
//Page 1 / Register 25: AINR Volume Control Register - 0x01 / 0x19
const uint8_t	DEFAULT_AINR_VOLUME = 0;
//Page 1 / Register 45: Speaker Amplifier Control 1 - 0x01 / 0x2D
const POWER_STATE_t	DEFAULT_SPK_OUT_POWER = POWER_DOWN;
//Page 1 / Register 46: Speaker Volume Control 1 - 0x01 / 0x2E
const uint8_t DEFAULT_SPK_VOLUME = 0;
//Page 1 / Register 48: Speaker Amplifier Volume Control 2 - 0x01 / 0x30
const SPK_AMP_VOL_t DEFAULT_SPK_AMP_VOL = SPK_MUTED;
//Page 1 / Register 122: Reference Power Up Delay - 0x01 / 0x7A
const REF_PWRUP_DELAY_t DEFAULT_REF_PWRUP_DELAY = REF_PWRUP_SLOW;
//Page 44 / Register 1: DAC Adaptive Filter Configuration Register - 0x2C / 0x01
const bool DEFAULT_DAC_ADAP_FILT_EN = false;
const bool DEFAULT_DAC_ADAP_FILT_BUF_SWITCH = false;//DAC Coefficient Buffers will not be switched at next frame boundary


//--------------------------register definition--------------------------------

//Page 0 / Register 4: Clock Setting Register 1, Multiplexers - 0x00 / 0x04
class P0R4_t{
    const static int Address = 4;
    PLL_RANGE_t pll_range:1;
    PLL_CLKIN_t pll_clk_in:2;
    CODEC_CLKIN_t codec_clk_in:2;
  public:
    P0R4_t(PLL_RANGE_t range=DEFAULT_PLL_RANGE,
	   PLL_CLKIN_t pll_in=DEFAULT_PLL_CLKIN,
	   CODEC_CLKIN_t codec_in=DEFAULT_CODEC_CLKIN):
	     pll_range(range),pll_clk_in(pll_in),codec_clk_in(codec_in){}

    P0R4_t(uint8_t reg){
      codec_clk_in = (CODEC_CLKIN_t)(reg&0x03);
      pll_clk_in  = (PLL_CLKIN_t)((reg>>2)&0x03);
      pll_range  = (PLL_RANGE_t)((reg>>6)&0x01);
    }

    uint8_t GetAddress(){return Address;}
    uint8_t toByte(){return codec_clk_in|(pll_clk_in<<2)|(pll_range<<5);}

    void setPLLRange(PLL_RANGE_t range){pll_range = range;}
    PLL_RANGE_t getPLLRange(){return pll_range;}

    void setPLLClk(PLL_CLKIN_t clk){pll_clk_in=clk;}
    PLL_CLKIN_t getPLLClk(){return pll_clk_in;}

    void setCodecClk(CODEC_CLKIN_t clk){codec_clk_in=clk;}
    CODEC_CLKIN_t getCodecClk(){return codec_clk_in;}
};

//Page 0 / Register 5: Clock Setting Register 2, PLL P and R Values - 0x00 / 0x05
class P0R5_t{
      const static int Address = 5;
      POWER_STATE_t pll_on:1;
      PLL_DIV_P_t pll_div_p:3;
      PLL_MULT_R_t pll_mult_r:4;
    public:
      P0R5_t(POWER_STATE_t pll_on_i = DEFAULT_PLL_ON,
	     PLL_DIV_P_t pll_div_p_i = DEFAULT_PLL_DIV_P,
	     PLL_MULT_R_t pll_mult_r_i = DEFAULT_PLL_MULT_R):
	       pll_on(pll_on_i),pll_div_p(pll_div_p_i),pll_mult_r(pll_mult_r_i){}

      P0R5_t(uint8_t reg){
	pll_mult_r = (PLL_MULT_R_t)(reg&0x0F);
	pll_div_p  = (PLL_DIV_P_t)((reg>>4)&0x07);
	pll_on	= (POWER_STATE_t)((reg>>7)&0x01);
      }
      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return pll_mult_r|(pll_div_p<<4)|(pll_on<<7);}

      void setPLLPower(POWER_STATE_t power){pll_on = power;}
      POWER_STATE_t getPLLPower(){return pll_on;}

      void setPLLDivP(PLL_DIV_P_t P){pll_div_p = P;}
      PLL_DIV_P_t getPLLDivP(){return pll_div_p;}

      void setPLLMultR(PLL_MULT_R_t R){pll_mult_r = R;}
      PLL_MULT_R_t getPLLMultR(){return pll_mult_r;}
};

//Page 0 / Register 6: Clock Setting Register 3, PLL J Values - 0x00 / 0x06
class P0R6_t{
      const static int Address = 6;
      uint8_t pll_div_j:6;
    public:
      P0R6_t(uint8_t reg= DEFAULT_PLL_DIV_J):pll_div_j(reg){
	if(reg<4) reg=4;
	if(reg>63) reg=63;
	pll_div_j  = reg;
      }
      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return pll_div_j;}

      void setPLLDivJ(uint8_t j){pll_div_j = j;}
      uint8_t getPLLDivJ(){return pll_div_j;}
};

//Page 0 / Register 7: Clock Setting Register 4, PLL D Values (MSB) - 0x00 / 0x07
// Note: This register will be updated only when the Page-0, Reg-8 is written immediately after
//Page-0, Reg-7.
class P0R7_t{
      const static int Address = 7;
      uint8_t pll_div_d_msb:6;
    public:
      P0R7_t(uint16_t pll_div_d= DEFAULT_PLL_DIV_D){
	if(pll_div_d>9999) pll_div_d=9999;
	pll_div_d_msb  = (pll_div_d&0x3F00)>>8;
      }

      P0R7_t(uint8_t reg){
      	pll_div_d_msb  = (reg&0x3F);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return pll_div_d_msb;}

      void setPLLDivD_MSB(uint8_t d){pll_div_d_msb = d&0x3F;}
      uint8_t getPLLDivD_MSB(){return pll_div_d_msb;}
};

//Page 0 / Register 8: Clock Setting Register 5, PLL D Values (LSB) - 0x00 / 0x08
// Note: Page-0, Reg-8 should be written immediately after Page-0, Reg-7.
class P0R8_t{
      const static int Address = 8;
      uint8_t pll_div_d_lsb;
    public:
      P0R8_t(uint16_t pll_div_d= DEFAULT_PLL_DIV_D){
	if(pll_div_d>9999) pll_div_d=9999;
	pll_div_d_lsb  = (pll_div_d&0x00FF);
      }

      P0R8_t(uint8_t reg){
	pll_div_d_lsb  = (reg);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return pll_div_d_lsb;}

      void setPLLDivD_LSB(uint8_t d){pll_div_d_lsb = d;}
      uint8_t getPLLDivD_LSB(){return pll_div_d_lsb;}
};

//Page 0 / Registers 9–10: Reserved - 0x00 / 0x09-0x0A

//Page 0 / Register 11: Clock Setting Register 6, NDAC Values - 0x00 / 0x0B
class P0R11_t{
      const static int Address = 11;
      POWER_STATE_t ndac_power:1;
      uint8_t ndac:7;
    public:
      P0R11_t(uint8_t ndac_i= DEFAULT_NDAC_VALUE,POWER_STATE_t ndac_power_i = DEFAULT_NDAC_POWER){
	ndac  = (ndac_i&0x7F);
	ndac_power = ndac_power_i;
      }

      P0R11_t(uint8_t reg){
      	ndac  = (reg&0x7F);
      	ndac_power = (POWER_STATE_t)((reg&0x80)>>7);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return ndac|(ndac_power<<7);}

      void setNDACPower(POWER_STATE_t p){ndac_power = p;}
      POWER_STATE_t getNDACPower(){return ndac_power;}

      void setNDACValue(uint8_t p){ndac = p&0x7F;}
      uint8_t getNDACValue(){return ndac;}
};

//Page 0 / Register 12: Clock Setting Register 7, MDAC Values - 0x00 / 0x0C
class P0R12_t{
      const static int Address = 12;
      POWER_STATE_t mdac_power:1;
      uint8_t mdac:7;
    public:
      P0R12_t(uint8_t mdac_i= DEFAULT_MDAC_VALUE,POWER_STATE_t mdac_power_i = DEFAULT_MDAC_POWER){
	mdac  = (mdac_i&0x7F);
	mdac_power = mdac_power_i;
      }

      P0R12_t(uint8_t reg){
	mdac  = (reg&0x7F);
	mdac_power = (POWER_STATE_t)((reg&0x80)>>7);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return mdac|(mdac_power<<7);}

      void setMDACPower(POWER_STATE_t p){mdac_power = p;}
      POWER_STATE_t getMDACPower(){return mdac_power;}

      void setMDACValue(uint8_t p){mdac = p&0x7F;}
      uint8_t getMDACValue(){return mdac;}
};
//Page 0 / Register 13: DAC OSR Setting Register 1, MSB Value - 0x00 / 0x0D
//Note: This register is updated when Page-0, Reg-14 is written to immediately after Page-0,Reg-13.
class P0R13_t{
      const static int Address = 13;
      uint8_t dac_osr_msb:2;
    public:
      P0R13_t(uint16_t osr= DEFAULT_DAC_OSR){
	if(osr>1023) osr=1023;
	dac_osr_msb  = (osr&0x0300)>>8;
      }

      P0R13_t(uint8_t reg){
	dac_osr_msb  = (reg&0x03);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return dac_osr_msb;}

      void setPLLDivD_MSB(uint8_t d){dac_osr_msb = d&0x03;}
      uint8_t getPLLDivD_MSB(){return dac_osr_msb;}
};

//Page 0 / Register 14: DAC OSR Setting Register 2, LSB Value - 0x00 / 0x0E
class P0R14_t{
      const static int Address = 14;
      uint8_t dac_osr_lsb;
    public:
      P0R14_t(uint16_t osr= DEFAULT_DAC_OSR){
	if(osr>1023) osr=1023;
	dac_osr_lsb  = (osr&0x00FF);
      }

      P0R14_t(uint8_t reg){
	dac_osr_lsb  = (reg);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return dac_osr_lsb;}

      void setPLLDivD_MSB(uint8_t d){dac_osr_lsb = d;}
      uint8_t getPLLDivD_MSB(){return dac_osr_lsb;}
};

//Note: This register should be written immediately after Page-0, Reg-13.
//Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 )
//Note: Page-0, Reg-15 takes effect after programming Page-0, Reg-16 in the immediate next
//control command.
//Page 0 / Register 15: miniDSP_D Instruction Control Register 1 - 0x00 / 0x0F
class P0R15_t{
      const static int Address = 15;
      uint8_t IDAC_msb:7;
    public:
      P0R15_t(uint16_t idac_i= DEFAULT_MINIDSP_IDAC){
	IDAC_msb  = (idac_i&0x7F00)>>8;
      }

      P0R15_t(uint8_t reg){
	IDAC_msb  = (reg&0x7F);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return IDAC_msb;}

      void setIDACValue(uint8_t p){IDAC_msb = p&0x7F;}
      uint8_t getIDACValue(){return IDAC_msb;}
};

//Page 0 / Register 16: miniDSP_D Instruction Control Register 2 - 0x00 / 0x10
//Note: IDAC should be a integral multiple of INTERP ( Page-0, Reg-17, D3-D0 )
//Note: Page-0, Reg-16 should be programmed immediately after Page-0, Reg-15.
class P0R16_t{
      const static int Address = 16;
      uint8_t IDAC_lsb;
    public:
      P0R16_t(uint16_t idac_i= DEFAULT_MINIDSP_IDAC){
	IDAC_lsb  = (idac_i&0x00FF);
      }

      P0R16_t(uint8_t reg){
	IDAC_lsb  = (reg);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return IDAC_lsb;}

      void setIDACValue(uint8_t p){IDAC_lsb = p;}
      uint8_t getIDACValue(){return IDAC_lsb;}
};
//Page 0 / Register 17: miniDSP_D Interpolation Factor Setting Register - 0x00 / 0x11
class P0R17_t{
      const static int Address = 17;
      uint8_t interp:4;
    public:
      P0R17_t(uint8_t interp_i= DEFAULT_MINIDSP_INTERP){
	interp  = (interp_i&0x0F);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return interp;}

      void setIDACValue(uint8_t p){interp = p&0x0F;}
      uint8_t getIDACValue(){return interp;}
};
//Page 0 / Registers 18 - 24: Reserved Register - 0x00 / 0x12

//Page 0 / Registers 25: Clock Setting Register 10, Multiplexers - 0x00 / 0x19
class P0R25_t{
    const static int Address = 25;
    CDIV_CLKIN_t cdiv_clk_in:3;
  public:
    P0R25_t(CDIV_CLKIN_t cdiv_in=DEFAULT_CDIV_CLKIN): cdiv_clk_in(cdiv_in){}

    P0R25_t(uint8_t reg){
      cdiv_clk_in = (CDIV_CLKIN_t)(reg&0x03);
    }

    uint8_t GetAddress(){return Address;}
    uint8_t toByte(){return cdiv_clk_in;}

    void setCDIVClk(CDIV_CLKIN_t clk){cdiv_clk_in=clk;}
    CDIV_CLKIN_t getCDIVClk(){return cdiv_clk_in;}
};

//Page 0 / Registers 26: Clock Setting Register 11, CLKOUT M divider value - 0x00 / 0x1A
class P0R26_t{
      const static int Address = 26;
      POWER_STATE_t clkout_mdiv_power:1;
      uint8_t clkout_mdiv:7;
    public:
      P0R26_t(uint8_t clkout_i= DEFAULT_CLKOUT_MDIV_VALUE,
	      POWER_STATE_t clkout_mdiv_power_i = DEFAULT_CLKOUT_MDIV_POWER){
	clkout_mdiv  = (clkout_i&0x7F);
	clkout_mdiv_power = clkout_mdiv_power_i;
      }

      P0R26_t(uint8_t reg){
	clkout_mdiv  = (reg&0x7F);
	clkout_mdiv_power = (POWER_STATE_t)((reg&0x80)>>7);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return clkout_mdiv|(clkout_mdiv_power<<7);}

      void setCLKoutMdivPower(POWER_STATE_t p){clkout_mdiv_power = p;}
      POWER_STATE_t getCLKoutMdivPower(){return clkout_mdiv_power;}

      void setCLKoutMdivValue(uint8_t p){clkout_mdiv = p&0x7F;}
      uint8_t getCLKoutMdivValue(){return clkout_mdiv;}
};
//Page 0 / Register 27: Audio Interface Setting Register 1 - 0x00 / 0x1B
class P0R27_t{
      const static int Address = 27;

      INTERFACE_t interface:2;
      WORD_LENGTH_t wordlength:2;
      DIRECTION_T bclk_dir:1;
      DIRECTION_T wclk_dir:1;

    public:
      P0R27_t(INTERFACE_t interface_i = DEFAULT_AUDIO_INTERFACE,
	      WORD_LENGTH_t wordlength_i = DEFAULT_WORD_LENGTH,
	      DIRECTION_T bclk_dir_i = DEFAULT_BCLK_DIR,
	      DIRECTION_T wclk_dir_i = DEFAULT_WCLK_DIR):
		interface(interface_i),wordlength(wordlength_i),
		bclk_dir(bclk_dir_i),wclk_dir(wclk_dir_i){}

      P0R27_t(uint8_t reg){
	wclk_dir = (DIRECTION_T)((reg&0x04)>>2);
	bclk_dir = (DIRECTION_T)((reg&0x08)>>3);
	wordlength = (WORD_LENGTH_t)((reg&0x30)>>4);
	interface = (INTERFACE_t)((reg&0xC0)>>6);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (interface<<6)|(wordlength<<4)|(bclk_dir<<3)|(wclk_dir<<2);}

      void setInterface(INTERFACE_t p){interface = p;}
      INTERFACE_t getInterface(){return interface;}

      void setWordLength(WORD_LENGTH_t p){wordlength = p;}
      WORD_LENGTH_t getWordLength(){return wordlength;}

      void setBclkDir(DIRECTION_T p){bclk_dir = p;}
      DIRECTION_T getBclkDir(){return bclk_dir;}

      void setWclkDir(DIRECTION_T p){wclk_dir = p;}
      DIRECTION_T getWclkDir(){return wclk_dir;}
};
//Page 0 / Register 28: Audio Interface Setting Register 2, Data offset setting - 0x00 / 0x1C
class P0R28_t{
      const static int Address = 28;
      uint8_t offset;
    public:
      P0R28_t(uint8_t reg=DEFAULT_DATA_OFFSET):offset(reg){}

      uint8_t GetAddress(){return Address;}
      uint8_t toByte(){return offset;}

      void setOffset(uint8_t p){offset = p;}
      uint8_t getOffset(){return offset;}
};
//Page 0 / Register 29: Audio Interface Setting Register 3 - 0x00 / 0x1D
class P0R29_t{
      const static int Address = 29;

      BIT_POLARITY_t ClockPolarity:1;
      POWER_STATE_t bclk_wlck_power:1;
      BDIV_CLKIN_t bdiv_clkin:2;

    public:
      P0R29_t(BIT_POLARITY_t ClockPolarity_i = DEFAULT_CLK_POLARITY,
	      POWER_STATE_t bclk_wlck_power_i = DEFAULT_PRIM_WCLK_BCLK_POWER,
	      BDIV_CLKIN_t bdiv_clkin_i = DEFAULT_BDIV_CLKIN):
		ClockPolarity(ClockPolarity_i),
		bclk_wlck_power(bclk_wlck_power_i),
		bdiv_clkin(bdiv_clkin_i){}

      P0R29_t(uint8_t reg){
	ClockPolarity = (BIT_POLARITY_t)((reg&0x08)>>3);
	bclk_wlck_power = (POWER_STATE_t)((reg&0x04)>>2);
	bdiv_clkin = (BDIV_CLKIN_t)(reg&0x03);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (ClockPolarity<<3)|(bclk_wlck_power<<2)|(bdiv_clkin);}

      void setAudioBitClockPolarity(BIT_POLARITY_t p){ClockPolarity = p;}
      BIT_POLARITY_t getAudioBitClockPolarity(){return ClockPolarity;}

      void setPrimaryBCLK_WCLKPower(POWER_STATE_t p){bclk_wlck_power = p;}
      POWER_STATE_t getPrimaryBCLK_WCLKPower(){return bclk_wlck_power;}

      void setBDIVCLKINMultiplexer(BDIV_CLKIN_t p){bdiv_clkin = p;}
      BDIV_CLKIN_t getBDIVCLKINMultiplexer(){return bdiv_clkin;}
};
//Page 0 / Register 30: Clock Setting Register 12, BCLK N Divider- 0x00 / 0x1E
class P0R30_t{
      const static int Address = 30;
      POWER_STATE_t bclk_n_div_power:1;
      uint8_t bclk_n_div_value:7;

    public:
      P0R30_t(POWER_STATE_t bclk_n_div_power_i = DEFAULT_BCLK_NDIV_POWER,
	      uint8_t bclk_n_div_value_i = DEFAULT_BCLK_NDIV_VALUE):
		bclk_n_div_power(bclk_n_div_power_i),
		bclk_n_div_value(bclk_n_div_value_i){}

      P0R30_t(uint8_t reg){
	bclk_n_div_power = (POWER_STATE_t)((reg&0x80)>>7);
	bclk_n_div_value = (uint8_t)((reg&0x7F));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (bclk_n_div_power<<7)|bclk_n_div_value;}

      void seBclkNDIVValue(uint8_t p){bclk_n_div_value = p;}
      uint8_t geBclkNDIVValue(){return bclk_n_div_value;}

      void seBclkNDIVPower(POWER_STATE_t p){bclk_n_div_power = p;}
      POWER_STATE_t geBclkNDIVPower(){return bclk_n_div_power;}
};
//Page 0 / Register 31: Audio Interface Setting Register 4, Secondary Audio Interface - 0x00 / 0x1F
class P0R31_t{
      const static int Address = 31;
      SEC_CLK_MUX_t sec_bit_clk_mux:2;
      SEC_CLK_MUX_t sec_wrd_clk_mux:2;
      DATA_IN_MUX_t sec_data_in_mux:1;
    public:
      P0R31_t(SEC_CLK_MUX_t sec_bit_clk_mux_i = DEFAULT_SEC_BIT_CLK_MUX,
	      SEC_CLK_MUX_t sec_wrd_clk_mux_i = DEFAULT_SEC_WRD_CLK_MUX,
	      DATA_IN_MUX_t sec_data_in_mux_i = DEFAULT_SEC_DATA_IN_MUX):
		sec_bit_clk_mux(sec_bit_clk_mux_i),
		sec_wrd_clk_mux(sec_wrd_clk_mux_i),
		sec_data_in_mux(sec_data_in_mux_i){}

      P0R31_t(uint8_t reg){
	sec_bit_clk_mux = (SEC_CLK_MUX_t)((reg&0x60)>>5);
	sec_wrd_clk_mux = (SEC_CLK_MUX_t)((reg&0x18)>>3);
	sec_data_in_mux = (DATA_IN_MUX_t)(reg&0x01);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (sec_bit_clk_mux<<5)|(sec_wrd_clk_mux<<3)|sec_data_in_mux;}

      void setBitClkMux(SEC_CLK_MUX_t p){sec_bit_clk_mux = p;}
      SEC_CLK_MUX_t geBitClkMux(){return sec_bit_clk_mux;}

      void setWrdClkMux(SEC_CLK_MUX_t p){sec_wrd_clk_mux = p;}
      SEC_CLK_MUX_t geWrdClkMux(){return sec_wrd_clk_mux;}

      void setDataInMux(DATA_IN_MUX_t p){sec_data_in_mux = p;}
      DATA_IN_MUX_t getDataInMux(){return sec_data_in_mux;}
};
//Page 0 / Register 32: Audio Interface Setting Register 5 - 0x00 / 0x20
class P0R32_t{
      const static int Address = 32;
      bool bit_clk_ctrl;
      bool wrd_clk_ctrl;
      bool data_in_ctrl;
    public:
      P0R32_t(bool bit_clk_ctrl_i = DEFAULT_BIT_CLK_CTRL,
	      bool wrd_clk_ctrl_i = DEFAULT_WRD_CLK_CTRL,
	      bool data_in_ctrl_i = DEFAULT_DATA_IN_CTRL):
		bit_clk_ctrl(bit_clk_ctrl_i),
		wrd_clk_ctrl(wrd_clk_ctrl_i),
		data_in_ctrl(data_in_ctrl_i){}

      P0R32_t(uint8_t reg){
	bit_clk_ctrl = ((reg&0x08)>>3);
	wrd_clk_ctrl = ((reg&0x04)>>2);
	data_in_ctrl = (reg&0x01);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (bit_clk_ctrl<<3)|(wrd_clk_ctrl<<2)|data_in_ctrl;}

      void setBitClkCtrl(bool p){bit_clk_ctrl = p;}
      bool geBitClkCtrl(){return bit_clk_ctrl;}

      void setWrdClkCtrl(bool p){wrd_clk_ctrl = p;}
      bool geWrdClkCtrl(){return wrd_clk_ctrl;}

      void setDataInCtrl(bool p){data_in_ctrl = p;}
      bool getDataInCtrl(){return data_in_ctrl;}
};
//Page 0 / Register 33: Audio Interface Setting Register 6 - 0x00 / 0x21
class P0R33_t{
      const static int Address = 33;

      bool bclk_out_ctrl;
      bool sec_bit_clk_out_ctrl;
      uint8_t wclk_out_ctrl:2;
      uint8_t sec_wrd_clk_out_ctrl:2;
      bool pri_data_out_ctrl;
      bool sec_data_out_ctrl;

    public:
      P0R33_t(bool bclk_out_ctrl_i = DEFAULT_BCLK_OUT_CTRL,
	      bool sec_bit_clk_out_ctrl_i = DEFAULT_BIT_CLK_CTRL,
	      uint8_t wclk_out_ctrl_i = DEFAULT_WCLK_OUT_CTRL,
	      uint8_t sec_wrd_clk_out_ctrl_i = DEFAULT_SEC_WRD_CLK_OUT_CTRL,
	      bool pri_data_out_ctrl_i = DEFAULT_PRIM_DATA_OUT_CTRL,
	      bool sec_data_out_ctrl_i = DEFAULT_SEC_DATA_OUT_CTRL):
		bclk_out_ctrl(bclk_out_ctrl_i),
		sec_bit_clk_out_ctrl(sec_bit_clk_out_ctrl_i),
		wclk_out_ctrl(wclk_out_ctrl_i),
		sec_wrd_clk_out_ctrl(sec_wrd_clk_out_ctrl_i),
		pri_data_out_ctrl(pri_data_out_ctrl_i),
		sec_data_out_ctrl(sec_data_out_ctrl_i){}

      P0R33_t(uint8_t reg){
	bclk_out_ctrl = ((reg&0x80)>>7);
	sec_bit_clk_out_ctrl = ((reg&0x40)>>6);
	wclk_out_ctrl = ((reg&0x30)>>4);
	sec_wrd_clk_out_ctrl = ((reg&0x0C)>>2);
	pri_data_out_ctrl = ((reg&0x2)>>1);
	sec_data_out_ctrl = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (bclk_out_ctrl<<7)|(sec_bit_clk_out_ctrl<<6)|(wclk_out_ctrl<<4)|
			      (sec_wrd_clk_out_ctrl<<2)|(pri_data_out_ctrl<<1)|sec_data_out_ctrl;}

      void setBclkOutCtrl(bool p){bclk_out_ctrl = p;}
      bool getBclkOutCtrl(){return bclk_out_ctrl;}

      void setSecBitClkOutCtrl(bool p){sec_bit_clk_out_ctrl = p;}
      bool getSecBitClkOutCtrl(){return sec_bit_clk_out_ctrl;}

      void setWclkOutCtrl(uint8_t p){wclk_out_ctrl = p;}
      uint8_t getWclkOutCtrl(){return wclk_out_ctrl;}

      void setWrdClkOutCtrl(uint8_t p){sec_wrd_clk_out_ctrl = p;}
      uint8_t getWrdClkOutCtrl(){return sec_wrd_clk_out_ctrl;}

      void setPriDataOutCtrl(bool p){pri_data_out_ctrl = p;}
      bool getPriDataOutCtrl(){return pri_data_out_ctrl;}

      void setSecDataOutCtrl(bool p){sec_data_out_ctrl = p;}
      bool getSecDataOutCtrl(){return sec_data_out_ctrl;}
};

//Page 0 / Register 34: Digital Interface Misc. Setting Register - 0x00 / 0x22
class P0R34_t{
      const static int Address = 34;

      bool i2c_call_addr;

    public:
      P0R34_t(bool i2c_call_addr_i = DEFAULT_I2C_CALL_ADDR_CONF):i2c_call_addr(i2c_call_addr_i){}

      P0R34_t(uint8_t reg){ i2c_call_addr = ((reg&0x20)>>5);}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (i2c_call_addr<<5);}

      void setI2CCallAddr(bool p){i2c_call_addr = p;}
      bool getI2CCallAddr(){return i2c_call_addr;}
};

//Page 0 / Register 35 - 36 Reserved- 0x00 / 0x23 - 0x24

//Page 0 / Register 37: DAC Flag Register 1 - 0x00 / 0x25
class P0R37_t{
      const static int Address = 37;

      POWER_STATE_t dac_pwr;
      POWER_STATE_t hpout_pwr;
    public:
      P0R37_t(uint8_t reg){
	dac_pwr = (POWER_STATE_t)((reg&0x80)>>7);
	hpout_pwr = (POWER_STATE_t)((reg&0x20)>>5);}

      uint8_t GetAddress(){return Address;}

      POWER_STATE_t getDacPowerState(){return dac_pwr;}
      POWER_STATE_t getHpoutPowerState(){return hpout_pwr;}
};

//Page 0 / Register 38: DAC Flag Register 2- 0x00 / 0x26
class P0R38_t{
      const static int Address = 38;

      bool dac_pga_stat;
    public:
      P0R38_t(uint8_t reg){dac_pga_stat = ((reg&0x10)>>4);}

      uint8_t GetAddress(){return Address;}

      bool getPGAStat(){return dac_pga_stat;}
};

//Page 0 / Registers 42: Sticky Flag Register 1- 0x00 / 0x2A
class P0R42_t{
      const static int Address = 42;

      bool dac_overflow;
      bool shifter_overflow;
    public:
      P0R42_t(uint8_t reg){
	dac_overflow = ((reg&0x80)>>7);
	shifter_overflow = ((reg&0x20)>>5);}

      uint8_t GetAddress(){return Address;}

      bool getDacOverflow(){return dac_overflow;}
      bool getShifterOveflow(){return shifter_overflow;}
};

//Page 0 / Registers 43: Interrupt Flags Register 1 - 0x00 / 0x2B
class P0R43_t{
      const static int Address = 43;

      bool dac_overflow;
      bool shifter_overflow;
    public:
      P0R43_t(uint8_t reg){
	dac_overflow = ((reg&0x80)>>7);
	shifter_overflow = ((reg&0x20)>>5);}

      uint8_t GetAddress(){return Address;}

      bool getDacOverflow(){return dac_overflow;}
      bool getShifterOveflow(){return shifter_overflow;}
};

//Page 0 / Register 44: Sticky Flag Register 2 - 0x00 / 0x2C
class P0R44_t{
      const static int Address = 44;

      bool hpout_over_current;
      bool minidsp_std_int;
      bool minidsp_aux_int;
    public:
      P0R44_t(uint8_t reg){
	hpout_over_current = ((reg&0x80)>>7);
	minidsp_std_int = ((reg&0x02)>>1);
	minidsp_aux_int = ((reg&0x01));}

      uint8_t GetAddress(){return Address;}

      bool getHpoutOverCurrent(){return hpout_over_current;}
      bool getMinDspStdInt(){return minidsp_std_int;}
      bool getMinDspAuxInt(){return minidsp_aux_int;}
};
//Page 0 / Register 46: Interrupt Flag Register 2 - 0x00 / 0x2E
class P0R46_t{
      const static int Address = 46;

      bool hpout_over_current;
      bool minidsp_std_int;
      bool minidsp_aux_int;
    public:
      P0R46_t(uint8_t reg){
	hpout_over_current = ((reg&0x80)>>7);
	minidsp_std_int = ((reg&0x02)>>1);
	minidsp_aux_int = ((reg&0x01));}

      uint8_t GetAddress(){return Address;}

      bool getHpoutOverCurrent(){return hpout_over_current;}
      bool getMinDspStdInt(){return minidsp_std_int;}
      bool getMinDspAuxInt(){return minidsp_aux_int;}
};
//Page 0 / Register 48: INT1 Control Register - 0x00 / 0x30
class P0R48_t{
      const static int Address = 48;

      bool over_current_int_en;
      bool minidsp_overflow_int_en;
      bool int1_pulse_ctrl;

    public:
      P0R48_t(bool over_current_int_en_i = DEFAULT_OVER_CURRENT_INT1_EN,
	      bool minidsp_overflow_int_en_i = DEFAULT_MINIDSP_OVERFLOW_INT1_EN,
	      bool int1_pulse_ctrl_i = DEFAULT_INT1_PULSE_CTRL
	      ):over_current_int_en(over_current_int_en_i),
	      minidsp_overflow_int_en(minidsp_overflow_int_en_i),
	      int1_pulse_ctrl(int1_pulse_ctrl_i){}

      P0R48_t(uint8_t reg){
	over_current_int_en = ((reg&0x08)>>3);
	minidsp_overflow_int_en = ((reg&0x04)>>2);
	int1_pulse_ctrl = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (over_current_int_en<<3)|(minidsp_overflow_int_en<<2)|(int1_pulse_ctrl);}

      void setOverCurrentIntEn(bool p){over_current_int_en = p;}
      bool getOverCurrentIntEn(){return over_current_int_en;}

      void setOverflowIntEn(bool p){minidsp_overflow_int_en = p;}
      bool getOverflowIntEn(){return minidsp_overflow_int_en;}

      void setPulseCtrl(bool p){int1_pulse_ctrl = p;}
      bool getPulseCtrl(){return int1_pulse_ctrl;}
};

//Page 0 / Register 49: INT2 Interrupt Control Register - 0x00 / 0x31
class P0R49_t{
      const static int Address = 49;

      bool over_current_int_en;
      bool minidsp_overflow_int_en;
      bool int1_pulse_ctrl;

    public:
      P0R49_t(bool over_current_int_en_i = DEFAULT_OVER_CURRENT_INT2_EN,
	      bool minidsp_overflow_int_en_i = DEFAULT_MINIDSP_OVERFLOW_INT2_EN,
	      bool int1_pulse_ctrl_i = DEFAULT_INT2_PULSE_CTRL
	      ):over_current_int_en(over_current_int_en_i),
	      minidsp_overflow_int_en(minidsp_overflow_int_en_i),
	      int1_pulse_ctrl(int1_pulse_ctrl_i){}

      P0R49_t(uint8_t reg){
	over_current_int_en = ((reg&0x08)>>3);
	minidsp_overflow_int_en = ((reg&0x04)>>2);
	int1_pulse_ctrl = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (over_current_int_en<<3)|(minidsp_overflow_int_en<<2)|(int1_pulse_ctrl);}

      void setOverCurrentIntEn(bool p){over_current_int_en = p;}
      bool getOverCurrentIntEn(){return over_current_int_en;}

      void setOverflowIntEn(bool p){minidsp_overflow_int_en = p;}
      bool getOverflowIntEn(){return minidsp_overflow_int_en;}

      void setPulseCtrl(bool p){int1_pulse_ctrl = p;}
      bool getPulseCtrl(){return int1_pulse_ctrl;}
};

//Page 0 / Register 52: GPIO/DOUT Control Register - 0x00 / 0x34
class P0R52_t{
      const static int Address = 52;

      uint8_t gpio_ctrl;
      bool gpio_out_state;
      bool gpio_in_state;
    public:
      P0R52_t(uint8_t gpio_ctrl_i = DEFAULT_GPIO_CTRL,
	      bool gpio_state_i = DEFAULT_GPIO_OUT_STATE
	      ):gpio_ctrl(gpio_ctrl_i),
	      gpio_out_state(gpio_state_i),
	      gpio_in_state(false){}

      P0R52_t(uint8_t reg){
	gpio_ctrl = ((reg&0x3C)>>2);
	gpio_out_state = ((reg&0x01));
	gpio_in_state = ((reg&0x02)>>1);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (gpio_ctrl<<2)|(gpio_out_state);}

      void setGpioCtrl(uint8_t p){gpio_ctrl = p;}
      uint8_t getGpioCtrl(){return gpio_ctrl;}

      void setGpioOutState(bool p){gpio_out_state = p;}
      bool getGpioOutState(){return gpio_out_state;}

      bool getGpioInState(void){return gpio_in_state;}
};

//Page 0 / Register 53: DOUT Function Control Register - 0x00 / 0x35
class P0R53_t{
      const static int Address = 53;

      bool bus_keeper_en;
      DOUT_MUX_CTRL_t dout_mux_ctrl;
      bool dout_gpio_state;
    public:
      P0R53_t(bool bus_keeper_en_i = DEFAULT_DOUT_BUS_KEEPER_EN,
	      DOUT_MUX_CTRL_t dout_mux_ctrl_i = DEFAULT_DOUT_MUX_CTRL,
	      bool dout_gpio_state_i = DEFAULT_DOUT_GPIO_VAL
	      ):bus_keeper_en(bus_keeper_en_i),
	      dout_mux_ctrl(dout_mux_ctrl_i),
	      dout_gpio_state(dout_gpio_state_i){}

      P0R53_t(uint8_t reg){
	bus_keeper_en = ((reg&0x10)>>4);
	dout_mux_ctrl = (DOUT_MUX_CTRL_t)((reg&0x0E)>>1);
	dout_gpio_state = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (bus_keeper_en<<4)|(dout_mux_ctrl<<1)|(dout_gpio_state);}

      void setBusKeepEn(bool p){bus_keeper_en = p;}
      bool getBusKeepEn(){return bus_keeper_en;}

      void setDoutMuxCtrl(DOUT_MUX_CTRL_t p){dout_mux_ctrl = p;}
      DOUT_MUX_CTRL_t getDoutMuxCtrl(){return dout_mux_ctrl;}

      void setDoutGpio(bool p){dout_gpio_state = p;}
      bool getDoutGpio(){return dout_gpio_state;}
};

//Page 0 / Register 54: DIN Function Control Register - 0x00 / 0x36
class P0R54_t{
      const static int Address = 54;

      DIN_FUNC_CTRL_t din_func_ctrl;
      bool din_gpio_state;
    public:
      P0R54_t(DIN_FUNC_CTRL_t din_func_ctrl_i = DEFAULT_DIN_FUNC
	      ):din_func_ctrl(din_func_ctrl_i),
	      din_gpio_state(0){}

      P0R54_t(uint8_t reg){
	din_func_ctrl = (DIN_FUNC_CTRL_t)((reg&0x06)>>1);
	din_gpio_state = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (din_func_ctrl<<1)|(din_gpio_state);}

      void setDinFunc(DIN_FUNC_CTRL_t p){din_func_ctrl = p;}
      DIN_FUNC_CTRL_t getDinFunc(){return din_func_ctrl;}

      bool getDinGPIOState(){return din_gpio_state;}
};

//Page 0 / Register 55: MISO Function Control Register - 0x00 / 0x37
class P0R55_t{
      const static int Address = 55;

      MISO_FUNC_CTRL_t miso_func_ctrl;
      bool miso_gpio_state;
    public:
      P0R55_t(MISO_FUNC_CTRL_t miso_func_ctrl_i = DEFAULT_MISO_FUNC,
	      bool miso_gpio_state_i = DEFAULT_MISO_GPIO_VAL
	      ):miso_func_ctrl(miso_func_ctrl_i),
	      miso_gpio_state(miso_gpio_state_i){}

      P0R55_t(uint8_t reg){
	miso_func_ctrl = (MISO_FUNC_CTRL_t)((reg&0x1E)>>1);
	miso_gpio_state = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (miso_func_ctrl<<1)|(miso_gpio_state);}

      void setMisoFunc(MISO_FUNC_CTRL_t p){miso_func_ctrl = p;}
      MISO_FUNC_CTRL_t getMisoFunc(){return miso_func_ctrl;}

      void setMISOGPIOState(bool p){miso_gpio_state = p;}
      bool getMISOGPIOState(){return miso_gpio_state;}
};
//Page 0 / Register 56: SCLK/DMDIN2 Function Control Register- 0x00 / 0x38
class P0R56_t{
      const static int Address = 56;

      SCLK_FUNC_CTRL_t sclk_func_ctrl;
      bool sclk_gpio_state;
    public:
      P0R56_t(SCLK_FUNC_CTRL_t sclk_func_ctrl_i = DEFAULT_SCLK_FUNC
	      ):sclk_func_ctrl(sclk_func_ctrl_i),
	      sclk_gpio_state(0){}

      P0R56_t(uint8_t reg){
	sclk_func_ctrl = (SCLK_FUNC_CTRL_t)((reg&0x1E)>>1);
	sclk_gpio_state = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (sclk_func_ctrl<<1)|(sclk_gpio_state);}

      void setSclkFunc(SCLK_FUNC_CTRL_t p){sclk_func_ctrl = p;}
      SCLK_FUNC_CTRL_t getSclkFunc(){return sclk_func_ctrl;}

      bool getSclkGPIOState(){return sclk_gpio_state;}
};

//Page 0 / Register 60: DAC Instruction Set - 0x00 / 0x3C
class P0R60_t{
      const static int Address = 60;
      DAC_INST_t dac_inst;

    public:
      P0R60_t(DAC_INST_t dac_inst_i = DEFAULT_DAC_INSTRUCIOTN):dac_inst(dac_inst_i){}

      P0R60_t(uint8_t reg){dac_inst = (DAC_INST_t)((reg&0x1F));}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dac_inst);}

      void setDacInstruction(DAC_INST_t p){dac_inst = p;}
      DAC_INST_t getDacInstruction(){return dac_inst;}
};

//Page 0 / Register 62: miniDSP_D Configuration Register - 0x00 / 0x3E
class P0R62_t{
      const static int Address = 62;

      bool dsp_aux_bit_a;
      bool dsp_aux_bit_b;
      bool dsp_reset_inst_cnt;
    public:
      P0R62_t(bool dsp_aux_bit_a_i = DEFAULT_MINIDSP_AUX_BIT_A,
	      bool dsp_aux_bit_b_i = DEFAULT_MINIDSP_AUX_BIT_B,
	      bool dsp_reset_inst_cnt_i = DEFAULT_MINIDSP_RESET_INST_COUNTER
	      ):dsp_aux_bit_a(dsp_aux_bit_a_i),
	      dsp_aux_bit_b(dsp_aux_bit_b_i),
	      dsp_reset_inst_cnt(dsp_reset_inst_cnt_i){}

      P0R62_t(uint8_t reg){
	dsp_aux_bit_a = ((reg&0x04)>>2);
	dsp_aux_bit_b = ((reg&0x02)>>1);
	dsp_reset_inst_cnt = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dsp_aux_bit_a<<2)|(dsp_aux_bit_b<<1)|(dsp_reset_inst_cnt);}

      void setAuxBitA(bool p){dsp_aux_bit_a = p;}
      bool getAuxBitA(){return dsp_aux_bit_a;}

      void setAuxBitB(bool p){dsp_aux_bit_b = p;}
      bool getAuxBitB(){return dsp_aux_bit_b;}

      void setResetInstCnt(bool p){dsp_reset_inst_cnt = p;}
      bool getResetInstCnt(){return dsp_reset_inst_cnt;}
};

//Page 0 / Register 63: DAC Channel Setup Register 1 - 0x00 / 0x3F
class P0R63_t{
      const static int Address = 63;

      POWER_STATE_t dac_pwr_ctrl;
      DAC_DATA_PATH_t data_path;
      uint8_t chan_col_ctrl:2;
    public:
      P0R63_t(POWER_STATE_t dac_pwr_ctrl_i = DEFAULT_DAC_CHAN_POWER,
	      DAC_DATA_PATH_t data_path_i = DEFAULT_DAC_DATA_PATH,
	      uint8_t chan_col_ctrl_i = DEFAULT_DAC_CHAN_VOLUME
	      ):dac_pwr_ctrl(dac_pwr_ctrl_i),
	      data_path(data_path_i),
	      chan_col_ctrl(chan_col_ctrl_i){}

      P0R63_t(uint8_t reg){
	dac_pwr_ctrl = (POWER_STATE_t)((reg&0x80)>>7);
	data_path = (DAC_DATA_PATH_t)((reg&0x30)>>4);
	chan_col_ctrl = ((reg&0x03));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dac_pwr_ctrl<<7)|(data_path<<4)|(chan_col_ctrl);}

      void setDacChanPwr(POWER_STATE_t p){dac_pwr_ctrl = p;}
      POWER_STATE_t getDacChanPwr(){return dac_pwr_ctrl;}

      void setDacDataPath(DAC_DATA_PATH_t p){data_path = p;}
      DAC_DATA_PATH_t getDacDataPath(){return data_path;}

      void setDacChanVolume(uint8_t p){chan_col_ctrl = p&0x03;}
      uint8_t getDacChanVolume(){return chan_col_ctrl;}
};

//Page 0 / Register 64: DAC Channel Setup Register 2 - 0x00 / 0x40
class P0R64_t{
      const static int Address = 64;

      POWER_STATE_t dac_pwr_ctrl;
      DAC_DATA_PATH_t data_path;
      uint8_t chan_col_ctrl:2;
    public:
      P0R64_t(POWER_STATE_t dac_pwr_ctrl_i = DEFAULT_DAC_CHAN_POWER,
	      DAC_DATA_PATH_t data_path_i = DEFAULT_DAC_DATA_PATH,
	      uint8_t chan_col_ctrl_i = DEFAULT_DAC_CHAN_VOLUME
	      ):dac_pwr_ctrl(dac_pwr_ctrl_i),
	      data_path(data_path_i),
	      chan_col_ctrl(chan_col_ctrl_i){}

      P0R64_t(uint8_t reg){
	dac_pwr_ctrl = (POWER_STATE_t)((reg&0x80)>>7);
	data_path = (DAC_DATA_PATH_t)((reg&0x30)>>4);
	chan_col_ctrl = ((reg&0x03));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dac_pwr_ctrl<<7)|(data_path<<4)|(chan_col_ctrl);}

      void setDacChanPwr(POWER_STATE_t p){dac_pwr_ctrl = p;}
      POWER_STATE_t getDacChanPwr(){return dac_pwr_ctrl;}

      void setDacDataPath(DAC_DATA_PATH_t p){data_path = p;}
      DAC_DATA_PATH_t getDacDataPath(){return data_path;}

      void setDacChanVolume(uint8_t p){chan_col_ctrl = p&0x03;}
      uint8_t getDacChanVolume(){return chan_col_ctrl;}
};

//Page 0 / Register 65: DAC Channel Digital Volume Control Register - 0x00 / 0x41
class P0R65_t{
      const static int Address = 65;
      uint8_t dac_chan_vol;
    public:
      P0R65_t(uint8_t dac_chan_vol_i = DEFAULT_DAC_CHAN_VOLUME):dac_chan_vol(dac_chan_vol_i){}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return dac_chan_vol;}

      void setDacChanVol(uint8_t p){dac_chan_vol = p;}
      uint8_t getDacChanVol(){return dac_chan_vol;}
};

//////////////////////////////////////////////////////////////////
//	Page 1: DAC Routing, Power-Controls and MISC Logic
// 		Related Programmabilities
//////////////////////////////////////////////////////////////////

//Page 1 / Registers 1: REF, POR and LDO BGAP Control Register - 0x01 / 0x01
class P1R1_t{
      const static int Address = 1;
      POWER_STATE_t mast_ref_ctrl;
      bool	por_pwr_ctrl;
      POWER_STATE_t ldo_pwr_ctrl;
    public:
      P1R1_t( POWER_STATE_t mast_ref_ctrl_i = DEFAULT_MAST_REF_CTRL,
	      bool	por_pwr_ctrl_i = DEFAULT_POR_PWR_CTRL,
	      POWER_STATE_t ldo_pwr_ctrl_i = DEFAULT_LDO_PWR_CTRL):
	      mast_ref_ctrl(mast_ref_ctrl_i),
	      por_pwr_ctrl(por_pwr_ctrl_i),
	      ldo_pwr_ctrl(ldo_pwr_ctrl_i){}

      P1R1_t(uint8_t reg){
	mast_ref_ctrl = (POWER_STATE_t)((reg & 0x10)>>4);
	por_pwr_ctrl = ((reg & 0x08)>>3);
	ldo_pwr_ctrl = (POWER_STATE_t)((reg & 0x02)>>1);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (mast_ref_ctrl<<4)|(por_pwr_ctrl<<3)|(ldo_pwr_ctrl<<1);}

      void setMastRefCtrl(POWER_STATE_t p){mast_ref_ctrl = p;}
      POWER_STATE_t getMastRefCtrl(){return mast_ref_ctrl;}

      void setPorPwrCtrl(bool p){por_pwr_ctrl = p;}
      bool getPorPwrCtrl(){return por_pwr_ctrl;}

      void setLdoPwrCtrl(POWER_STATE_t p){ldo_pwr_ctrl = p;}
      POWER_STATE_t getLdoPwrCtrl(){return ldo_pwr_ctrl;}
};
//Page 1 / Register 2: LDO Control Register - 0x01 / 0x02
class P1R2_t{
      const static int Address = 2;
      AVDD_LDO_CTRL_t nom_avdd_ldo_volt;
      bool pll_hp_lvl_shift_pwr;
      bool short_det;
      bool ldo_sel;
    public:
      P1R2_t( AVDD_LDO_CTRL_t nom_avdd_ldo_volt_i = DEFAULT_AVDD_LDO_CTRL,
	      bool pll_hp_lvl_shift_pwr_i = DEFAULT_PLL_HP_LVL_SHIFT_PWR):
		nom_avdd_ldo_volt(nom_avdd_ldo_volt_i),
		pll_hp_lvl_shift_pwr(pll_hp_lvl_shift_pwr_i),
		short_det(false),
		ldo_sel(false){}

      P1R2_t(uint8_t reg){
	nom_avdd_ldo_volt = (AVDD_LDO_CTRL_t)((reg & 0x30)>>4);
	pll_hp_lvl_shift_pwr = ((reg & 0x08)>>3);
	short_det = ((reg & 0x02)>>1);
	ldo_sel = (reg & 0x01);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (nom_avdd_ldo_volt<<4)|(pll_hp_lvl_shift_pwr<<3);}

      void setAvddLdoVoltage(AVDD_LDO_CTRL_t p){nom_avdd_ldo_volt = p;}
      AVDD_LDO_CTRL_t getAvddLdoVoltage(){return nom_avdd_ldo_volt;}

      void setLvlShiftPwr(bool p){pll_hp_lvl_shift_pwr = p;}
      bool getLvlShiftPwr(){return pll_hp_lvl_shift_pwr;}

      bool getAvddLdoShort(){return short_det;}
      bool getLdoSelState(){return ldo_sel;}
};
//Page 1 / Playback Configuration Register 1 - 0x01 / 0x03
class P1R3_t{
      const static int Address = 3;
      DAC_MODE_t dac_mode;
      uint8_t dac_ptm;

    public:
      P1R3_t( DAC_MODE_t dac_mode_i = DEFAULT_DAC_MODE,
	      uint8_t dac_ptm_i = DEFAULT_DAC_PTM_CTRL):
		dac_mode(dac_mode_i),
		dac_ptm(dac_ptm_i){}

      P1R3_t(uint8_t reg){
	dac_mode = (DAC_MODE_t)((reg & 0x20)>>5);
	dac_ptm = ((reg & 0x1C)>>2);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dac_mode<<5)|(dac_ptm<<2);}

      void setDacMode(DAC_MODE_t p){dac_mode = p;}
      DAC_MODE_t getDacMode(){return dac_mode;}

      void setDacPtmCtrl(uint8_t p){dac_ptm = p;}
      uint8_t geDacPtmCtrl(){return dac_ptm;}
};

//Page 1 / Register 8: DAC PGA Control Register- 0x01 / 0x08
class P1R8_t{
      const static int Address = 8;
      bool soft_step_en;
      bool soft_step_mode;

    public:
      P1R8_t( bool soft_step_en_i = DEFAULT_SOFT_STEPPING_ENABLE,
	      bool soft_step_mode_i = DEFAULT_SOF_STEPPING_MODE):
		soft_step_en(soft_step_en_i),
		soft_step_mode(soft_step_mode_i){}

      P1R8_t(uint8_t reg){
	soft_step_en = ((reg & 0x80)>>7);
	soft_step_mode = ((reg & 0x40)>>6);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (soft_step_en<<7)|(soft_step_mode<<6);}

      void setSoftStepEn(bool p){soft_step_en = p;}
      bool getSoftStepEn(){return soft_step_en;}

      void setSoftStepMode(bool p){soft_step_mode = p;}
      bool getSoftStepMode(){return soft_step_mode;}
};

//Page 1 / Register 9: Output Drivers, AINL, AINR, Control Register - 0x01 / 0x09
class P1R9_t{
      const static int Address = 9;
      POWER_STATE_t hpl_pwr;
      bool ainl_en;
      bool ainr_en;

    public:
      P1R9_t( POWER_STATE_t hpl_pwr_i = DEFAULT_HPL_PWR,
	      bool ainl_en_i = DEFAULT_AINL_INPUT_EN,
	      bool ainr_en_i = DEFAULT_AINR_INPUT_EN):
		hpl_pwr(hpl_pwr_i),
		ainl_en(ainl_en_i),
		ainr_en(ainr_en_i){}

      P1R9_t(uint8_t reg){
	hpl_pwr = (POWER_STATE_t)((reg & 0x20)>>5);
	ainl_en = ((reg & 0x02)>>1);
	ainr_en = ((reg & 0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (hpl_pwr<<5)|(ainl_en<<1)|(ainr_en);}

      void setHplPwr(POWER_STATE_t p){hpl_pwr = p;}
      POWER_STATE_t getHplPwr(){return hpl_pwr;}

      void setAinlEn(bool p){ainl_en = p;}
      bool getAinlEn(){return ainl_en;}

      void setAinrEn(bool p){ainr_en = p;}
      bool getAinrEn(){return ainr_en;}
};

//Page 1 / Register 10: Common Mode Control Register - 0x01 / 0x0A
class P1R10_t{
      const static int Address = 10;
      CHIP_COMMON_MODE_t comm_mode;
      HEADPHONE_DRIVE_ABILITY_t hp_drive;

    public:
      P1R10_t( CHIP_COMMON_MODE_t comm_mode_i = DEFAULT_CHIP_COMMON_MODE,
	       HEADPHONE_DRIVE_ABILITY_t hp_drive_i = DEFAULT_DRIVE_ABILITY):
		 comm_mode(comm_mode_i),
		hp_drive(hp_drive_i){}

      P1R10_t(uint8_t reg){
	comm_mode = (CHIP_COMMON_MODE_t)((reg & 0x40)>>6);
	hp_drive = (HEADPHONE_DRIVE_ABILITY_t)((reg & 0x04)>>2);
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (comm_mode<<6)|(hp_drive<<2);}

      void setCommonMode(CHIP_COMMON_MODE_t p){comm_mode = p;}
      CHIP_COMMON_MODE_t getCommonMode(){return comm_mode;}

      void setHpDrive(HEADPHONE_DRIVE_ABILITY_t p){hp_drive = p;}
      HEADPHONE_DRIVE_ABILITY_t getHpDrive(){return hp_drive;}
};

//Page 1 / Register 11: HP Over Current Protection Configuration Register - 0x01 / 0x0B
class P1R11_t{
      const static int Address = 11;
      OVER_CURRENT_DETECTION_DEBOUNCE_t debounce;
      OVER_CURRENT_CONDITION_t ovr_curr_cdt;
    public:
      P1R11_t( OVER_CURRENT_DETECTION_DEBOUNCE_t debounce_i = DEFAULT_OVER_CURRENT_DEBOUNCE,
	       OVER_CURRENT_CONDITION_t ovr_curr_cdt_i = DEFAULT_OVER_CURRENT_COND):
		 debounce(debounce_i),
		 ovr_curr_cdt(ovr_curr_cdt_i){}

      P1R11_t(uint8_t reg){
	debounce = (OVER_CURRENT_DETECTION_DEBOUNCE_t)((reg & 0x0E)>>1);
	ovr_curr_cdt = (OVER_CURRENT_CONDITION_t)((reg & 0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return 0x10|(debounce<<1)|(ovr_curr_cdt);}

      void setOvrCurrDebounce(OVER_CURRENT_DETECTION_DEBOUNCE_t p){debounce = p;}
      OVER_CURRENT_DETECTION_DEBOUNCE_t getOvrCurrDebounce(){return debounce;}

      void setOvrCurrCondition(OVER_CURRENT_CONDITION_t p){ovr_curr_cdt = p;}
      OVER_CURRENT_CONDITION_t getOvrCurrCondition(){return ovr_curr_cdt;}
};

//Page 1 / Register 12: HP Routing Selection Register - 0x01 / 0x0C
class P1R12_t{
      const static int Address = 12;
      ANALOG_ROUTING_t an_rout;
      bool dac_out;
      bool mixer_p;
      bool ainl;
      bool ainr;

    public:
      P1R12_t( ANALOG_ROUTING_t an_rout_i = DEFAULT_ANALOG_ROUTING,
	       bool dac_out_i = DEFAULT_DAC_TO_HP_ROUTING,
	       bool mixer_p_i = DEFAULT_CONNECT_MIX_P_TO_HP_ATTN,
	       bool ainl_i = DEFAULT_ROUT_AINL_ATTN_TO_HP,
	       bool ainr_i = DEFAULT_ROUT_AINR_ATTN_TO_HP):
		 an_rout(an_rout_i),
		 dac_out(dac_out_i),
		 mixer_p(mixer_p_i),
		 ainl(ainl_i),
		 ainr(ainr_i){}

      P1R12_t(uint8_t reg){
	an_rout = (ANALOG_ROUTING_t)((reg & 0xF0)>>4);
	dac_out = ((reg & 0x08)>>3);
	mixer_p = ((reg & 0x04)>>2);
	ainl = ((reg & 0x02)>>1);
	ainr = ((reg & 0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (an_rout<<4)|(dac_out<<3)|(mixer_p<<2)|(ainl<<1)|(ainr);}

      void setAnalogRouting(ANALOG_ROUTING_t p){an_rout = p;}
      ANALOG_ROUTING_t getAnalogRouting(){return an_rout;}

      void setDacRout(bool p){dac_out = p;}
      bool getDacRout(){return dac_out;}

      void setMixerPtoHpAttn(bool p){mixer_p = p;}
      bool getMixerPtoHpAttn(){return mixer_p;}

      void setAinlAttn(bool p){ainl = p;}
      bool getAinlAttn(){return ainl;}

      void setAinrAttn(bool p){ainr = p;}
      bool getAinrAttn(){return ainr;}
};

//Page 1 / Register 16: HP Driver Gain Setting Register - 0x01 / 0x10
class P1R16_t{
      const static int Address = 16;
      bool hp_drv_mute;
      uint8_t hp_drv_gain;

    public:
      P1R16_t( bool hp_drv_mute_i = DEFAULT_MUTE_HP,
	       uint8_t hp_drv_gain_i = DEFAULT_HP_DRIVER_GAIN):
		 hp_drv_mute(hp_drv_mute_i),
		 hp_drv_gain(hp_drv_gain_i){}

      P1R16_t(uint8_t reg){
	hp_drv_mute = ((reg & 0x40)>>6);
	hp_drv_gain = ((reg & 0x3F));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (hp_drv_mute<<6)|(hp_drv_gain);}

      void setHpDrvMute(bool p){hp_drv_mute = p;}
      bool getHpDrvMute(){return hp_drv_mute;}

      void setHpDrvGain(uint8_t p){hp_drv_gain = p;}
      uint8_t getHpDrvGain(){return hp_drv_gain;}
};

//Page 1 / Registers 20: Headphone Driver Startup Control Register - 0x01 / 0x14
class P1R20_t{
      const static int Address = 20;
      SOFT_ROUTING_STEP_TIME_t soft_rout_step;
      HP_AMP_SLOW_POWERUP_TIME_t slow_pwrup_time;
      HP_AMP_POWERUP_RESISTOR_t pwrup_res;
    public:
      P1R20_t( SOFT_ROUTING_STEP_TIME_t soft_rout_step_i = DEFAULT_SOFT_ROUT_STEP_TIME,
	       HP_AMP_SLOW_POWERUP_TIME_t slow_pwrup_time_i = DEFAULT_HP_SLOW_PWRUP,
	       HP_AMP_POWERUP_RESISTOR_t pwrup_res_i = DEFAULT_HP_PWRUP_RES):
		 soft_rout_step(soft_rout_step_i),
		 slow_pwrup_time(slow_pwrup_time_i),
		 pwrup_res(pwrup_res_i){}

      P1R20_t(uint8_t reg){
	soft_rout_step = (SOFT_ROUTING_STEP_TIME_t)((reg & 0xC0)>>6);
	slow_pwrup_time = (HP_AMP_SLOW_POWERUP_TIME_t)((reg & 0x3C)>>2);
	pwrup_res = (HP_AMP_POWERUP_RESISTOR_t)((reg & 0x03));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (soft_rout_step<<6)|(slow_pwrup_time<<2)|(pwrup_res);}

      void setsoftRoutingStepTime(SOFT_ROUTING_STEP_TIME_t p){soft_rout_step = p;}
      SOFT_ROUTING_STEP_TIME_t getsoftRoutingStepTime(){return soft_rout_step;}

      void setSlowHpAmpPwrUp(HP_AMP_SLOW_POWERUP_TIME_t p){slow_pwrup_time = p;}
      HP_AMP_SLOW_POWERUP_TIME_t getSlowHpAmpPwrUp(){return slow_pwrup_time;}

      void setHpAmpPwrUpResistor(HP_AMP_POWERUP_RESISTOR_t p){pwrup_res = p;}
      HP_AMP_POWERUP_RESISTOR_t getHpAmpPwrUpResistor(){return pwrup_res;}
};

//Page 1 / Register 22: HP Volume Control Register - 0x01 / 0x16
class P1R22_t{
      const static int Address = 22;
      uint8_t volume;
    public:
      P1R22_t( uint8_t volume_i = DEFAULT_HP_VOLUME):volume(volume_i&0x7F){}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return volume;}

      void setVolume(uint8_t p){volume = p & 0x7F;}
      uint8_t getVolume(){return volume;}
};

//Page 1 / Register 24: AINL Volume Control Register - 0x01 / 0x18
class P1R24_t{
      const static int Address = 24;
      bool mixer_p_force_en;
      uint8_t volume;
    public:
      P1R24_t( bool mixer_p_force_en_i = DEFAULT_MIXER_P_M_FORCE_EN,
	       uint8_t volume_i = DEFAULT_AINL_VOLUME):
		 mixer_p_force_en(mixer_p_force_en_i),
		 volume(volume_i&0x7F){}

      P1R24_t(uint8_t reg){
	mixer_p_force_en = ((reg &0x80)>>7);
	volume = (reg & 0x7F);
      }
      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (mixer_p_force_en<<7)|(volume);}

      void setMixerPMForceEn(bool p){volume = p & 0x7F;}
      bool getMixerPMForceEn(){return volume;}

      void setVolume(uint8_t p){volume = p & 0x7F;}
      uint8_t getVolume(){return volume;}
};
//Page 1 / Register 25: AINR Volume Control Register - 0x01 / 0x19
class P1R25_t{
      const static int Address = 25;
      uint8_t volume;
    public:
      P1R25_t( uint8_t volume_i = DEFAULT_AINR_VOLUME): volume(volume_i&0x7F){}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (volume);}

      void setVolume(uint8_t p){volume = p & 0x7F;}
      uint8_t getVolume(){return volume;}
};
//Page 1 / Register 45: Speaker Amplifier Control 1 - 0x01 / 0x2D
class P1R45_t{
      const static int Address = 45;
      POWER_STATE_t spk_drv_pwr;
    public:
      P1R45_t( POWER_STATE_t spk_drv_pwr_i = DEFAULT_SPK_OUT_POWER): spk_drv_pwr(spk_drv_pwr_i){}
      P1R45_t(int8_t reg)
      {
	spk_drv_pwr = (POWER_STATE_t)((reg&0x02)>>1);
      }
      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (spk_drv_pwr<<1);}

      void setSpkDrvPwr(POWER_STATE_t p){spk_drv_pwr = p;}
      POWER_STATE_t getSpkDrvPwr(){return spk_drv_pwr;}
};
//Page 1 / Register 46: Speaker Volume Control 1 - 0x01 / 0x2E
class P1R46_t{
      const static int Address = 46;
      uint8_t volume;
    public:
      P1R46_t( uint8_t volume_i = DEFAULT_SPK_VOLUME): volume(volume_i&0x7F){}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (volume);}

      void setVolume(uint8_t p){volume = p & 0x7F;}
      uint8_t getVolume(){return volume;}
};
//Page 1 / Register 48: Speaker Amplifier Volume Control 2 - 0x01 / 0x30
class P1R48_t{
      const static int Address = 48;
      SPK_AMP_VOL_t volume;
    public:
      P1R48_t( SPK_AMP_VOL_t volume_i = DEFAULT_SPK_AMP_VOL): volume(volume_i){}
      P1R48_t(int8_t reg){volume = (SPK_AMP_VOL_t)((reg&0x70)>>4);}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (volume);}

      void setVolume(SPK_AMP_VOL_t p){volume = p;}
      SPK_AMP_VOL_t getVolume(){return volume;}
};

//Page 1 / Register 63: DAC Analog Gain Control Flag Register - 0x01 / 0x3F
class P1R63_t{
      const static int Address = 63;
      bool hp_gain;
      bool ain1l_mix;
      bool left_mix;
      bool right_mix;

    public:
      P1R63_t(int8_t reg){
	hp_gain = (reg&0x80)>>7;
	ain1l_mix = (reg&0x08)>>3;
	left_mix = (reg&0x02)>>1;
	right_mix = (reg&0x1);
      }

      uint8_t GetAddress(){return Address;}

      bool getHPGainFlag(){return hp_gain;}
      bool getAIN1LMixPGAFlag(){return ain1l_mix;}
      bool getLeftMixerPGAFlag(){return left_mix;}
      bool getRightMixerPGAFlag(){return right_mix;}
};
//Page 1 / Register 122: Reference Power Up Delay - 0x01 / 0x7A
class P1R122_t{
      const static int Address = 122;
      REF_PWRUP_DELAY_t delay;
    public:
      P1R122_t( REF_PWRUP_DELAY_t delay_i = DEFAULT_REF_PWRUP_DELAY): delay(delay_i){}
      P1R122_t(int8_t reg){delay = (REF_PWRUP_DELAY_t)((reg&0x07));}

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (delay);}

      void setPwrUpDelay(REF_PWRUP_DELAY_t p){delay = p;}
      REF_PWRUP_DELAY_t getPwrUpDelay(){return delay;}
};
//////////////////////////////////////////////////////////////
//	Page44: DAC Programmable Coefficients RAM
//////////////////////////////////////////////////////////////
//Page 44 / Register 1: DAC Adaptive Filter Configuration Register - 0x2C / 0x01
class P44R1_t{
      const static int Address = 1;
      bool dac_filt_ctrl;
      bool dac_filt_buf_ctrl_flag;
      bool dac_filt_buf;

    public:
      P44R1_t( bool dac_filt_ctrl_i = DEFAULT_DAC_ADAP_FILT_EN,
	       bool dac_filt_buf_i = DEFAULT_DAC_ADAP_FILT_BUF_SWITCH):
		 dac_filt_ctrl(dac_filt_ctrl_i),
		 dac_filt_buf_ctrl_flag(false),
		 dac_filt_buf(dac_filt_buf_i){}

      P44R1_t(int8_t reg){
	dac_filt_ctrl = ((reg&0x04)>>2);
	dac_filt_buf_ctrl_flag = ((reg&0x02)>>1);
	dac_filt_buf = ((reg&0x01));
      }

      uint8_t GetAddress(){return Address;}

      uint8_t toByte(){return (dac_filt_ctrl<<2)|(dac_filt_buf);}

      void setDacAdapFiltEn(bool p){dac_filt_ctrl = p;}
      bool getDacAdapFiltEn(){return dac_filt_ctrl;}

      bool getDacAdapFiltBufFlag(){return dac_filt_buf_ctrl_flag;}

      void setDacAdapFiltBufSwitch(bool p){dac_filt_buf = p;}
      bool getDacAdapFiltBufSwitch(){return dac_filt_buf;}
};






class FyberLabs_TAS2521 {

  const uint8_t _i2c_address;
  uint8_t _page;
  void switchPage(uint8_t page);
  uint8_t read8(uint8_t reg);
  void write8(uint8_t reg, uint8_t data);
public:
  	FyberLabs_TAS2521(uint8_t addr=0x18):_i2c_address(addr){};
  	uint8_t GetAddress(void){return _i2c_address;}
  	void begin(void);
  	void reset(void);

  	void setPLLCLKRangeLow(void);
  	void setPLLCLKRangeHigh(void);
  	/*
  		Select PLL Input Clock
		00: MCLK pin is input to PLL
		01: BCLK pin is input to PLL
		10: GPIO pin is input to PLL
		11: DIN pin is input to PLL
  	*/
  	void setPLLCLK(PLL_CLKIN_t);
  	/*
  		Select CODEC_CLKIN
		00: MCLK pin is CODEC_CLKIN
		01: BCLK pin is CODEC_CLKIN
		10: GPIO pin is CODEC_CLKIN
		11: PLL Clock is CODEC_CLKIN
  	*/
  	void setCODECCLK(CODEC_CLKIN_t);

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

};

}//namespace TAS2521

#endif

