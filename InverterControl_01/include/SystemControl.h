/**********************************************************************************
// File: SystemControl.h
// Devices: TMS320C28346
//_________________________________________________________________________________
//
// Tecogen	Inc.                               				Copyright © 2012
//
// Authors:		Jian Wen
//
//
// Description:
// ------------
// This is the header file for SystemControl.c.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/27/12	|  J Wen 	| Original
**********************************************************************************/

#ifndef SYSTEMCONTRL_H_
#define SYSTEMCONTRL_H_

typedef struct {
   Uint16 state;
   Uint16 status;
   Uint16 setting;
   Uint16 faults;
   Uint16 SW_faults;
   Uint16 SW_warning;
   Uint16 command;
   Uint16 EMS;
   Uint16 InverterTest;
   Uint16 FlashEnable;
   Uint16 TestMode;
   Uint16 HostActiveCnt;
}SYSTEM_INFO;

typedef struct {
   Uint16 crc_CheckSUM;
   Uint16 Flash_Date;
   Uint16 Flash_Year;
}FLASH_INFO;

//  System STATES definition
#define    	SYS_INIT              0
#define    	STANDBY_MODE          1
#define  	MICRO_GRID_MODE    	  2
#define  	GRID_TIE_MODE         3
#define  	DC_LINK_CTRL_MODE     4
#define  	BYPASS                5
#define  	GRID_TIE_STANDBY      6
#define  	FAULT_CONDITION       7
#define    	DISCHARGE             8
#define  	SYS_OFF               9
#define  	FLASH_PROGRAMMING    10
#define  	UNKNOWN_STATE        11

//  System STATUS definition
#define    	HOST_COMMUNICATION     	0x0001		// bit 0
#define    	OVER_LOAD			   	0x0002		// bit 1
#define    	BOOST_ON			   	0x0004		// bit 2
#define    	INVERTER_ON			   	0x0008		// bit 3

#define    	AC_FREQUENCY_OK		   	0x0010		// bit 4
#define    	PLL_ENABLE			   	0x0020		// bit 5
#define    	PLL_LOCK			   	0x0040		// bit 6
#define    	AC_LINE_OK			   	0x0080		// bit 7

#define    	MICRO_GRID_ENABLE	   	0x0100		// bit 8
#define    	BOOST2_ON			   	0x0200		// bit 9
#define    	BRAKE_ON			   	0x0400		// bit 10

#define    	EEPROM_OK              	0x2000		// bit 13
#define    	GT_SPEED_CTRL          	0x4000		// bit 14

//  Software FAULTS definition
#define    	DC_BUS_OVERVOLTAGE		0x0001		// bit 0
#define    	GENERATOR_OVERCURRENT	0x0002		// bit 1
#define    	EMBIENT_OVER_TEMP		0x0004		// bit 2
#define    	PHASE_ROTATION_FAULT	0x0008		// bit 3
#define    	DC_BUS_UNDERVOLTAGE		0x0010		// bit 4
#define    	FILTER_CONTACTER_FAULT	0x0020		// bit 5
#define    	OUTPUT_CONTACTER_FAULT	0x0040		// bit 6
#define    	UL_1741_TRIP			0x0080		// bit 7
#define    	HOST_COMM_FAULT    		0x0100		// bit 8
#define    	OVER_LOAD_FAULT    		0x0200		// bit 9
#define    	BOOST_OVER_CURRENT 		0x0400		// bit 10
#define    	I_GEN_PEAK_OVERCURRENT	0x0800		// bit 11

//  Software WARNING definition
#define    	I_OUT_IMBALANCE			0x0001		// bit 0
#define    	I_GEN_IMBALANCE			0x0002		// bit 1


//  System SETTING COMMAND definition
#define    	CMD_MICRO_GRID_MODE		0x0001		
#define    	CMD_GRID_TIE_MODE		0x0002		
#define 	SysCtrlCmd_TestMode		0x0003
#define 	SysCtrlCmd_TestModeOff	0x0004

#define    	HOST_ACTIVE				0xAA55		

//  System SETTING definition
#define    	TEST_MODE				0x0001		// bit 0
#define    	OP_MODE_MG_0_GT_1		0x0002		// bit 1
#define    	FREQ_SET_60HZ_0_50HZ_1	0x0004		// bit 2
#define    	ANTI_ISLANDING_ACTIVE	0x0008		// bit 3
#define    	SURGE_MODULE_READY		0x0010		// bit 4
#define    	LINE_PH_ROTATION_ABC	0x0020		// bit 5

// Faults definition
#define    	HEATSINK_OVER_TEMP    	0x1000		// bit 12
#define    	BRAKE_OVER_TEMP    		0x2000		// bit 13
#define    	SW_FAULT_TRUE    		0x8000		// bit 15


//  System operation command definition
#define		SysCtrlCmd_MainOutputOn		0x03
#define		SysCtrlCmd_MainOutputOff	0x04
#define 	SysCtrlCmd_Fault_Reset		0x08
#define 	SysCtrlCmd_BoostOn			0x09
#define 	SysCtrlCmd_BoostOff			0x0A
#define 	SysCtrlCmd_Discharge		0x0F
#define 	SysCtrlCmd_SystemReset		0x10
#define 	SysCtrlCmd_FlashProgram		0x11
#define 	SysCtrlCmd_SPI_A_Test		0x12
#define 	SysCtrlCmd_FlashComplete	0x13
#define 	SysCtrlCmd_DoNothing		0x00
#define 	SysCtrlCmd_2ndBoostOn		0x19
#define 	SysCtrlCmd_2ndBoostOff		0x1A
#define 	SysCtrlCmd_BrakeOn			0x1B
#define 	SysCtrlCmd_BrakeOff			0x1C

//  System operation parameters definition
#define		SYSTEM_OUTPUT_VOLTAGE_MAX		380.0	// Max Vout
#define		SYSTEM_OUTPUT_VOLTAGE_NOMINAL	277.0	// Nominal Vout

#define		PHASE_REF_OFFSET				-1.8	// for ABC rotation


#endif /* SYSTEMCONTRL_H_ */
