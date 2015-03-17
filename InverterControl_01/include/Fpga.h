/**********************************************************************************
// File: fpga.h
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
// This is the header file define FPGA location.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 11/21/12	|  J Wen 	| Original
**********************************************************************************/
#ifndef FPGA_H_
#define FPGA_H_

//---------------------------------------------------------------------------------
//--- FPGA access through zone_6 @ address: 0x100000
//--- Read from FPGA, XA16 = 0. Write to FPGA, XA16 = 1.
#define		pFPGA_RegWR	((volatile struct sFPGA_RegWR *) 0x110000)
#define		pFPGA_RegRD	((volatile struct sFPGA_RegRD *) 0x100000)



struct sFPGA_RegWR				// FPGA Data Object
{
	int16	Data_1;
	int16	Data_2;
	int16	Data_3;
	int16	Data_4;
	int16	Data_5;
	int16	Data_6;
	int16	Data_7;
	int16	Data_8;
	int16	Data_9;
	int16	Data_10;
	int16	Data_11;
	int16	Data_12;
	int16	Data_13;
	int16	Data_14;
	int16	Data_15;
	int16	Data_16;
}; // struct sFPGA_Reg

typedef struct {				// FPGA Data Object
	int16	Data_1;
	int16	Data_2;
	int16	Data_3;
	int16	Data_4;
	int16	Data_5;
	int16	Data_6;
	int16	Data_7;
	int16	Data_8;
	int16	Data_9;
	int16	Data_10;
	int16	Data_11;
	int16	Data_12;
	int16	Data_13;
	int16	Data_14;
	int16	Data_15;
	int16	Data_16;
}FPGA_REGWR_IMAGE;

struct sFPGA_RegRD				// FPGA Data Object
{
	int16	Data_1;
	int16	Data_2;
	int16	Data_3;
	int16	Data_4;
	int16	Data_5;
	int16	Data_6;
	int16	Data_7;
	int16	Data_8;
	int16	Data_9;
	int16	Data_10;
	int16	Data_11;
	int16	Data_12;
	int16	Data_13;
	int16	Data_14;
	int16	Data_15;
	int16	Data_16;
}; // struct sFPGA_Reg

// FPGA pFPGA_RegWR->Data_1 register bit function define
#define FAULT_RESET			0x0001		// Bit 0: Fault reset
#define DSP_HART_BEAT		0x0002		// Bit 1: DSP hart beat
#define DO_5				0x0004		// Bit 2: DO_5
#define DO_6				0x0008		// Bit 3: DO_5
#define DO_7				0x0010		// Bit 4: DO_5
#define BOOST_ON_LED		0x0020		// Bit 5: boost on indicator
#define INVERTER_ON_LED		0x0040		// Bit 6: inverter on indicator

// FPGA pFPGA_RegWR->Data_2 register bit function define
#define PWM_POWER_CTRL		0x0001		// Bit 0: SET = power on

#endif /* FPGA_H_ */

