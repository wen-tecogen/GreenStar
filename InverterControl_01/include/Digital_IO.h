/**********************************************************************************
// File: Digital_IO.h
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
// 1/11/13	|  J Wen 	| Original
**********************************************************************************/
#ifndef DIGITAL_IO_H_
#define DIGITAL_IO_H_


// Digital input/output
struct DI_BITS{          // bits   description
   Uint16 DI1_EMS:1;           // 0      GPIO22
   Uint16 DI2:1;           	// 1      GPIO23
   Uint16 DI3:1;           	// 2      GPIO34
   Uint16 DI4:1;           	// 3      GPIO14
   Uint16 DI5:1;           	// 4      GPIO52
   Uint16 DI6:1;           	// 5      GPIO53
   Uint16 DI7:1;           	// 6      GPIO58
   Uint16 DI8:1;           	// 7      GPIO59
   Uint16 DO1:1;           		// 0
   Uint16 DO2:1;           		// 1
   Uint16 DO3:1;           		// 2
   Uint16 DO4:1;           		// 3
   Uint16 DO5:1;           		// 4
   Uint16 DO6:1;           		// 5
   Uint16 DO7:1;           		// 6
   Uint16 DO8:1;           		// 7
   Uint16 rsvd2:8;           	// remaining 8 bits
   Uint16 rsvd3:8;           	// remaining 8 bits
};

union DIGITAL_IO {
   Uint32           all;
   struct DI_BITS 	bit;
};

#define	CONTACTOR_CLOSE		1
#define	CONTACTOR_OPEN		0


#endif /* FPGA_H_ */

