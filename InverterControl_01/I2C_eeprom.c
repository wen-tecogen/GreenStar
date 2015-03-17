/**********************************************************************************
// File: I2C_eeprom.c
// Devices: TMS320C28346
//_________________________________________________________________________________
//
// Tecogen	Inc.                               				Copyright � 2012
//
// Authors:		Jian Wen
//
//
// Description:
// ------------
// This module contains EEPROM access function.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/22/12	|  J Wen 	| Original
**********************************************************************************/


#include "DSP28x_Project.h"     // Device Header file
#include "SystemControl.h"      // System control Header file
#include "I2C_eeprom.h"			// EEPROM header file

// Note: I2C Macros used in this example can be found in the
// DSP2834x_I2C_defines.h file

// Prototype statements for functions found within this file.
void   I2CA_Init(void);
Uint16 I2CA_WriteData(struct I2CMSG *msg);
Uint16 I2CA_ReadData(struct I2CMSG *msg);
//void I2C_eeprom(void);
void Load_eeprom(void);
Uint16 crc16(Uint16 *message, Uint16 length);

#define I2C_EEPROM_HIGH_ADDR  0x00
#define I2C_EEPROM_LOW_ADDR   0x30

// Global variables
// Two bytes will be used for the outgoing address,
// thus only setup 14 bytes maximum
struct I2CMSG I2cMsgOut1={I2C_MSGSTAT_SEND_WITHSTOP,
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          I2C_EEPROM_HIGH_ADDR,
                          I2C_EEPROM_LOW_ADDR,
                          0x12,                   // Msg Byte 1
                          0x34,                   // Msg Byte 2
                          0x56,                   // Msg Byte 3
                          0x78,                   // Msg Byte 4
                          0x9A,                   // Msg Byte 5
                          0xBC,                   // Msg Byte 6
                          0xDE,                   // Msg Byte 7
                          0xF0,                   // Msg Byte 8
                          0x11,                   // Msg Byte 9
                          0x10,                   // Msg Byte 10
                          0x11,                   // Msg Byte 11
                          0x12,                   // Msg Byte 12
                          0x13,                   // Msg Byte 13
                          0x12};                  // Msg Byte 14


struct I2CMSG I2cMsgIn1={ I2C_MSGSTAT_SEND_NOSTOP,
                          I2C_SLAVE_ADDR,
                          I2C_NUMBYTES,
                          I2C_EEPROM_HIGH_ADDR,
                          I2C_EEPROM_LOW_ADDR};

struct I2CMSG *CurrentMsgPtr;				// Used in interrupts
Uint16 PassCount;
Uint16 FailCount;

extern Uint16	CRC_checkSum;
extern SYSTEM_INFO system;

/***************************************
void I2C_eeprom(void) {
Uint16 IntSource;
Uint16 i;

	//////////////////////////////////
	// Write data to EEPROM section //
	//////////////////////////////////
	I2CA_WriteData(&I2cMsgOut1);

	IntSource = I2caRegs.I2CISRC.all;

	// Wait for stop condition detected
	while(IntSource != I2C_SCD_ISRC) {
		IntSource = I2caRegs.I2CISRC.all;
	}

    // end of write section
	DELAY_US(2500L); // Delay 2.5 msec for eeprom writing.

	///////////////////////////////////
	// Read data from EEPROM section //
	///////////////////////////////////
	while (I2caRegs.I2CMDR.bit.STP == 1) {
	 // Wait until the STP bit is cleared from any previous master communication.
	}

	I2caRegs.I2CSAR = I2cMsgIn1.SlaveAddress;
	I2caRegs.I2CCNT = 2;
	I2caRegs.I2CDXR = I2cMsgIn1.MemoryHighAddr;
	I2caRegs.I2CDXR = I2cMsgIn1.MemoryLowAddr;
	I2caRegs.I2CMDR.all = 0x2620;			// Send data to setup EEPROM address

	DELAY_US(100L);

    I2caRegs.I2CCNT = I2cMsgIn1.NumOfBytes;	// Setup how many bytes to expect
    I2caRegs.I2CMDR.all = 0x2C20;			// Send restart as master receiver

	DELAY_US(100L);
	while (I2caRegs.I2CMDR.bit.STP == 1) {
	  // Wait until the STP bit is cleared from any previous master communication.
	}
    for(i=0; i < I2C_NUMBYTES; i++) {
      I2cMsgIn1.MsgBuffer[i] = I2caRegs.I2CDRR;
    }

}   // end of main
***************************************/

//
//  ======== I2CA_Init ========
//  I2C port initialization
//
void I2CA_Init(void) {
Uint16 i;

CurrentMsgPtr = &I2cMsgOut1;

   InitI2CGpio();
   // Initialize I2C
   I2caRegs.I2CSAR = 0x0050;		// Slave address - EEPROM control code
   I2caRegs.I2CPSC.all = 29;   		// Pre-scaler - need 7-12 Mhz on module clk (300/30 = 10MHz)

   I2caRegs.I2CCLKL = 10;			// NOTE: must be non zero
   I2caRegs.I2CCLKH = 5;			// NOTE: must be non zero
   I2caRegs.I2CIER.all = 0x24;		// Enable SCD & ARDY interrupts

   I2caRegs.I2CMDR.all = 0x0020;	// Take I2C out of reset
   									// Stop I2C when suspended
   I2caRegs.I2CFFTX.all = 0x6000;	// Enable FIFO mode and TXFIFO
   I2caRegs.I2CFFRX.all = 0x2040;	// Enable RXFIFO, clear RXFFINT,

   // Clear incoming message buffer
   for (i = 0; i < I2C_MAX_BUFFER_SIZE; i++) {
       I2cMsgIn1.MsgBuffer[i] = 0x0000;
   }

   I2cMsgOut1.MsgStatus = I2C_MSGSTAT_SEND_WITHSTOP;
   return;
}

//
//  ======== Load_eeprom ========
//  Load EEPROM data during system initialization
//
void Load_eeprom(void) {
Uint16 i, uintTemp;
Uint16 eepromData[I2C_NUMBYTES];

	///////////////////////////////////
	// Read data from EEPROM section //
	///////////////////////////////////
	while (I2caRegs.I2CMDR.bit.STP == 1) {
	 // Wait until the STP bit is cleared from any previous master communication.
	}

	I2caRegs.I2CSAR = I2cMsgIn1.SlaveAddress;
	I2caRegs.I2CCNT = 2;
	I2caRegs.I2CDXR = I2cMsgIn1.MemoryHighAddr;
	I2caRegs.I2CDXR = I2cMsgIn1.MemoryLowAddr;
	I2caRegs.I2CMDR.all = 0x2620;			// Send data to setup EEPROM address

	DELAY_US(100L);

    I2caRegs.I2CCNT = I2cMsgIn1.NumOfBytes;	// Setup how many bytes to expect
    I2caRegs.I2CMDR.all = 0x2C20;			// Send restart as master receiver

	DELAY_US(100L);
	while (I2caRegs.I2CMDR.bit.STP == 1) {
	  // Wait until the STP bit is cleared from any previous master communication.
	}
    for(i=0; i < I2C_NUMBYTES; i++) {
      I2cMsgIn1.MsgBuffer[i] = I2caRegs.I2CDRR;
      eepromData[i] = I2cMsgIn1.MsgBuffer[i];
    }
	CRC_checkSum = crc16(eepromData, 6);
	uintTemp = I2cMsgIn1.MsgBuffer[6] + (I2cMsgIn1.MsgBuffer[7] << 8);
	if (uintTemp == CRC_checkSum) {
		system.status |= EEPROM_OK;
	} else {
		system.status &= ~EEPROM_OK;
	}

}   // end of Load_eeprom();


Uint16 I2CA_WriteData(struct I2CMSG *msg) {
Uint16 i;
Uint16 IntSource;
Uint16 eepromData[I2C_NUMBYTES];

	for (i=0; i<I2C_NUMBYTES-2; i++) {
		eepromData[i] = msg->MsgBuffer[i];
	}
	CRC_checkSum = crc16(eepromData, 6);
	msg->MsgBuffer[I2C_NUMBYTES-2] = CRC_checkSum & 0x00FF;
	msg->MsgBuffer[I2C_NUMBYTES-1] = CRC_checkSum >> 8;

	// Wait until the STP bit is cleared from any previous master communication.
	// Clearing of this bit by the module is delayed until after the SCD bit is
	// set. If this bit is not checked prior to initiating a new message, the
	// I2C could get confused.
	if (I2caRegs.I2CMDR.bit.STP == 1) {
	   return I2C_STP_NOT_READY_ERROR;
	}

	// Setup slave address
	I2caRegs.I2CSAR = msg->SlaveAddress;

	// Check if bus busy
	if (I2caRegs.I2CSTR.bit.BB == 1) {
	   return I2C_BUS_BUSY_ERROR;
	}

	// Setup number of bytes to send
	// MsgBuffer + Address
	I2caRegs.I2CCNT = msg->NumOfBytes+2;

	// Setup data to send
	I2caRegs.I2CDXR = msg->MemoryHighAddr;
	I2caRegs.I2CDXR = msg->MemoryLowAddr;
	for (i=0; i<msg->NumOfBytes; i++) {
	   I2caRegs.I2CDXR = *(msg->MsgBuffer+i);
	}

	// Send start as master transmitter
	I2caRegs.I2CMDR.all = 0x6E20;

	IntSource = I2caRegs.I2CISRC.all;

	// Wait for stop condition detected
	while(IntSource != I2C_SCD_ISRC) {
		IntSource = I2caRegs.I2CISRC.all;
	}

    // end of write section
	DELAY_US(2500L); // Delay 2.5 msec for eeprom writing.

	return I2C_SUCCESS;
}


Uint16 I2CA_ReadData(struct I2CMSG *msg) {
   // Wait until the STP bit is cleared from any previous master communication.
   // Clearing of this bit by the module is delayed until after the SCD bit is
   // set. If this bit is not checked prior to initiating a new message, the
   // I2C could get confused.
   if (I2caRegs.I2CMDR.bit.STP == 1) {
      return I2C_STP_NOT_READY_ERROR;
   }

   I2caRegs.I2CSAR = msg->SlaveAddress;

   if(msg->MsgStatus == I2C_MSGSTAT_SEND_NOSTOP) {
      // Check if bus busy
      if (I2caRegs.I2CSTR.bit.BB == 1) {
         return I2C_BUS_BUSY_ERROR;
      }
      I2caRegs.I2CCNT = 2;
      I2caRegs.I2CDXR = msg->MemoryHighAddr;
      I2caRegs.I2CDXR = msg->MemoryLowAddr;
      I2caRegs.I2CMDR.all = 0x2620;			// Send data to setup EEPROM address
   } else if (msg->MsgStatus == I2C_MSGSTAT_RESTART) {
      I2caRegs.I2CCNT = msg->NumOfBytes;	// Setup how many bytes to expect
      I2caRegs.I2CMDR.all = 0x2C20;			// Send restart as master receiver
   }

   return I2C_SUCCESS;
}

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
//
//	Type:			Function
//	Name:			crc16
//
//	Description:	crc checksum function.
//
//	Version:		Release	Changes														test status
//					===============================================================================
//					020301	First version.												red
//
//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
Uint16 crc16(Uint16 *message, Uint16 length) {
Uint16  i, sum = 0xFFFF;
Uint16	j = 6;

	while (j--){
		sum = sum ^ (*message++ & 0x00FF);
		for (i=0; i < 8; i++){
			if ((sum & 0x0001) != 0)
				sum = (sum >> 1) ^ 0xA001;
			else  sum = (sum >> 1);
		}
	}
	return 	sum;
}

//===========================================================================
// End of file.
//===========================================================================
