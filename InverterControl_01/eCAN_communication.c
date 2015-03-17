/**********************************************************************************
// File: eCAN_communication.c
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
// This module contains data communication function by using eCAN.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/24/12	|  J Wen 	| Original
**********************************************************************************/
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>

#include "DSP28x_Project.h"     // Device Header file and Examples Include File
#include "EPWM_control.h"       // Device Header file and Examples Include File
#include "eCAN_communication.h" // Device Header file and Examples Include File

/******* Swi handle defined in swi.cfg *******/
extern const Swi_Handle swi1;
extern const Semaphore_Handle systemSema;
extern const Semaphore_Handle commSema;

//====== Variables in this module ======
Uint16 CAN_Tx_DelayCount;
struct ECAN_REGS ECanShadow;
ECAN_MESSAGE eCAN_TxMessage, eCAN_RxMessage;

//====== Function prototype ======
void eCAN_DataTx(ECAN_MESSAGE *eCAN_msg);
void eCAN_TxSetup(void);
void eCAN_RxSetup(void);
interrupt void ecan0intb_isr(void);    // eCAN-B system
interrupt void ecan1intb_isr(void);    // eCAN-B message

//
//  ======== eCAN_TxSetup ========
//  Setup CAN mailbox 25 as transmit channel
//
void eCAN_TxSetup(void) {
Uint32	CAN_DataID, CAN_Data1, CAN_Data2;
	/* Write to the MSGID field  */
	CAN_DataID = 0x90001005;
	ECanbMboxes.MBOX25.MSGID.all = CAN_DataID; // Extended Identifier

	/* Configure Mailbox under test as a Transmit mailbox */
	ECanShadow.CANMD.all = ECanbRegs.CANMD.all;
	ECanShadow.CANMD.bit.MD25 = 0;
	ECanbRegs.CANMD.all = ECanShadow.CANMD.all;

	/* Enable Mailbox under test */
	ECanShadow.CANME.all = ECanbRegs.CANME.all;
	ECanShadow.CANME.bit.ME25 = 1;
	ECanbRegs.CANME.all = ECanShadow.CANME.all;

	/* Write to DLC field in Master Control reg */
    ECanbMboxes.MBOX25.MSGCTRL.bit.DLC = 8;

    CAN_Data1 = 0x12345678;
    CAN_Data2 = 0x87654321;
	/* Write to the mailbox RAM field */
    ECanbMboxes.MBOX25.MDL.all = CAN_Data1;
    ECanbMboxes.MBOX25.MDH.all = CAN_Data2;

    CAN_Tx_DelayCount = 0;
}

//
//  ======== eCAN_RxSetup ========
//  Setup CAN mailbox #1 as receive channel
//
void eCAN_RxSetup(void) {
	EALLOW;
	//------ eCAN mail box #1 interrupt ------
	ECanShadow.CANMIM.all = 0;					// interrupt mask register
  	ECanShadow.CANMIM.bit.MIM1  = 1;  			// MB#1  Mailbox interrupt is enabled
  	ECanShadow.CANMIM.bit.MIM6  = 1;  			// MB#6  Mailbox interrupt is enabled
  	ECanbRegs.CANMIM.all = ECanShadow.CANMIM.all;
    ECanShadow.CANMIL.all = 0;					// interrupt level register
  	ECanShadow.CANMIL.bit.MIL1  = 1;  			// Int.-Level MB#1  -> I1EN
  	ECanShadow.CANMIL.bit.MIL6  = 1;  			// Int.-Level MB#6  -> I1EN
  	ECanbRegs.CANMIL.all = ECanShadow.CANMIL.all;
    ECanShadow.CANGIM.all = 0;					// interrupt mask register
  	ECanShadow.CANGIM.bit.I1EN = 1;  			// enable I1EN for mailbox interrupt
  	ECanShadow.CANGIM.bit.GIL = 0;				// Global interrupt level 0
  	ECanShadow.CANGIM.bit.I0EN = 1;  			// enable I0EN
  	ECanbRegs.CANGIM.all = ECanShadow.CANGIM.all;
	//------ Receiver Mailbox #MBX1 & 6 ------
  	ECanShadow.CANME.all = ECanbRegs.CANME.all;	// Mailbox Enable Register
  	ECanShadow.CANME.bit.ME1 = 0;          		// disable Mailbox #1
  	ECanShadow.CANME.bit.ME6 = 0;          		// disable Mailbox #6
	ECanbRegs.CANME.all = ECanShadow.CANME.all;	//

 	ECanbMboxes.MBOX1.MSGID.all = 0x00001001;	// temp receive ID
  	ECanbMboxes.MBOX1.MSGID.bit.IDE = 1;    	// Extended Identifier

 	ECanbMboxes.MBOX6.MSGID.all = 0xDEFFFFFF;	// temp receive ID
  	ECanbMboxes.MBOX6.MSGID.bit.IDE = 1;    	// Extended Identifier
  	ECanbMboxes.MBOX6.MSGID.bit.AME = 1;    	// Acceptance enable
  	/* Write to the Local Mask field  */
  	ECanbLAMRegs.LAM6.bit.LAM_L=0xFFFF;
  	ECanbLAMRegs.LAM6.bit.LAM_H=0x9EFF;			// Filter for system controller Host-Interface
  	ECanbLAMRegs.LAM6.bit.LAMI=1;

  	ECanShadow.CANMD.all = ECanbRegs.CANMD.all;	// Mailbox direction
  	ECanShadow.CANMD.bit.MD1 = 1;          		// receive
  	ECanShadow.CANMD.bit.MD6 = 1;          		// receive
  	ECanbRegs.CANMD.all = ECanShadow.CANMD.all;	//

  	ECanShadow.CANME.all = ECanbRegs.CANME.all;	// Mailbox Enable Register
  	ECanShadow.CANME.bit.ME1 = 1;          		// enable Mailbox #1
  	ECanShadow.CANME.bit.ME6 = 1;          		// enable Mailbox #6
	ECanbRegs.CANME.all = ECanShadow.CANME.all;	//

    ECanbRegs.CANGAM.all = 0x9FFFFFFF; //  Identifier

	EDIS;

	//====== Initialize Variables ======
	eCAN_RxMessage.Data1 = 0;
	eCAN_RxMessage.Data2 = 0;
	eCAN_RxMessage.Data3 = 0;
	eCAN_RxMessage.Data4 = 0;
	eCAN_RxMessage.New_Message = FALSE;


}

//
//  ======== eCAN_communication ========
//  Data communication function
//
void eCAN_DataTx(ECAN_MESSAGE *eCAN_msg) {
	CAN_Tx_DelayCount = 0;
	//====== CAN ======
    /* Before change msg ID, Disable Mailbox for transmission */
    ECanShadow.CANME.all = ECanbRegs.CANME.all;
    ECanShadow.CANME.bit.ME25 = 0;
    ECanbRegs.CANME.all = ECanShadow.CANME.all;

    /* Write to the MSGID field  */
    ECanbMboxes.MBOX25.MSGID.all = eCAN_msg->eCAN_ID | 0x80000000;

    /* Enable Mailbox for transmission */
    ECanShadow.CANME.all = ECanbRegs.CANME.all;
    ECanShadow.CANME.bit.ME25 = 1;
    ECanbRegs.CANME.all = ECanShadow.CANME.all;

    /* Write to DLC field in Master Control reg */
    ECanbMboxes.MBOX25.MSGCTRL.bit.DLC = eCAN_msg->Data_Length;

    /* Write to the mailbox RAM field */
    //ECanbMboxes.MBOX25.MDL.all = CAN_Data1;
    //ECanbMboxes.MBOX25.MDH.all = CAN_Data2;
    ECanbMboxes.MBOX25.MDL.word.LOW_WORD = eCAN_msg->Data1;
    ECanbMboxes.MBOX25.MDL.word.HI_WORD = eCAN_msg->Data2;
    ECanbMboxes.MBOX25.MDH.word.LOW_WORD = eCAN_msg->Data3;
    ECanbMboxes.MBOX25.MDH.word.HI_WORD = eCAN_msg->Data4;

    ECanShadow.CANTRS.all = 0;
    ECanShadow.CANTRS.bit.TRS25 = 1;            // Set TRS for transmit mailbox
    ECanbRegs.CANTRS.all = ECanShadow.CANTRS.all;

    do {  // Wait for TA5 bit to be set..
       	ECanShadow.CANTA.all = ECanbRegs.CANTA.all;
       	CAN_Tx_DelayCount ++ ;
    } while(ECanShadow.CANTA.bit.TA25 == 0 && CAN_Tx_DelayCount <= 5000);

    ECanShadow.CANTA.all = 0;
    ECanShadow.CANTA.bit.TA25 = 1;     	        // Clear TA5
    ECanbRegs.CANTA.all = ECanShadow.CANTA.all;

}

//
//  ======== ecan1intb_isr ========
//  eCANB interrupt level 1: mailbox #1 receive message
//
interrupt void ecan1intb_isr(void) {
	Swi_post(swi1);
	ECanbRegs.CANTA.all	= 0xFFFFFFFF;	/* Clear all TAn bits   */
	ECanbRegs.CANRMP.all = 0xFFFFFFFF;	/* Clear all RMPn bits  */
	ECanbRegs.CANGIF1.all = 0xFFFFFFFF; /* Clear interrupt flag */

  	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

//
//  ======== ecan0intb_isr ========
//  eCANB interrupt level 0: eCAN error
//
interrupt void ecan0intb_isr(void) {
  	PieCtrlRegs.PIEACK.bit.ACK9 = 1;
}

//
//  ======== eCAN_Rx_swi ========
//  Software interrupt function that to perform
//  eCAN Rx functions.
//
Void eCAN_Rx_swi(UArg arg) {
	//GpioDataRegs.GPATOGGLE.bit.GPIO10 = 1;

	eCAN_RxMessage.eCAN_ID = ECanbMboxes.MBOX6.MSGID.all & 0x1FFFFFFF;
	eCAN_RxMessage.Data1 = ECanbMboxes.MBOX6.MDL.word.LOW_WORD;
	eCAN_RxMessage.Data2 = ECanbMboxes.MBOX6.MDL.word.HI_WORD;
	eCAN_RxMessage.Data3 = ECanbMboxes.MBOX6.MDH.word.LOW_WORD;
	eCAN_RxMessage.Data4 = ECanbMboxes.MBOX6.MDH.word.HI_WORD;
	eCAN_RxMessage.New_Message = TRUE;

    Semaphore_post(systemSema);
    //Semaphore_post(commSema);
}



/************* End of File ***************/



