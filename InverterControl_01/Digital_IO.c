//_________________________________________________________________________________
//
// Tecogen	Inc.                               				Copyright © 2012
//
// Authors:		Jian Wen
//
//
// Description:
// ------------
// This is the c file to access FPGA registers location.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/14/12	|  J Wen 	| Original
//_________________________________________________________________________________
#include <xdc/std.h>

#include "DSP28x_Project.h"     // Device Header-file and Examples Include File
#include "SystemControl.h"      // Header file for system control
#include "Fpga.h"
#include "Digital_IO.h"

//====== Global variables used in this module ======
union DIGITAL_IO Digital_IO;
int	DI_1_EMS_count;
int	DI_2_count;
int	DI_3_count;
int	DI_4_count;
int	DI_5_count;
int	DI_6_count;
int	DI_7_count;
int	DI_8_count;
Uint16	Digital_In;
Uint16	Filter_Contactor;
Uint16	Output_Contactor;
int		Filter_Contact_Cnt;
int		Output_Contact_Cnt;

extern FPGA_REGWR_IMAGE FPGA_RegWR_Image;
extern SYSTEM_INFO system;
extern unsigned int FPGA_r_data[16];

//====== Function prototype ======
void Digital_IO_Init(void);
void Digital_Input(void);
void DO_1_On(void);
void DO_2_On(void);
void DO_3_On(void);
void DO_4_On(void);
void DO_5_On(void);
void DO_6_On(void);
void DO_7_On(void);
void DO_1_Off(void);
void DO_2_Off(void);
void DO_3_Off(void);
void DO_4_Off(void);
void DO_5_Off(void);
void DO_6_Off(void);
void DO_7_Off(void);


/**********************************************************************
// Function: Digital_IO_Init()
// Description: Digital IO initialization.
// Revisions:
// ----------
**********************************************************************/
void Digital_IO_Init() {
	Digital_IO.all = 0;
	DI_1_EMS_count = 0;
	DI_2_count = 0;
	DI_3_count = 0;
	DI_4_count = 0;
	DI_5_count = 0;
	DI_6_count = 0;
	DI_7_count = 0;
	DI_8_count = 0;

	Filter_Contactor = CONTACTOR_OPEN;
	Output_Contactor = CONTACTOR_OPEN;
	Filter_Contact_Cnt = 0;
	Output_Contact_Cnt = 0;
}

/**********************************************************************
// Function: Digital_Input()
// Description: Digital input with de-bounce function.
// Revisions:
// ----------
**********************************************************************/
void Digital_Input() {
	// Digital inputs:
	//**** DI_1_EMS ****
	Digital_In = GpioDataRegs.GPADAT.bit.GPIO22;
	if (Digital_In == 0) {
		DI_1_EMS_count++;
		if (DI_1_EMS_count >= 3) {
			DI_1_EMS_count = 3;
			Digital_IO.bit.DI1_EMS = 1;
			system.EMS = FALSE;
		}
	} else {
		DI_1_EMS_count--;
		if (DI_1_EMS_count <= 0) {
			DI_1_EMS_count = 0;
			Digital_IO.bit.DI1_EMS = 0;
			system.EMS = TRUE;
		}
	} 

	//Digital_IO.bit.DI2 = ~GpioDataRegs.GPADAT.bit.GPIO23;
	//**** DI_2 ****
	Digital_In = GpioDataRegs.GPADAT.bit.GPIO23;
	if (Digital_In == 0) {
		DI_2_count++;
		if (DI_2_count >= 3) {
			DI_2_count = 3;
			Digital_IO.bit.DI2 = 1;
		}
	} else {
		DI_2_count--;
		if (DI_2_count <= 0) {
			DI_2_count = 0;
			Digital_IO.bit.DI2 = 0;
		}
	}
	
	//Digital_IO.bit.DI3 = FPGA_r_data[4] & 0x0001;	// No use
	//**** DI_3 ****
	Digital_In = FPGA_r_data[4] & 0x0001;
	if (Digital_In == 1) {
		DI_3_count++;
		if (DI_3_count >= 3) {
			DI_3_count = 3;
			Digital_IO.bit.DI3 = 1;
		}
	} else {
		DI_3_count--;
		if (DI_3_count <= 0) {
			DI_3_count = 0;
			Digital_IO.bit.DI3 = 0;
		}
	}

	//Digital_IO.bit.DI4 = ~GpioDataRegs.GPADAT.bit.GPIO14;
	//**** DI_4 ****
	Digital_In = GpioDataRegs.GPADAT.bit.GPIO14;
	if (Digital_In == 0) {
		DI_4_count++;
		if (DI_4_count >= 3) {
			DI_4_count = 3;
			Digital_IO.bit.DI4 = 1;
		}
	} else {
		DI_4_count--;
		if (DI_4_count <= 0) {
			DI_4_count = 0;
			Digital_IO.bit.DI4 = 0;
		}
	}

	//Digital_IO.bit.DI5 = ~GpioDataRegs.GPBDAT.bit.GPIO52;	  
	//**** DI_5 ****		// filter contactor aux			  
	Digital_In = GpioDataRegs.GPBDAT.bit.GPIO52;
	if (Digital_In == 0) {
		DI_5_count++;
		if (DI_5_count >= 3) {
			DI_5_count = 3;
			Digital_IO.bit.DI5 = 1;
		}
	} else {
		DI_5_count--;
		if (DI_5_count <= 0) {
			DI_5_count = 0;
			Digital_IO.bit.DI5 = 0;
		}
	}
	if (Filter_Contactor == CONTACTOR_OPEN) {
		Filter_Contact_Cnt ++;
		if (Filter_Contact_Cnt >= 6) {
			Filter_Contact_Cnt = 6;
			//if (Digital_IO.bit.DI5 == 1 && system.state != DISCHARGE){
			if (Digital_IO.bit.DI5 == 1){
				// Contactor fault
				system.SW_faults |= FILTER_CONTACTER_FAULT;
			}
		}
	} else {
		Filter_Contact_Cnt --;
		if (Filter_Contact_Cnt <= 0) {
			Filter_Contact_Cnt = 0;
			//if (Digital_IO.bit.DI5 == 0 && system.state != DISCHARGE){
			if (Digital_IO.bit.DI5 == 0){
				// Contactor fault
				system.SW_faults |= FILTER_CONTACTER_FAULT;
			}
		}
	}
	//Digital_IO.bit.DI6 = ~GpioDataRegs.GPBDAT.bit.GPIO53;
	//**** DI_6 ****		// output contactor aux	   
	Digital_In = GpioDataRegs.GPBDAT.bit.GPIO53;
	if (Digital_In == 0) {
		DI_6_count++;
		if (DI_6_count >= 3) {
			DI_6_count = 3;
			Digital_IO.bit.DI6 = 1;
		}
	} else {
		DI_6_count--;
		if (DI_6_count <= 0) {
			DI_6_count = 0;
			Digital_IO.bit.DI6 = 0;
		}
	}
	if (Output_Contactor == CONTACTOR_OPEN){
		Output_Contact_Cnt ++;
		if (Output_Contact_Cnt >= 10) {
			Output_Contact_Cnt = 10;
			if (Digital_IO.bit.DI6 == 1){
				// Contactor fault
				system.SW_faults |= OUTPUT_CONTACTER_FAULT;
			}
		}
	} else {
		Output_Contact_Cnt --;
		if (Output_Contact_Cnt <= 0) {
			Output_Contact_Cnt = 0;
			if (Digital_IO.bit.DI6 == 0){
				// Contactor fault
				system.SW_faults |= OUTPUT_CONTACTER_FAULT;
			}
		}
	}
	 
	//Digital_IO.bit.DI2 = ~GpioDataRegs.GPADAT.bit.GPIO23;
	//Digital_IO.bit.DI3 = FPGA_r_data[4] & 0x0001;
	//Digital_IO.bit.DI4 = ~GpioDataRegs.GPADAT.bit.GPIO14;
	//Digital_IO.bit.DI5 = ~GpioDataRegs.GPBDAT.bit.GPIO52;	// Di_5: filter contactor aux
	//Digital_IO.bit.DI6 = ~GpioDataRegs.GPBDAT.bit.GPIO53;	// Di_6: output contactor aux
	Digital_IO.bit.DI7 = ~GpioDataRegs.GPBDAT.bit.GPIO58;
	Digital_IO.bit.DI8 = ~GpioDataRegs.GPBDAT.bit.GPIO59;
}

/**********************************************************************
// Function: DO_1_On()
// Description: DO_1 On, active LOW.
// Revisions:
// ----------
**********************************************************************/
void DO_1_On() {
	GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;
	Digital_IO.bit.DO1 = 1;
}

/**********************************************************************
// Function: DO_1_Off()
// Description: DO_1 Off, set high.
// Revisions:
// ----------
**********************************************************************/
void DO_1_Off() {
	GpioDataRegs.GPBSET.bit.GPIO63 = 1;
	Digital_IO.bit.DO1 = 0;
}

/**********************************************************************
// Function: DO_2_On()
// Description: DO_2 On, active LOW.
// Revisions:
// ----------
**********************************************************************/
void DO_2_On() {
	GpioDataRegs.GPBCLEAR.bit.GPIO62 = 1;
	Digital_IO.bit.DO2 = 1;
}

/**********************************************************************
// Function: DO_2_Off()
// Description: DO_2 Off, set high.
// Revisions:
// ----------
**********************************************************************/
void DO_2_Off() {
	GpioDataRegs.GPBSET.bit.GPIO62 = 1;
	Digital_IO.bit.DO2 = 0;
}

/**********************************************************************
// Function: DO_3_On()
// Description: DO_3 On, active LOW.   Control filter contactor
// Revisions:
// ----------
**********************************************************************/
void DO_3_On() {
	GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;
	Digital_IO.bit.DO3 = 1;
	//Filter_Contactor = CONTACTOR_CLOSE;
}

/**********************************************************************
// Function: DO_3_Off()
// Description: DO_3 Off, set high.
// Revisions:
// ----------
**********************************************************************/
void DO_3_Off() {
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;
	Digital_IO.bit.DO3 = 0;
	//Filter_Contactor = CONTACTOR_OPEN;
}

/**********************************************************************
// Function: DO_4_On()
// Description: DO_4 On, active LOW.  Control output contactor
// Revisions:
// ----------
**********************************************************************/
void DO_4_On() {
	GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;
	Digital_IO.bit.DO4 = 1;
	//Output_Contactor = CONTACTOR_CLOSE;
}

/**********************************************************************
// Function: DO_4_Off()
// Description: DO_4 Off, set high.
// Revisions:
// ----------
**********************************************************************/
void DO_4_Off() {
	GpioDataRegs.GPBSET.bit.GPIO60 = 1;
	Digital_IO.bit.DO4 = 0;
	//Output_Contactor = CONTACTOR_OPEN;
}

/**********************************************************************
// Function: DO_5_On()
// Description: DO_5 On, active high.
// Revisions:
// ----------
**********************************************************************/
void DO_5_On() {
	FPGA_RegWR_Image.Data_1 |= DO_5;		// to FPGA
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO5 = 1;
}

/**********************************************************************
// Function: DO_5_Off()
// Description: DO_5 Off, set low.
// Revisions:
// ----------
**********************************************************************/
void DO_5_Off() {
	FPGA_RegWR_Image.Data_1 &= ~DO_5;
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO5 = 0;
}

/**********************************************************************
// Function: DO_6_On()
// Description: DO_6 On, active high.
// Revisions:
// ----------
**********************************************************************/
void DO_6_On() {
	FPGA_RegWR_Image.Data_1 |= DO_6;		// to FPGA
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO6 = 1;
	Filter_Contactor = CONTACTOR_CLOSE;
}

/**********************************************************************
// Function: DO_6_Off()
// Description: DO_6 Off, set low.
// Revisions:
// ----------
**********************************************************************/
void DO_6_Off() {
	FPGA_RegWR_Image.Data_1 &= ~DO_6;
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO6 = 0;
	Filter_Contactor = CONTACTOR_OPEN;
}

/**********************************************************************
// Function: DO_7_On()
// Description: DO_7 On, active high.
// Revisions:
// ----------
**********************************************************************/
void DO_7_On() {
	FPGA_RegWR_Image.Data_1 |= DO_7;		// to FPGA
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO7 = 1;
	Output_Contactor = CONTACTOR_CLOSE;
}

/**********************************************************************
// Function: DO_7_Off()
// Description: DO_7 Off, set low.
// Revisions:
// ----------
**********************************************************************/
void DO_7_Off() {
	FPGA_RegWR_Image.Data_1 &= ~DO_7;
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	Digital_IO.bit.DO7 = 0;
	Output_Contactor = CONTACTOR_OPEN;
}

/*** end of file *****************************************************/
