/**********************************************************************************
// File: fpga.c
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
// This is the c file to access FPGA registers location.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/14/12	|  J Wen 	| Original
**********************************************************************************/
#include "DSP28x_Project.h"     // Device Header-file and Examples Include File

#include "SystemControl.h"      // Header file for system control
#include "Fpga.h"

unsigned int FPGA_r_data[16];
FPGA_REGWR_IMAGE FPGA_RegWR_Image;

unsigned long zero_crossing[3];
unsigned long Mag_pickup_sensing;
float	Frequency_fpga_ch[3];
float   Mag_pickup_freq;
float	Engine_Speed_RPM;

void FPGA_Write(void);
void FPGA_Read(void);
void MAG_Sensor_Read(void);


/**********************************************************************
// Function: FPGA_Write()
// Description: Write FPGA data.
// Revisions:
// ----------
**********************************************************************/
void FPGA_Write() {
static Uint16 Testdata=0;

	Testdata++;

	//pFPGA_RegWR->Data_1= 0;
	//pFPGA_RegWR->Data_2= 1;
	pFPGA_RegWR->Data_3= 0xaaaa;
	pFPGA_RegWR->Data_4= 0x7711;
	pFPGA_RegWR->Data_5= 0x1177;
	pFPGA_RegWR->Data_6= 0x2266;
	pFPGA_RegWR->Data_7= 0x6622;
	pFPGA_RegWR->Data_8= 0x8888;
	pFPGA_RegWR->Data_9= 0x1111;
	pFPGA_RegWR->Data_10= 0x2222;
	pFPGA_RegWR->Data_11= 0x3333;
	pFPGA_RegWR->Data_12= 0x4444;		
	pFPGA_RegWR->Data_13= 0x5555;
	pFPGA_RegWR->Data_14= 0x6666;
	pFPGA_RegWR->Data_15= 0x7777;
	pFPGA_RegWR->Data_16= 0xffff;

}


/**********************************************************************
* Function: FPGA_Read()
*
* Description: Read FPGA data.
// Revisions:
// ----------
**********************************************************************/
void FPGA_Read()	{
static Uint16 TestdataR;
	TestdataR++;

    FPGA_r_data[0]= pFPGA_RegRD->Data_1;
	FPGA_r_data[1]= pFPGA_RegRD->Data_2;		
	FPGA_r_data[2]= pFPGA_RegRD->Data_3;
	FPGA_r_data[3]= pFPGA_RegRD->Data_4;
	FPGA_r_data[4]= pFPGA_RegRD->Data_5;
	FPGA_r_data[5]= pFPGA_RegRD->Data_6;
	FPGA_r_data[6]= pFPGA_RegRD->Data_7;
	FPGA_r_data[7]= pFPGA_RegRD->Data_8;
	FPGA_r_data[8]= pFPGA_RegRD->Data_9;
	FPGA_r_data[9]= pFPGA_RegRD->Data_10;
	FPGA_r_data[10]= pFPGA_RegRD->Data_11;
	FPGA_r_data[11]= pFPGA_RegRD->Data_12;
	FPGA_r_data[12]= pFPGA_RegRD->Data_13;
	FPGA_r_data[13]= pFPGA_RegRD->Data_14;
	FPGA_r_data[14]= pFPGA_RegRD->Data_15;
	FPGA_r_data[15]= pFPGA_RegRD->Data_16;

	//zero_crossing[0] = (unsigned long)FPGA_r_data[9] + (unsigned long)FPGA_r_data[8]<<16;
	//zero_crossing[1] = (unsigned long)FPGA_r_data[11] + (unsigned long)FPGA_r_data[10]<<16;
	//zero_crossing[2] = (unsigned long)FPGA_r_data[13] + (unsigned long)FPGA_r_data[2]<<16;
	zero_crossing[0] = (unsigned long)FPGA_r_data[8] + ((unsigned long)FPGA_r_data[9]<<16);
	zero_crossing[1] = (unsigned long)FPGA_r_data[10] + ((unsigned long)FPGA_r_data[11]<<16);
	zero_crossing[2] = (unsigned long)FPGA_r_data[12] + ((unsigned long)FPGA_r_data[13]<<16);
	// FPGA clock = 37.5MHz
	Frequency_fpga_ch[0] = 37500000.0 / (float)zero_crossing[0];
	Frequency_fpga_ch[1] = 37500000.0 / (float)zero_crossing[1];
	Frequency_fpga_ch[2] = 37500000.0 / (float)zero_crossing[2];

	// For test
//	if (FPGA_r_data[1] != 7) {
//		FPGA_r_data[10] += 1;
//	}
//	if (FPGA_r_data[2] != 5) {
//		FPGA_r_data[11] += 1;
//	}
//	if (FPGA_r_data[3] != 2013) {
//		FPGA_r_data[12] += 1;
//	}

}

/**********************************************************************
* Function: MAG_Sensor_Read()
*
* Description: Read Mag sensor count every 5 msec from line monitor task.
* Revisions:
* ----------
**********************************************************************/
void MAG_Sensor_Read()	{

	FPGA_r_data[14]= pFPGA_RegRD->Data_15;
	FPGA_r_data[15]= pFPGA_RegRD->Data_16;

	Mag_pickup_sensing = (unsigned long)FPGA_r_data[14] + ((unsigned long)FPGA_r_data[15]<<16);
	Mag_pickup_freq	= 32 * 37500000.0 / (float)Mag_pickup_sensing;
	if ((int) Mag_pickup_freq < 20 ) Mag_pickup_freq = 0.0;
	//Engine_Speed_RPM = 0.05 * (Mag_pickup_freq / 2.8) + 0.95 * Engine_Speed_RPM;		// one mechanical cycle = 168 sensing cycle 168/60 = 2.8 
	Engine_Speed_RPM = 0.1 * (Mag_pickup_freq / 2.8) + 0.9 * Engine_Speed_RPM;		// one mechanical cycle = 168 sensing cycle 168/60 = 2.8 
}	
/*** end of file *****************************************************/
