/**********************************************************************************
// File: Grid_Control.c
// Devices: TMS320C28346
//_________________________________________________________________________________
//
// Tecogen	Inc.                               				Copyright © 2013
//
// Authors:		Jian Wen
//
//
// Description:
// ------------
// This is the c file for micro-grid control and UL1741.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/9/13	|  J Wen 	| Original
**********************************************************************************/
#include <xdc/std.h>
#include <math.h>

#include "DSP28x_Project.h"     // Device Header-file and Examples Include File
#include "SystemControl.h"      // Header file for system control
#include "Grid_Control.h"		// Header file for grid control
#include "EPWM_control.h"       // Header file for ePWM module
#include "ECAP_VLine.h"     	// Module Include File
#include "eCAN_communication.h" // Header file for eCAN
#include "ADC.h"				// Header file for A to D converter	   

/****** Variables for ADC channels ******/
MICRO_GRID	MicroGrid;
PI_CONTROL	P_max_control;
PI_CONTROL	P_min_control;

Uint16 MG_Setting_Report;

/****** External variables ******/
extern OPERATION_DATA 	OperationData;
extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern ECAN_MESSAGE 	eCAN_TxMessage, eCAN_RxMessage;

/******* Function prototype *******/
void MicroGrid_init(void);
void MicroGrid_ctrl(void);
void Pwr_vs_Freq(void);
void Q_vs_Vout(void);
void MicroGrid_Setting_CAN(void);
void MicroGrid_Setting_Report(void);

extern void eCAN_DataTx(ECAN_MESSAGE *eCAN_msg);

/**********************************************************************
* Function: MicroGrid_init()
*
* Description: micro-grid control initialization.
**********************************************************************/
void MicroGrid_init(void) {
	MicroGrid.mP = P_VS_FREQ_MP;		
	MicroGrid.mQ = Q_VS_VOUT_MQ;		
	MicroGrid.P_max_setting = P_MAX;
	MicroGrid.P_min_setting = P_MIN;
	MicroGrid.Freq_Upper_Limit = FREQ_UPPER_LIMIT;	
	MicroGrid.Freq_Lower_Limit = FREQ_LOWER_LIMIT;	
	MicroGrid.P_cmd_init = PWR_CMD_INIT;		
	MicroGrid.P_cmd = PWR_CMD_INIT;
	MicroGrid.enable = FALSE;	
	
	P_max_control.err = 0.0;	
	P_max_control.int_old = 0.0;	
	P_max_control.integral = 0.0;
	P_max_control.Ki = 0.01;	
	P_max_control.Kp = 0.01;	

	P_min_control.err = 0.0;	
	P_min_control.int_old = 0.0;	
	P_min_control.integral = 0.0;
	P_min_control.Ki = 0.01;	
	P_min_control.Kp = 0.01;

	MicroGrid.P	= 0.0;
	MicroGrid.Q	= 0.0;
	MicroGrid.KVA = 0.0;
	
	MG_Setting_Report = TRUE;	
}
/**********************************************************************
* Function: MicroGrid_ctrl()
*
* Description: micro-grid control function.
**********************************************************************/
void MicroGrid_ctrl(void) {
	 
	//---------------------------------------------------------
	//	Active power calculation	 
	//	Power = E.Ds*I.Ds + E.Qs*I.Qs
	//  Reactive Power = E.Qs*I.Ds - E.Ds*I.Qs
	//  Add lowpass filter for P and Q
	MicroGrid.P = AC_Line_Measurement.V_Line_D * AC_Line_Measurement.I_Line_D +	AC_Line_Measurement.V_Line_Q * AC_Line_Measurement.I_Line_Q;
	MicroGrid.P *= POWER_SCALING_FACTOR;
	MicroGrid.Q = AC_Line_Measurement.V_Line_Q * AC_Line_Measurement.I_Line_D -	AC_Line_Measurement.V_Line_D * AC_Line_Measurement.I_Line_Q;
	MicroGrid.Q *= POWER_SCALING_FACTOR;

	//---------------------------------------------------------
	if (MicroGrid.enable) {
		Q_vs_Vout();
		Pwr_vs_Freq();
	} else {
		MicroGrid.Frequency = OperationData.Frequency;
		MicroGrid.V_cmd = OperationData.VoltageCommand;
	}
}
  

/**********************************************************************
* Function: Pwr_vs_Freq()
*
* Description: Power vs frequency control.
**********************************************************************/
void Pwr_vs_Freq(void){
int	iTemp;
int iP_measure;
float fTemp;

	//---- Power vs Frequency output ----
	// mP: Power vs Frequency rate	(Hz/KW)
	// Delta Frequency = mP * (PowerCommand - Power)
	MicroGrid.Freq_Delta = MicroGrid.mP * ((float)(MicroGrid.P_cmd)/10.0 - MicroGrid.P);

	//---- Pmax Control ----
	P_max_control.err = MicroGrid.P_max_setting - MicroGrid.P;
	P_max_control.integral = P_max_control.Ki * P_max_control.err;
	P_max_control.integral += P_max_control.int_old;

	iTemp = (int)(P_max_control.integral * 10.0);
	if (iTemp > 0)		P_max_control.integral = 0.0; 
	if (iTemp < -5000)	P_max_control.integral = -500.0;
	P_max_control.int_old = P_max_control.integral;

	iP_measure = (int)( MicroGrid.P * 10);
	if (iP_measure > MicroGrid.P_max_setting) {
		MicroGrid.PvsPmax_Delta = P_max_control.Kp * P_max_control.err;
		MicroGrid.PvsPmax_Delta += P_max_control.integral;
	} else {
		MicroGrid.PvsPmax_Delta = 0.0;;
	}
	 
	iTemp = (int)(MicroGrid.PvsPmax_Delta * 100.0);
	if (iTemp >= 300) {
		MicroGrid.PvsPmax_Delta = 3.0;		   // Allow maximum frequency droop 3.0Hz. 
	}

	//---- Pmin Control ----
	P_min_control.err = MicroGrid.P_min_setting - MicroGrid.P;
	P_min_control.integral = P_min_control.Ki * P_min_control.err;
	P_min_control.integral += P_min_control.int_old;

	iTemp = (int)(P_min_control.integral * 10.0);
	if (iTemp < 0)		P_min_control.integral = 0.0; 
	if (iTemp > 5000)	P_min_control.integral = 500.0;
	P_min_control.int_old = P_min_control.integral;

	iP_measure = (int)( MicroGrid.P * 10);
	if (iP_measure < MicroGrid.P_min_setting) {
		MicroGrid.PvsPmin_Delta = P_min_control.Kp * P_min_control.err;
		MicroGrid.PvsPmin_Delta += P_min_control.integral;
	} else {
		MicroGrid.PvsPmin_Delta = 0.0;;
	}
	 
	iTemp = (int)(MicroGrid.PvsPmin_Delta * 100.0);
	if (iTemp >= 300) {
		MicroGrid.PvsPmin_Delta = 3.0;		   // Allow maximum frequency increase 3.0Hz. 
	}
	//-----------------------*/

	//------ Frequency output --------
	MicroGrid.Frequency = OperationData.Frequency - MicroGrid.Freq_Delta;
	MicroGrid.Frequency -= MicroGrid.PvsPmax_Delta;
	MicroGrid.Frequency += MicroGrid.PvsPmin_Delta;
	//--- Modify sampling frequency ---
	OperationData.PWM_Frequency = OperationData.Period_Cnt * MicroGrid.Frequency;
	fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
	OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
	//OperationData.PWM_Period_Reg = 14881;
	EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
	EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
	EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update  ****/
}

/**********************************************************************
* Function: Q_vs_Vout()
*
* Description: Reactive power Q vs output voltage control.
**********************************************************************/
void Q_vs_Vout(void){
int iTemp;	
	//---- V out command vs Q ----
	// mQ: Q vs output Voltage command (V/KVA)
	// E command: Norminal output voltage command
	// V out command = E command - mQ * Q;
	
	//MicroGrid.V_cmd = OperationData.VoltageCommand;
	MicroGrid.V_cmd = OperationData.VoltageCommand - MicroGrid.mQ * MicroGrid.Q;
	
	// Clamp V out command to Max or Min
	iTemp = (int)(MicroGrid.V_cmd * 10.0);
	if (iTemp > E0_MAX) {
		MicroGrid.V_cmd = E0_MAX/10.0;
	}
	if (iTemp < E0_MIN) {
		MicroGrid.V_cmd = E0_MIN/10.0;
	}
	// Use MicroGrid.V_cmd as for voltage regulater command, replace OperationData.VoltageCommand

	 	     
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	MicroGrid_Setting_CAN							
//_________________________________________________________________________________
void MicroGrid_Setting_CAN(){
int iTempData;

	switch (eCAN_RxMessage.eCAN_ID)	{
		case eCANMsgID_FrequencyLimit_Setting:				// Frequency upper and lower limit seeting
			iTempData = eCAN_RxMessage.Data1;				// 0.01Hz/bit
			if ( iTempData > 500)   iTempData = 500;		// 10Hz as limit, Min Freq = 50Hz
			MicroGrid.Freq_Lower_Limit = iTempData;
													  
			iTempData = eCAN_RxMessage.Data2;		  		// 5Hz as limit, Max Freq = 65Hz
			if ( iTempData > 1000)   iTempData = 1000;		
			MicroGrid.Freq_Upper_Limit = iTempData;

			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_FrequencyLimit_Setting_Echo;
			eCAN_TxMessage.Data_Length = 4;
			eCAN_TxMessage.Data1 = MicroGrid.Freq_Lower_Limit;
			eCAN_TxMessage.Data2 = MicroGrid.Freq_Upper_Limit;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_Power_Setting:						// Frequency upper and lower limit seeting
			iTempData = eCAN_RxMessage.Data1;				// 0.1KW/bit
			if ( iTempData > 1250)   iTempData = 1250;		// 125KW as Max
			MicroGrid.P_cmd = iTempData;
													  
			iTempData = eCAN_RxMessage.Data2;		  
			if ( iTempData > 1000)   iTempData = 1000;
			MicroGrid.P_cmd_init = iTempData;

			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_Power_Setting_Echo;
			eCAN_TxMessage.Data_Length = 4;
			eCAN_TxMessage.Data1 = MicroGrid.P_cmd;
			eCAN_TxMessage.Data2 = MicroGrid.P_cmd_init;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_P_vs_Freq_Setting:				// Frequency upper and lower limit seeting
			iTempData = eCAN_RxMessage.Data1;			// Hz/Kw .... >
			MicroGrid.mP = (float)iTempData * 0.0001;	// default value = -0.005
													  
			iTempData = eCAN_RxMessage.Data2;		  
			if ( iTempData > 1250)   iTempData = 1250;
			MicroGrid.P_max_setting = iTempData;

			iTempData = eCAN_RxMessage.Data3;		  
			if ( iTempData > 1250)   iTempData = 1250;
			MicroGrid.P_min_setting = iTempData;

			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_P_vs_Freq_Setting_Echo;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)(MicroGrid.mP * 10000);
			eCAN_TxMessage.Data2 = MicroGrid.P_max_setting;
			eCAN_TxMessage.Data3 = MicroGrid.P_min_setting;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_Q_vs_Vout_Setting:				// Frequency upper and lower limit seeting
			iTempData = eCAN_RxMessage.Data1;			// V/KBVA
			MicroGrid.mQ = (float)iTempData * 0.01;
													  
			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_Q_vs_Vout_Setting_Echo;
			eCAN_TxMessage.Data_Length = 2;
			eCAN_TxMessage.Data1 = (int)(MicroGrid.mQ * 100);
			eCAN_DataTx(&eCAN_TxMessage);

		break;
		
		default:
		break;
	}
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	MicroGrid_Setting_Report							
//_________________________________________________________________________________
void MicroGrid_Setting_Report(){
static int MGReportCount=0;

	switch (MGReportCount) {
		case 0:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_FrequencyLimit_Setting_Echo;
			eCAN_TxMessage.Data_Length = 4;
			eCAN_TxMessage.Data1 = MicroGrid.Freq_Lower_Limit;
			eCAN_TxMessage.Data2 = MicroGrid.Freq_Upper_Limit;
			eCAN_DataTx(&eCAN_TxMessage);
			MGReportCount = 1;
		break;

		case 1:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_Power_Setting_Echo;
			eCAN_TxMessage.Data_Length = 4;
			eCAN_TxMessage.Data1 = MicroGrid.P_cmd;
			eCAN_TxMessage.Data2 = MicroGrid.P_cmd_init;
			eCAN_DataTx(&eCAN_TxMessage);
			MGReportCount = 2;
		break;

		case 2:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_P_vs_Freq_Setting_Echo;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)(MicroGrid.mP * 10000);
			eCAN_TxMessage.Data2 = MicroGrid.P_max_setting;
			eCAN_TxMessage.Data3 = MicroGrid.P_min_setting;
			eCAN_DataTx(&eCAN_TxMessage);
			MGReportCount = 3;
		break;

		case 3:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_Q_vs_Vout_Setting_Echo;
			eCAN_TxMessage.Data_Length = 2;
			eCAN_TxMessage.Data1 = (int)(MicroGrid.mQ * 100);
			eCAN_DataTx(&eCAN_TxMessage);
			MGReportCount = 0;
			MG_Setting_Report = FALSE;
		break;


		default:	// unknown status
			// do nothing
			MGReportCount = 0;
		break;
	}
}

/*** end of file *****************************************************/
