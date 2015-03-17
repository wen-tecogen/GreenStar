/**********************************************************************************
// File: UL1741.c
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

#include "DSP28x_Project.h"     // Device Header file and Examples Include File
#include "SystemControl.h"      // Header file for system control
#include "Grid_Control.h"		// Header file for grid control
#include "UL1741.h"				// Header file for UL1741
#include "EPWM_control.h"       // Header file for ePWM module
#include "eCAN_communication.h" // Header file for eCAN
#include "ADC.h"				// Header file for A to D converter
#include "ECAP_VLine.h"     	// Module Include File

// UL 1741 variables:
UL1741_SETTING	UL1741_Setting;
TRIP_TIME_CNT	VF_Trip;
Uint16	UL1741_report;
Uint16	ACLine_Test;

extern OPERATION_DATA 	OperationData;
extern ECAN_MESSAGE 	eCAN_TxMessage, eCAN_RxMessage;
extern SYSTEM_INFO 	system;
extern GRID_TIE	iGrid_Tie;
extern ANTI_ISLANDING	Anti_Islanding;

extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern AC_LINE_DATA	ACLine;

// UL 1741 functions:
void	UL1741_Setting_Init(void);
void 	UL1741_Setting_CAN(void);
void 	UL1741_Setting_Report(void);
void 	UL1741_Line_Monitor(void);

extern void eCAN_DataTx(ECAN_MESSAGE *eCAN_msg);
extern void DO_7_Off(void);
void Boost_Off(void);
void Boost2_Off(void);
void Inverter_Off(void);



/**********************************************************************
// Function: UL1741_Setting_Init()
// Description: initialize voltage and frequency upper and lower limit.
// Revisions:
// ----------
**********************************************************************/
void UL1741_Setting_Init() {
	UL1741_Setting.V_Upper_Limit_1 = (int)(OperationData.VoltageCommand * 1.10);
	UL1741_Setting.V_Upper_Limit_2 = (int)(OperationData.VoltageCommand * 1.2);
	UL1741_Setting.V_Lower_Limit_1 = (int)(OperationData.VoltageCommand * 0.88);
	UL1741_Setting.V_Lower_Limit_2 = (int)(OperationData.VoltageCommand * 0.5);

	UL1741_Setting.V_Upper_Limit_1_Trip_Time = 198;		// 1 sec		Note: trip time 5msec/bit
	UL1741_Setting.V_Upper_Limit_2_Trip_Time = 30;		// 160 msec
	UL1741_Setting.V_Lower_Limit_1_Trip_Time = 398;		// 2 sec
	UL1741_Setting.V_Lower_Limit_2_Trip_Time = 30;		// 160 msec

	UL1741_Setting.F_Upper_Limit_1 = (int)(OperationData.Frequency*100 + 50);	 	//60.5Hz
	UL1741_Setting.F_Upper_Limit_2 = (int)(OperationData.Frequency*100 + 100); 		//61.0Hz
	UL1741_Setting.F_Lower_Limit_1 = (int)(OperationData.Frequency*100 - 70);		//59.3Hz
	UL1741_Setting.F_Lower_Limit_2 = (int)(OperationData.Frequency*100 - 300);		//57.0Hz

	UL1741_Setting.F_Upper_Limit_1_Trip_Time = 30;		// 160 msec
	UL1741_Setting.F_Upper_Limit_2_Trip_Time = 30;		// 160 msec
	UL1741_Setting.F_Lower_Limit_1_Trip_Time = 58;		// 300 msec
	UL1741_Setting.F_Lower_Limit_2_Trip_Time = 30;		// 160 msec

	UL1741_Setting.GT_ReconnectTime = 300;				// 300 sec		will not implement
	UL1741_report = TRUE;

	//----- Trip time counter init -----
	VF_Trip.F_Upper_Limit1_cnt = 0;
	VF_Trip.F_Upper_Limit2_cnt = 0;
	VF_Trip.F_Lower_Limit1_cnt = 0;
	VF_Trip.F_Lower_Limit2_cnt = 0;

	VF_Trip.V_Upper_Limit1_cnt = 0;
	VF_Trip.V_Upper_Limit2_cnt = 0;
	VF_Trip.V_Lower_Limit1_cnt = 0;
	VF_Trip.V_Lower_Limit2_cnt = 0;

	iGrid_Tie.Trip = 0;
}


//_________________________________________________________________________________
//
// Type:	function
// Name:	UL1741_Setting_CAN							
//_________________________________________________________________________________
void UL1741_Setting_CAN(){
Uint16 iTempData;

	switch (eCAN_RxMessage.eCAN_ID)	{
		case eCANMsgID_OverVoltage_Setting:					// Over voltage 1 and 2 seeting
			iTempData = eCAN_RxMessage.Data1;				// 1V/bit
			if ( iTempData > (unsigned int)(OperationData.VoltageCommand * 1.3))   iTempData = (int)(OperationData.VoltageCommand * 1.3);
			if ( iTempData < (unsigned int)(OperationData.VoltageCommand * 1.05))  iTempData = (int)(OperationData.VoltageCommand * 1.05);
			UL1741_Setting.V_Upper_Limit_1 = iTempData;		 				

			iTempData = eCAN_RxMessage.Data2;
			if ( iTempData > (unsigned int)(OperationData.VoltageCommand * 1.3))   iTempData = (int)(OperationData.VoltageCommand * 1.3);
			if ( iTempData < (unsigned int)(OperationData.VoltageCommand * 1.05))  iTempData = (int)(OperationData.VoltageCommand * 1.05);
			UL1741_Setting.V_Upper_Limit_2 = iTempData;

			iTempData = eCAN_RxMessage.Data3;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.V_Upper_Limit_1_Trip_Time = iTempData/5 -2;	// 1ms per unit

			iTempData = eCAN_RxMessage.Data4;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.V_Upper_Limit_2_Trip_Time = iTempData/5 -2;	 // 

			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_OverVoltage_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.V_Upper_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.V_Upper_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.V_Upper_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.V_Upper_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_UnderVoltage_Setting:				// Under voltage 1 and 2 seeting
			iTempData = eCAN_RxMessage.Data1;				// 1V/bit
			if ( iTempData > (int)(OperationData.VoltageCommand * 0.95))  iTempData = (int)(OperationData.VoltageCommand * 0.95);
			if ( iTempData < (int)(OperationData.VoltageCommand * 0.5))   iTempData = (int)(OperationData.VoltageCommand * 0.5);
			UL1741_Setting.V_Lower_Limit_1 = iTempData;		 				

			iTempData = eCAN_RxMessage.Data2;
			if ( iTempData > (int)(OperationData.VoltageCommand * 0.95))  iTempData = (int)(OperationData.VoltageCommand * 0.95);
			if ( iTempData < (int)(OperationData.VoltageCommand * 0.5))   iTempData = (int)(OperationData.VoltageCommand * 0.5);
			UL1741_Setting.V_Lower_Limit_2 = iTempData;

			iTempData = eCAN_RxMessage.Data3;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.V_Lower_Limit_1_Trip_Time = iTempData/5 - 2;	// 1ms per unit

			iTempData = eCAN_RxMessage.Data4;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.V_Lower_Limit_2_Trip_Time = iTempData/5 - 2;	 // 

			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_UnderVoltage_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.V_Lower_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.V_Lower_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.V_Lower_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.V_Lower_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_OverFrequency_Setting:				// Over Frequency 1 and 2 seeting
		 	iTempData = eCAN_RxMessage.Data1;				// 0.1Hz/bit
		 	if (iTempData > (int)(OperationData.Frequency*100 + 100))  iTempData = (int)(OperationData.Frequency*100 + 100);		// Max: 61Hz
			if (iTempData < (int)(OperationData.Frequency*100 + 50))   iTempData = (int)(OperationData.Frequency*100 + 50);		// Normal 60.5Hz
		 	UL1741_Setting.F_Upper_Limit_1 = iTempData;		// 600 -> 60.00Hz, use fix point here for comparas		
			
		 	iTempData = eCAN_RxMessage.Data2;				// Frequency Protection
		 	if (iTempData > (int)(OperationData.Frequency*100 + 100))  iTempData = (int)(OperationData.Frequency*100 + 100);		// Max: 61Hz
			if (iTempData < (int)(OperationData.Frequency*100 + 50))   iTempData = (int)(OperationData.Frequency*100 + 50);		// Normal 60.5Hz
		 	UL1741_Setting.F_Upper_Limit_2 = iTempData;		// 600 -> 60.00Hz, use fix point here for comparas		
		 		
			iTempData = eCAN_RxMessage.Data3;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.F_Upper_Limit_1_Trip_Time = iTempData/5 - 2;	// 1ms per unit

			iTempData = eCAN_RxMessage.Data4;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.F_Upper_Limit_2_Trip_Time = iTempData/5 - 2;	 // 
		
			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_OverFrequency_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.F_Upper_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.F_Upper_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.F_Upper_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.F_Upper_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_UnderFrequency_Setting:				// Under Frequency 1 and 2 seeting
		 	iTempData = eCAN_RxMessage.Data1;				// 0.1Hz/bit
		 	if (iTempData > (int)(OperationData.Frequency*100 - 50))  iTempData = (int)(OperationData.Frequency*100 - 50);		// Max: 59.5Hz
			if (iTempData < (int)(OperationData.Frequency*100 - 300))   iTempData = (int)(OperationData.Frequency*100 - 300);	// Min: 57.0Hz
		 	UL1741_Setting.F_Lower_Limit_1 = iTempData;		// 600 -> 60.00Hz, use fix point here for comparas		
			
		 	iTempData = eCAN_RxMessage.Data2;				// Frequency Protection
		 	if ( iTempData > (int)(OperationData.Frequency*100 - 50))  iTempData = (int)(OperationData.Frequency*100 - 50);		// Max: 59.5Hz
			if ( iTempData < (int)(OperationData.Frequency*100 - 300))   iTempData = (int)(OperationData.Frequency*10 - 300);	// Min: 57.0Hz
		 	UL1741_Setting.F_Lower_Limit_2 = iTempData;		// 600 -> 60.00Hz, use fix point here for comparas		
		 		
			iTempData = eCAN_RxMessage.Data3;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.F_Lower_Limit_1_Trip_Time = iTempData/5 - 2;	// 1ms per unit

			iTempData = eCAN_RxMessage.Data4;
			if ( iTempData < 20 ) 	iTempData = 20;
			if ( iTempData > 10000 ) 	iTempData = 10000;
			UL1741_Setting.F_Lower_Limit_2_Trip_Time = iTempData/5 - 2;	 // 
		
			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_UnderFrequency_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.F_Lower_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.F_Lower_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.F_Lower_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.F_Lower_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);

		break;

		case eCANMsgID_ReConnTime_Setting:					// Reconnect time seeting
		 	iTempData = eCAN_RxMessage.Data1;				// Default 5min = 300sec
		 	UL1741_Setting.GT_ReconnectTime = iTempData;	// 1sec/bit		
			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_ReConnTime_Setting_Echo;
			eCAN_TxMessage.Data_Length = 2;
			eCAN_TxMessage.Data1 = UL1741_Setting.GT_ReconnectTime;
			eCAN_DataTx(&eCAN_TxMessage);
		
		break;
	
		case eCANMsgID_AntiIslanding_Setting:				// Anti-islanding sensitivity seeting
		 	iTempData = eCAN_RxMessage.Data1;				// 
		 	Anti_Islanding.sensitivity = iTempData;// 		
		 	iTempData = eCAN_RxMessage.Data2;				// 
		 	Anti_Islanding.trip_time = iTempData;// 		
		 	iTempData = eCAN_RxMessage.Data3;				// 
		 	iGrid_Tie.Q_inject_Setting = 0.01 * (float)iTempData;// 		
			// send the same data back to host
			eCAN_TxMessage.eCAN_ID = eCANMsgID_AntiIslanding_Setting_Echo;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = Anti_Islanding.sensitivity;
			eCAN_TxMessage.Data2 = Anti_Islanding.trip_time;
			eCAN_TxMessage.Data3 = (int)(iGrid_Tie.Q_inject_Setting * 100.0);
			eCAN_DataTx(&eCAN_TxMessage);
		
		break;

		default:
		break;
	}

}

//_________________________________________________________________________________
//
// Type:	function
// Name:	UL1741_Setting_Report							
//_________________________________________________________________________________
void UL1741_Setting_Report(){
static int ULReportCount=0;

	switch (ULReportCount) {
		case 0:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_OverVoltage_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.V_Upper_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.V_Upper_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.V_Upper_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.V_Upper_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);
			ULReportCount = 1;
		break;

		case 1:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_UnderVoltage_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.V_Lower_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.V_Lower_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.V_Lower_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.V_Lower_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);
			ULReportCount = 2;
		break;

		case 2:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_OverFrequency_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.F_Upper_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.F_Upper_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.F_Upper_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.F_Upper_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);
			ULReportCount = 3;
		break;

		case 3:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_UnderFrequency_Setting_Echo;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = UL1741_Setting.F_Lower_Limit_1;
			eCAN_TxMessage.Data2 = UL1741_Setting.F_Lower_Limit_2;
			eCAN_TxMessage.Data3 = (UL1741_Setting.F_Lower_Limit_1_Trip_Time + 2)*5;
			eCAN_TxMessage.Data4 = (UL1741_Setting.F_Lower_Limit_2_Trip_Time + 2)*5;
			eCAN_DataTx(&eCAN_TxMessage);
			ULReportCount = 4;
		break;

		case 4:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_ReConnTime_Setting_Echo;
			eCAN_TxMessage.Data_Length = 2;
			eCAN_TxMessage.Data1 = UL1741_Setting.GT_ReconnectTime;
			eCAN_DataTx(&eCAN_TxMessage);
			ULReportCount = 5;
		break;

		case 5:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_AntiIslanding_Setting_Echo;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = Anti_Islanding.sensitivity;
			eCAN_TxMessage.Data2 = Anti_Islanding.trip_time;
			eCAN_TxMessage.Data3 = (int)(iGrid_Tie.Q_inject_Setting * 100.0);
			eCAN_DataTx(&eCAN_TxMessage);
			UL1741_report = FALSE;
			ULReportCount = 0;
		break;

		default:	// unknown status
			// do nothing
			ULReportCount = 0;
		break;
	}
}

/**********************************************************************
// Function: UL1741_Line_Monitor()
// Description: Monitor AC line voltage and frequency cycle by cycle.
// Revisions:
// ----------
**********************************************************************/
void UL1741_Line_Monitor() {
int AC_Freq_x100;
    if (system.state != SYS_INIT) {
		ACLine_Test = 0;
		if 	 (((int)AC_Line_Measurement.V_AB_RMS < UL1741_Setting.V_Lower_Limit_1)
			||((int)AC_Line_Measurement.V_BC_RMS < UL1741_Setting.V_Lower_Limit_1)
			||((int)AC_Line_Measurement.V_CA_RMS < UL1741_Setting.V_Lower_Limit_1)) {
			VF_Trip.V_Lower_Limit1_cnt ++;
			if (VF_Trip.V_Lower_Limit1_cnt >= UL1741_Setting.V_Lower_Limit_1_Trip_Time)  {
				//***Inverter_ShutDown_Cmd = 1;				   // 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
					iGrid_Tie.Trip |= dV_LOWER_LIMIT_1_TRIP;
				}
				VF_Trip.V_Lower_Limit1_cnt =  UL1741_Setting.V_Lower_Limit_1_Trip_Time;
				ACLine_Test = 1;
			}
		} else  {
			VF_Trip.V_Lower_Limit1_cnt = 0; 
		}

		if 	 (((int)AC_Line_Measurement.V_AB_RMS < UL1741_Setting.V_Lower_Limit_2)
			||((int)AC_Line_Measurement.V_BC_RMS < UL1741_Setting.V_Lower_Limit_2)
			||((int)AC_Line_Measurement.V_CA_RMS < UL1741_Setting.V_Lower_Limit_2)) {
			VF_Trip.V_Lower_Limit2_cnt ++;
			if (VF_Trip.V_Lower_Limit2_cnt >= UL1741_Setting.V_Lower_Limit_2_Trip_Time)  {
				//***Inverter_ShutDown_Cmd = 1;				// 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
					iGrid_Tie.Trip |= dV_LOWER_LIMIT_2_TRIP;
				}
				VF_Trip.V_Lower_Limit2_cnt =  UL1741_Setting.V_Lower_Limit_2_Trip_Time;
				ACLine_Test = 2;
			}
		} else  {
			VF_Trip.V_Lower_Limit2_cnt = 0; 
		}

		if 	 (((int)AC_Line_Measurement.V_AB_RMS > UL1741_Setting.V_Upper_Limit_1)
			||((int)AC_Line_Measurement.V_BC_RMS > UL1741_Setting.V_Upper_Limit_1)
			||((int)AC_Line_Measurement.V_CA_RMS > UL1741_Setting.V_Upper_Limit_1)) {
			VF_Trip.V_Upper_Limit1_cnt ++;
			if (VF_Trip.V_Upper_Limit1_cnt >= UL1741_Setting.V_Upper_Limit_1_Trip_Time)  {
				//***Inverter_ShutDown_Cmd = 1;		  // 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
					iGrid_Tie.Trip |= dV_UPPER_LIMIT_1_TRIP;
				}
				VF_Trip.V_Upper_Limit1_cnt =  UL1741_Setting.V_Upper_Limit_1_Trip_Time;
				ACLine_Test = 3;
			}
		} else  {
			VF_Trip.V_Upper_Limit1_cnt = 0; 
		}

		if 	 (((int)AC_Line_Measurement.V_AB_RMS > UL1741_Setting.V_Upper_Limit_2)
			||((int)AC_Line_Measurement.V_BC_RMS > UL1741_Setting.V_Upper_Limit_2)
			||((int)AC_Line_Measurement.V_CA_RMS > UL1741_Setting.V_Upper_Limit_2)) {
			VF_Trip.V_Upper_Limit2_cnt ++;
			if (VF_Trip.V_Upper_Limit2_cnt >= UL1741_Setting.V_Upper_Limit_2_Trip_Time)  {
				//***Inverter_ShutDown_Cmd = 1;		  // 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
					iGrid_Tie.Trip |= dV_UPPER_LIMIT_2_TRIP;
				}
				VF_Trip.V_Upper_Limit2_cnt =  UL1741_Setting.V_Upper_Limit_2_Trip_Time;
				ACLine_Test = 4;
			}
		} else  {
			VF_Trip.V_Upper_Limit2_cnt = 0; 
		}

		AC_Freq_x100 = (int)(ACLine.Frequency * 100.0);	
		if (AC_Freq_x100 < UL1741_Setting.F_Lower_Limit_1) {
			VF_Trip.F_Lower_Limit1_cnt ++;
		 	if (VF_Trip.F_Lower_Limit1_cnt >= UL1741_Setting.F_Lower_Limit_1_Trip_Time)  {
		 		//***Inverter_ShutDown_Cmd = 1;			// 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
		 			iGrid_Tie.Trip |= dF_LOWER_LIMIT_1_TRIP;
				}
				VF_Trip.F_Lower_Limit1_cnt =  UL1741_Setting.V_Lower_Limit_1_Trip_Time;
				ACLine_Test = 5;
		 	}
		} else  {
			VF_Trip.F_Lower_Limit1_cnt = 0; 
		}

		if (AC_Freq_x100 < UL1741_Setting.F_Lower_Limit_2) {
			VF_Trip.F_Lower_Limit2_cnt ++;
		 	if (VF_Trip.F_Lower_Limit2_cnt >= UL1741_Setting.F_Lower_Limit_2_Trip_Time)  {
		 		//***Inverter_ShutDown_Cmd = 1;			// 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
		 			iGrid_Tie.Trip |= dF_LOWER_LIMIT_2_TRIP;
				}
				VF_Trip.F_Lower_Limit2_cnt =  UL1741_Setting.V_Lower_Limit_2_Trip_Time;
				ACLine_Test = 6;
		 	}
		} else  {
			VF_Trip.F_Lower_Limit2_cnt = 0; 
		}
		 			 
		if (AC_Freq_x100 > UL1741_Setting.F_Upper_Limit_1) {
			VF_Trip.F_Upper_Limit1_cnt ++;
		 	if (VF_Trip.F_Upper_Limit1_cnt >= UL1741_Setting.F_Upper_Limit_1_Trip_Time)  {
		 		//***Inverter_ShutDown_Cmd = 1;			// 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
		 			iGrid_Tie.Trip |= dF_UPPER_LIMIT_1_TRIP;
				}
				VF_Trip.F_Upper_Limit1_cnt =  UL1741_Setting.V_Upper_Limit_1_Trip_Time;
				ACLine_Test = 7;
		 	}
		} else  {
			VF_Trip.F_Upper_Limit1_cnt = 0; 
		}

		if (AC_Freq_x100 > UL1741_Setting.F_Upper_Limit_2) {
			VF_Trip.F_Upper_Limit2_cnt ++;
		 	if (VF_Trip.F_Upper_Limit2_cnt >= UL1741_Setting.F_Upper_Limit_2_Trip_Time)  {
		 		//***Inverter_ShutDown_Cmd = 1;			// 1: Only Inverter shutDown
				if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
					system.command = SysCtrlCmd_MainOutputOff;
		 			iGrid_Tie.Trip |= dF_UPPER_LIMIT_2_TRIP;
				}
				VF_Trip.F_Upper_Limit2_cnt =  UL1741_Setting.V_Upper_Limit_2_Trip_Time;
				ACLine_Test = 8;
		 	}
		} else  {
			VF_Trip.F_Upper_Limit2_cnt = 0; 
		}

		if (ACLine_Test == 0) {
			system.status |= AC_LINE_OK;
		} else {
			system.status &= ~AC_LINE_OK;
		}

		if (iGrid_Tie.Trip !=0) {
			Boost_Off();
			Inverter_Off();
			DO_7_Off();
		}


	} else {
		//----- Trip time counter init -----
		VF_Trip.F_Upper_Limit1_cnt = 0;
		VF_Trip.F_Upper_Limit2_cnt = 0;
		VF_Trip.F_Lower_Limit1_cnt = 0;
		VF_Trip.F_Lower_Limit2_cnt = 0;
		VF_Trip.V_Upper_Limit1_cnt = 0;
		VF_Trip.V_Upper_Limit2_cnt = 0;
		VF_Trip.V_Lower_Limit1_cnt = 0;
		VF_Trip.V_Lower_Limit2_cnt = 0;

	} 
}
/*** end of file *****************************************************/
