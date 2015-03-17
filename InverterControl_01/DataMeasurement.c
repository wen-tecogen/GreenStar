/**********************************************************************************
// File: DataMeasurement.c
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
// This is the c file all data measurement.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 2/1/13	|  J Wen 	| Original
**********************************************************************************/
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <math.h>

#include "C28x_FPU_FastRTS.h"	// Use sincos(), isqrt()
#include "DSP28x_Project.h"     // Device Header file and Examples Include File
#include "DataMeasurement.h"       // Header file for ePWM module
#include "SystemControl.h"      // Device Header file and Examples Include File
#include "ADC.h"				// Header file for A to D converter
#include "EPWM_control.h"       // Header file for ePWM module
#include "ECAP_VLine.h"     	// Module Include File
#include "Grid_Control.h"		// Header file for grid control


/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle mySem;

// variables for data measurement
float	T_ambient_C;
float	V_temp_sensor;
float	V_temp_sensor_1;
float	I_Boost_report;
float	I_Boost_1;
float	DC_BUS_Voltage, DC_BUS_V1;

float	Generator_IA_RMS;
float	Generator_IB_RMS;
float	Generator_IC_RMS;
float	SamplingFactor;

float	Output_KW;
float	Output_KVA;
float	Phase_A_KVA;
float	Phase_B_KVA;
float	Phase_C_KVA;

int		DC_bus_OV_setting;
int		Brake_ON_V_setting;
int		OverLoad_Counter;
int		Gen_OI_Counter;
// variables for speed control
SPEED_CONTROL speed_control;

// extern variables for data measurement
extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern AC_MEASURE_DATA 	VAC_out_Measurement;		// Inverter Output data
extern Uint16	EERPOM_report;
extern PLL_DATA		PLL_Data;
extern SYSTEM_INFO 	system;
extern OPERATION_DATA 	OperationData;
extern Uint16	UL1741_report;
extern Uint16 	MG_Setting_Report;
extern MICRO_GRID	MicroGrid;

extern ADC_RESULT	ADC3;				// ADC chip (U7) 	U9
extern float	Boost_Current;
extern float	DC_BUS_1;
extern BOOST_CONTROL	Boost_Control;

extern float	Generator_IA[210];
extern float	Generator_IB[210];
extern float	Generator_IC[210];

extern float	Engine_Speed_RPM; 
extern int	 	InputStageFilter;

/******* Function prototype *******/
void AC_Line_RMS(void);
void DC_Signal_Report(void);
void OverLoad_Monitor(void);
void Data_Measure_Init(void);
void EngineSpeedCtrl(void);
void SpeedCtrlRamp(void);

/******* External Function prototype *******/
extern void EEPROM_Report(void);
extern void Data_Report(void);
extern void BoostCommand(void);
extern void UL1741_Setting_Report(void);
extern void MicroGrid_Setting_Report(void);
extern void Brake_On(void);
extern void Brake_Off(void);
extern void Boost_Off(void);
extern void Inverter_Off(void);
extern void DO_6_Off(void);
extern void DO_7_Off(void);

//
// ================ Data_Measure_Init ================
//
void Data_Measure_Init() {
	Output_KW   = 0.0;
	Output_KVA  = 0.0;
	Phase_A_KVA	= 0.0;
	Phase_B_KVA	= 0.0;
	Phase_C_KVA	= 0.0;

	AC_Line_Measurement.I_A_RMS = 0.0;
	AC_Line_Measurement.I_B_RMS = 0.0;
	AC_Line_Measurement.I_C_RMS = 0.0;
	VAC_out_Measurement.V_AB_RMS = 0.0;
	VAC_out_Measurement.V_BC_RMS = 0.0;
	VAC_out_Measurement.V_CA_RMS = 0.0;

	speed_control.S_Control_Ki = 2.0;		 //	 1.0;	
	speed_control.S_Control_Kp = 20.0;		 //	 15.0;
	speed_control.S_Control_Kd = 0.0;
	speed_control.speed_error = 0.0;
	speed_control.speed_error_int = 0.0;
	speed_control.speed_error_int_old = 0.0;
	speed_control.speed_error_pro = 0.0;
	speed_control.speed_error_pro_old = 0.0;
	speed_control.speed_Loop_Compensator = 0.0;
	speed_control.speed_ctrl_state = 0;
	Engine_Speed_RPM = 0.0;
}

//
// ================ myTaskFxn ================
//  Task function that pends on a semaphore until 20 ticks have	(100 msec)
//  expired.
//
Void myTaskFxn(Void) {
	// Data initialization
	V_temp_sensor = 3.5;
	V_temp_sensor_1 = 3.5;
	I_Boost_report = 0.0;
	I_Boost_1 = 0.0;
	DC_BUS_V1 = 0.0;

	DC_bus_OV_setting = 880;
	Brake_ON_V_setting = 910;
	OverLoad_Counter = 0;
	Gen_OI_Counter = 0;
	
	Output_KW  = 0.0;
	Output_KVA = 0.0;
	Phase_A_KVA	= 0.0;
	Phase_B_KVA	= 0.0;
	Phase_C_KVA	= 0.0;
    
    /*
     * Do this forever
     */
    while (TRUE) {
        /* 
         * Pend on "mySem" until the timer ISR says
         * its time to do something.
         */ 
        Semaphore_pend(mySem, BIOS_WAIT_FOREVER);

    	//GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;		// GPIO63 : DO1
		SamplingFactor = 1.0 + 0.0027* (165.0 - (float)OperationData.Period_Cnt);
		if (system.state != SYS_INIT) {
			AC_Line_RMS();
			DC_Signal_Report();
		}
        // ==========================
		// Boost command ramp up/down:
		BoostCommand();
        // ==========================
		// Over load monitoring:
		if (system.state == GRID_TIE_MODE || system.state == MICRO_GRID_MODE) {
			OverLoad_Monitor();
		}
        // ==========================
		// Engine speed control in grid-tie mode:
		if (OperationData.GT_SpeedControl == TRUE) {
			SpeedCtrlRamp();
		}
//		if (OperationData.GT_SpeedControl == TRUE) {
//		//if (system.state == GRID_TIE_MODE && OperationData.GT_SpeedControl == TRUE) {
//			EngineSpeedCtrl();
//		} else {
//			// reset control initial
//			speed_control.speed_error = 0.0;
//			speed_control.speed_error_int = 0.0;
//			speed_control.speed_error_int_old = 0.0;
//			speed_control.speed_error_pro = 0.0;
//			speed_control.speed_error_pro_old = 0.0;
//			speed_control.speed_Loop_Compensator = 0.0;
//			speed_control.speed_ctrl_state = 0;
//		}
        // ==========================
		// System setting data report:
		if (!system.FlashEnable) {
        	if (EERPOM_report == TRUE) {
        		EEPROM_Report();
			} else if (UL1741_report== TRUE) {
				UL1741_Setting_Report();
			} else if (MG_Setting_Report== TRUE) {
				MicroGrid_Setting_Report();
        	} else {
        		Data_Report();
        	}
		} // else just do nothing for flash programming
    }
}



/**********************************************************************
// Function: AC_Line_RMS()
// Description: AC voltage/current rms measurement.
// Revisions:
// ----------
**********************************************************************/
void AC_Line_RMS() {
Uint16 i;
float	fTemp_0, fTemp_1, fTemp_2;
float	fTemp_3, fTemp_4, fTemp_5;
float	fTemp_6, fTemp_7, fTemp_8;
float	fTemp_9, fTemp_10, fTemp_11;
float	fTemp_PA, fTemp_PB, fTemp_PC;

	// AC voltage rms measurement
	fTemp_0 = 0.0;
	fTemp_1 = 0.0;
	fTemp_2 = 0.0;
	fTemp_3 = 0.0;
	fTemp_4 = 0.0;
	fTemp_5 = 0.0;
	fTemp_6 = 0.0;
	fTemp_7 = 0.0;
	fTemp_8 = 0.0;
	fTemp_9 = 0.0;
	fTemp_10 = 0.0;
	fTemp_11 = 0.0;


	for (i = 0; i<OperationData.Period_Cnt; i++) {  
		fTemp_PA = (AC_Line_Measurement.V_AB_Line_buff[i] - AC_Line_Measurement.V_CA_Line_buff[i])/3.0;	  		// VA	
		fTemp_PB = (AC_Line_Measurement.V_BC_Line_buff[i] - AC_Line_Measurement.V_AB_Line_buff[i])/3.0;	  		// VB	
		fTemp_PC = (AC_Line_Measurement.V_CA_Line_buff[i] - AC_Line_Measurement.V_BC_Line_buff[i])/3.0;	  		// VC
		fTemp_PA *= AC_Line_Measurement.I_A_buff[i];														  	// PA
		fTemp_PB *= AC_Line_Measurement.I_B_buff[i];														  	// PB
		fTemp_PC *= AC_Line_Measurement.I_C_buff[i];															// PC
		fTemp_0 += fTemp_PA;
		fTemp_1 += fTemp_PB;
		fTemp_2 += fTemp_PC;														  	
				
		fTemp_3 += 0.1 * AC_Line_Measurement.I_A_buff[i] * AC_Line_Measurement.I_A_buff[i];		
		fTemp_4 += 0.1 * AC_Line_Measurement.I_B_buff[i] * AC_Line_Measurement.I_B_buff[i];		
		fTemp_5 += 0.1 * AC_Line_Measurement.I_C_buff[i] * AC_Line_Measurement.I_C_buff[i];		
		fTemp_6 += 0.01 * VAC_out_Measurement.V_AB_Line_buff[i] * VAC_out_Measurement.V_AB_Line_buff[i];		
		fTemp_7 += 0.01 * VAC_out_Measurement.V_BC_Line_buff[i] * VAC_out_Measurement.V_BC_Line_buff[i];		
		fTemp_8 += 0.01 * VAC_out_Measurement.V_CA_Line_buff[i] * VAC_out_Measurement.V_CA_Line_buff[i];		
		fTemp_9  += 0.01 * Generator_IA[i] * Generator_IA[i];		
		fTemp_10 += 0.01 * Generator_IB[i] * Generator_IB[i];		
		fTemp_11 += 0.01 * Generator_IC[i] * Generator_IC[i];		
	}
	//AC_Line_Measurement.V_AB_RMS = sqrt(fTemp_0) * VAC_RMS_SCALE_FACTOR;
	//AC_Line_Measurement.V_BC_RMS = sqrt(fTemp_1) * VAC_RMS_SCALE_FACTOR;
	//AC_Line_Measurement.V_CA_RMS = sqrt(fTemp_2) * VAC_RMS_SCALE_FACTOR;
	AC_Line_Measurement.I_A_RMS = sqrt(fTemp_3) * IAC_RMS_SCALE_FACTOR * SamplingFactor;
	AC_Line_Measurement.I_B_RMS = sqrt(fTemp_4) * IAC_RMS_SCALE_FACTOR * SamplingFactor;
	AC_Line_Measurement.I_C_RMS = sqrt(fTemp_5) * IAC_RMS_SCALE_FACTOR * SamplingFactor;
	VAC_out_Measurement.V_AB_RMS = sqrt(fTemp_6) * VAC_RMS_SCALE_FACTOR * SamplingFactor;
	VAC_out_Measurement.V_BC_RMS = sqrt(fTemp_7) * VAC_RMS_SCALE_FACTOR * SamplingFactor;
	VAC_out_Measurement.V_CA_RMS = sqrt(fTemp_8) * VAC_RMS_SCALE_FACTOR * SamplingFactor;
	Generator_IA_RMS = sqrt(fTemp_9 ) * GEN_I_RMS_SCALE_FACTOR * SamplingFactor;
	Generator_IB_RMS = sqrt(fTemp_10) * GEN_I_RMS_SCALE_FACTOR * SamplingFactor;
	Generator_IC_RMS = sqrt(fTemp_11) * GEN_I_RMS_SCALE_FACTOR * SamplingFactor;

	AC_Line_Measurement.Power_A = POWER_SCALE_FACTOR * fTemp_0/OperationData.Period_Cnt;
	AC_Line_Measurement.Power_B = POWER_SCALE_FACTOR * fTemp_1/OperationData.Period_Cnt;
	AC_Line_Measurement.Power_C = POWER_SCALE_FACTOR * fTemp_2/OperationData.Period_Cnt;

	//***** Output power in KW and KVA *****
	Output_KW = 0.1 * (AC_Line_Measurement.Power_A + AC_Line_Measurement.Power_B + AC_Line_Measurement.Power_C)
			  + 0.9 * Output_KW;
	Phase_A_KVA	 = 0.005774 * VAC_out_Measurement.V_AB_RMS * AC_Line_Measurement.I_A_RMS;
	Phase_B_KVA	 = 0.005774 * VAC_out_Measurement.V_BC_RMS * AC_Line_Measurement.I_B_RMS;
	Phase_C_KVA	 = 0.005774 * VAC_out_Measurement.V_CA_RMS * AC_Line_Measurement.I_C_RMS;
	Output_KVA = 0.1 * (Phase_A_KVA + Phase_B_KVA + Phase_C_KVA) + 0.9 * Output_KVA;

	if ((int)Generator_IA_RMS > 220 || (int)Generator_IB_RMS > 220 || (int)Generator_IC_RMS > 220) {
		Gen_OI_Counter ++;
		if (Gen_OI_Counter > InputStageFilter) {
			system.SW_faults |= GENERATOR_OVERCURRENT;
		}
	} else {
		Gen_OI_Counter = 0;
	}

	// AC line OK set to 100V L-L for now
	if ((unsigned int)AC_Line_Measurement.V_AB_RMS > 100	&& (unsigned int)AC_Line_Measurement.V_BC_RMS > 100) {
		PLL_Data.AC_Line_Measure_OK = TRUE;
		//system.status |= AC_LINE_OK;
	} else {
		PLL_Data.AC_Line_Measure_OK = FALSE;
		//system.status &= ~AC_LINE_OK;
	}	
}

/**********************************************************************
// Function: DC_Signal_Report()
// Description: ADC3 ch0 for temperature sensor input.
// Revisions:
// ----------
**********************************************************************/
void DC_Signal_Report() {
	// Read ADC3 ch0
	V_temp_sensor = 0.1 * ADC3.ADC_ch0 + 0.9 * V_temp_sensor_1;
	V_temp_sensor_1 = V_temp_sensor;
	T_ambient_C = 5.43 * V_temp_sensor * V_temp_sensor - 44.5 * V_temp_sensor + 115.88;
	if ((int)T_ambient_C > 65) {
		system.SW_faults |= EMBIENT_OVER_TEMP;
	}

	// Boost current
	I_Boost_report = 0.02 * Boost_Current + 0.98 * I_Boost_1;
	I_Boost_1 = I_Boost_report;
	I_Boost_report *= I_BOOST_SCALE_FACTOR;

	// DC bus voltage
	DC_BUS_Voltage = 0.2 * DC_BUS_1 + 0.8 * DC_BUS_V1;
	DC_BUS_V1 = DC_BUS_Voltage;
	//if ((int)DC_BUS_Voltage > ((int)Boost_Control.DCBUS_cmd_Active + 30)) {
	if ((int)DC_BUS_Voltage > DC_bus_OV_setting) {
		system.SW_faults |= DC_BUS_OVERVOLTAGE;
	}

	if ((int)DC_BUS_1 > Brake_ON_V_setting) {
		Brake_On();
		Boost_Off();
		Inverter_Off();
		DO_6_Off();
		DO_7_Off();
	}

	if ((int)DC_BUS_Voltage < (Brake_ON_V_setting - 40) && !system.TestMode) {
		Brake_Off();
	}

}

/**********************************************************************
// Function: OverLoad_Monitor()
// Description: Based on output current, >150%, shutdown right away.
//				>120%, shutdown in 10sec
//				>105%, shutdown in 30sec
// Revisions:
// ----------
**********************************************************************/

void OverLoad_Monitor() {
int		Output_VA;
int		OverLoad_105_percent;
int		OverLoad_120_percent;
int		OverLoad_150_percent;
    
	MicroGrid.KVA = sqrt(MicroGrid.P * MicroGrid.P + MicroGrid.Q * MicroGrid.Q);
	Output_VA = (int)(10.0 * MicroGrid.KVA);
	OverLoad_105_percent = (int)(1.05 * MicroGrid.P_max_setting); 
	OverLoad_120_percent = (int)(1.20 * MicroGrid.P_max_setting);
	OverLoad_150_percent = (int)(1.50 * MicroGrid.P_max_setting);
	
	//---- Over load 105% ----
	if (Output_VA >= OverLoad_105_percent) {
		system.status |= OVER_LOAD;
		OverLoad_Counter ++ ;

		if (OverLoad_Counter >= OVER_LOAD_30SEC) {
			system.SW_faults |= OVER_LOAD_FAULT;  
			system.status &= ~OVER_LOAD;
		}
	} 
	//---- Over load 120% ----
	if (Output_VA >= OverLoad_120_percent) {
		system.status |= OVER_LOAD;
		OverLoad_Counter ++ ;

		if (OverLoad_Counter >= OVER_LOAD_10SEC) {
			system.SW_faults |= OVER_LOAD_FAULT;  
			system.status &= ~OVER_LOAD;
		}
	} 
	//---- Over load 150% ----
	if (Output_VA >= OverLoad_150_percent) {
		system.status |= OVER_LOAD;
		OverLoad_Counter ++ ;

		if (OverLoad_Counter >= OVER_LOAD_2SEC) {
			system.SW_faults |= OVER_LOAD_FAULT;  
			system.status &= ~OVER_LOAD;
		}
	} 

	//---- Clear over load status
	if (Output_VA <= OverLoad_105_percent) {
		system.status &= ~OVER_LOAD;
		OverLoad_Counter -= 5;
		if (OverLoad_Counter <= 0)	OverLoad_Counter = 0;

	}

}

/**********************************************************************
// Function: EngineSpeedCtrl()
// Description: Operating in GridTie mode.
//				Active control engine speed by changing output current
// Revisions:
// ----------
**********************************************************************/

void EngineSpeedCtrl() {
int S_PI_output;
//int i_speed_err;
	// Speed error
	speed_control.speed_error = OperationData.GT_SpeedCommand - Engine_Speed_RPM;
	//i_speed_err = (int)speed_control.speed_error;
	speed_control.speed_error *= 0.01;
	
	//if (i_speed_err < 100) {
	if (1) {
		// Proportional
		speed_control.speed_error_pro = speed_control.S_Control_Kp * speed_control.speed_error;
		// Integral
		speed_control.speed_error_int = speed_control.S_Control_Ki * speed_control.speed_error;
		speed_control.speed_error_int += speed_control.speed_error_int_old;
		// Integral limit with + 100A /- 25A
		S_PI_output = (int)(speed_control.speed_error_int * 10.0);
    	if (S_PI_output >= 250) speed_control.speed_error_int = 25.0;
    	if (S_PI_output <= -1000) speed_control.speed_error_int = -100.0;

		// Derivative
		speed_control.speed_error_der = speed_control.S_Control_Kd * 
									    (speed_control.speed_error_pro - speed_control.speed_error_pro_old);
		// P + I + D
		speed_control.speed_Loop_Compensator = speed_control.speed_error_pro
											 + speed_control.speed_error_int
											 + speed_control.speed_error_der; 
		speed_control.speed_ctrl_state = 1;
	} else {
		// Proportional
		speed_control.speed_error_pro = 5.0 * speed_control.S_Control_Kp * speed_control.speed_error;
		// Integral
		speed_control.speed_error_int = speed_control.S_Control_Ki * speed_control.speed_error;
		speed_control.speed_error_int += speed_control.speed_error_int_old;
		// Integral limit with + 100A /- 25A
		S_PI_output = (int)(speed_control.speed_error_int * 10.0);
    	if (S_PI_output >= 250) speed_control.speed_error_int = 25.0;
    	if (S_PI_output <= -1000) speed_control.speed_error_int = -100.0;

		// Derivative
		speed_control.speed_error_der = speed_control.S_Control_Kd * 
									    (speed_control.speed_error_pro - speed_control.speed_error_pro_old);
		// P + I + D
		speed_control.speed_Loop_Compensator = speed_control.speed_error_pro
											 + speed_control.speed_error_int
											 + speed_control.speed_error_der; 
		speed_control.speed_ctrl_state = 2;
	}

	// output limit with + 100A /- 25A
	S_PI_output = (int)(speed_control.speed_Loop_Compensator * 10.0);
    if (S_PI_output >= 50) speed_control.speed_Loop_Compensator = 5.0;
    if (S_PI_output <= -1000) speed_control.speed_Loop_Compensator = -100.0;
	
	speed_control.speed_error_int_old = speed_control.speed_error_int;
	speed_control.speed_error_pro_old = speed_control.speed_error_pro;


//	// Modified output current
//	OperationData.ID_out_cmd -=	speed_control.speed_Loop_Compensator;
//	if ((int)OperationData.ID_out_cmd > 130) {
//		OperationData.ID_out_cmd = 130.0;
//	}

}

/**********************************************************************
// Function: SpeedCtrlRamp()
// Description: Speed Control Ramp in GridTie mode.
//				
// Revisions:
// ----------
**********************************************************************/

void SpeedCtrlRamp() {
	if ((int)OperationData.GT_SpeedCommand > OperationData.int_GT_SpeedCommand + 3) {
		OperationData.GT_SpeedCommand -= 3.0;	
	} else if ((int)OperationData.GT_SpeedCommand < OperationData.int_GT_SpeedCommand - 3) {
		OperationData.GT_SpeedCommand += 1.0;	
	} else {
		OperationData.GT_SpeedCommand = (float)OperationData.int_GT_SpeedCommand;	
	}

}
/*** end of file *****************************************************/
