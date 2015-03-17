/**********************************************************************************
// File: ECAP_VLine.c
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
// This module contains the line frequency measurement function by using ECAP.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/13/12	|  J Wen 	| Original
**********************************************************************************/
#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>
#include <math.h>

#include "C28x_FPU_FastRTS.h"	// Use sincos(), isqrt()
#include "DSP28x_Project.h"     // Device Header-file Include File
#include "ECAP_VLine.h"     	// Module Include File
#include "EPWM_control.h"       // Header file for ePWM module
#include "SystemControl.h"      // Header file for system control
#include "ADC.h"				// Header file for A to D converter
#include "DataMeasurement.h"       // Header file for ePWM module

/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle lineSema;

/******* Swi handle defined in swi.cfg *******/
extern const Swi_Handle swi2;	// Capture interrupt
extern const Swi_Handle swi3;	// ADC interrupt

// Prototype statements for functions found within this file.
interrupt void ecap1_isr(void);
interrupt void ecap4_isr(void);
void ECAP_Setup(void);
void ECAP_Vars_init(void);
void ELine_PLL(void);
void ELine_PLL_QCtrl(void);

// External functions
extern void ADC1_read(void);
extern void ADC2_read(void);
extern void ADC3_read(void);
extern void ADC4_read(void);
extern void BoostControl(void);
extern void EPWM1_On(void);
extern void UL1741_Line_Monitor(void);
extern void MAG_Sensor_Read(void);
extern void EngineSpeedCtrl(void);

// Variables in this module
ECAP_DATA	eCAP1_Data;
ECAP_DATA	eCAP2_Data;
ECAP_DATA	eCAP3_Data;
AC_LINE_DATA	ACLine;

int	 phase_detect_cnt;
//int	 V_A_at_cap1isr;
//int	 V_B_at_cap1isr;
//int	 V_C_at_cap1isr;

float Frequency_new;
float Frequency_old;
int   PLL_Operation_Cnt;
Uint16	LineMonitorState;
int   SpdCtrl_Op_Cnt;

// Variables for PLL
PLL_DATA	PLL_Data;
Uint16		Phase_Lock;
int		iACLIne_F_delta;

extern OPERATION_DATA 	OperationData;
extern SYSTEM_INFO 	system;
extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern AC_MEASURE_DATA 	VAC_out_Measurement;		// Inverter output data
extern INVERTER_CONTROL Inverter_SysControl;
extern GRID_TIE	iGrid_Tie;
extern float	SamplingFactor;
extern CALIBRATION		Calibration;
extern SPEED_CONTROL speed_control;
extern Uint16	GT_SpeedCtrl_Cnt;
extern float	report_Engine_Speed_RPM;
extern int		EERPOM_AC_Frequency;


//
//  ======== LineMonitor ========
//  Line Monitor task, execute every 5msec
//
Void LineMonitor(Void) {
Uint16 i;
float	fTemp_0, fTemp_1, fTemp_2;

	/****** Variables initialization ******/
    //
    // Do this forever
    //
    while (TRUE) {
        /*
         * Pend on "lineSema" until the timer ISR says
         * its time to do something.
         */
        Semaphore_pend(lineSema, BIOS_WAIT_FOREVER);
		//--- lineSema rate = 5 msec ---//
		//GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;		// Profile

        eCAP1_Data.CaptureMonitor ++ ;
        if (eCAP1_Data.CaptureMonitor >= 50){
        	eCAP1_Data.CaptureMonitor = 50;
        	/**** if more than 250msec without capture signal ****/
        	PLL_Data.PLL_Enable = FALSE;
        	system.status &= ~PLL_ENABLE;
            PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
        	system.status &= ~PLL_LOCK;
			eCAP1_Data.CaptureFilter = 0;
        }
//        if (eCAP1_Data.CaptureMonitor == 2){
//        	/**** Reset capture pin function ****/
//        	InitECap1Gpio();
//    		// Clear interrupt flag, prepare for next interrupt.
//		    ECap1Regs.ECCLR.bit.CEVT1 = 1;
//			ECap1Regs.ECCLR.bit.INT = 1;
//			ECap1Regs.ECCTL2.bit.REARM = 1;
//    		GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;		// Profile isr
//        }
        if (eCAP1_Data.CaptureCnt >= 100 && PLL_Data.AC_Line_Measure_OK == TRUE){
        	eCAP1_Data.CaptureCnt = 100;
			if (system.state != MICRO_GRID_MODE) {
        		PLL_Data.PLL_Enable = TRUE;
        		system.status |= PLL_ENABLE;
			} else {
        		PLL_Data.PLL_Enable = FALSE;
        		system.status &= ~PLL_ENABLE;
        		system.status &= ~PLL_LOCK;
				PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
			}
        } else {
        	PLL_Data.PLL_Enable = FALSE;
        	system.status &= ~PLL_ENABLE;
        	system.status &= ~PLL_LOCK;
			PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
		}
		// ***** AC Line monitor *****
		fTemp_0 = 0.0;
		fTemp_1 = 0.0;
		fTemp_2 = 0.0;
		for (i = 0; i<OperationData.Period_Cnt; i++) {  
			fTemp_0 += 0.01 * AC_Line_Measurement.V_AB_Line_buff[i] * AC_Line_Measurement.V_AB_Line_buff[i];		
			fTemp_1 += 0.01 * AC_Line_Measurement.V_BC_Line_buff[i] * AC_Line_Measurement.V_BC_Line_buff[i];		
			fTemp_2 += 0.01 * AC_Line_Measurement.V_CA_Line_buff[i] * AC_Line_Measurement.V_CA_Line_buff[i];		
		}
		AC_Line_Measurement.V_AB_RMS = sqrt(fTemp_0) * VAC_RMS_SCALE_FACTOR * SamplingFactor * Calibration.V_ab;
		AC_Line_Measurement.V_BC_RMS = sqrt(fTemp_1) * VAC_RMS_SCALE_FACTOR * SamplingFactor * Calibration.V_bc;
		AC_Line_Measurement.V_CA_RMS = sqrt(fTemp_2) * VAC_RMS_SCALE_FACTOR * SamplingFactor * Calibration.V_ca;
		//**** Monitor anormal voltage and frequency ****
		if (system.state != MICRO_GRID_MODE) {
		//if (system.state == GRID_TIE_MODE) {
			UL1741_Line_Monitor();
		}

		// Engine speed
		MAG_Sensor_Read();
        // ==========================
		// Engine speed control in grid-tie mode:
		if (OperationData.GT_SpeedControl == TRUE) {
		//if (system.state == GRID_TIE_MODE && OperationData.GT_SpeedControl == TRUE) {
			SpdCtrl_Op_Cnt ++ ;
			if (SpdCtrl_Op_Cnt >= 4) {
				EngineSpeedCtrl();
				SpdCtrl_Op_Cnt = 0;
			}
		} else {
			// reset control initial
			speed_control.S_Control_Ki = 1.0;
			speed_control.S_Control_Kp = 15.0;
			speed_control.S_Control_Kd = 0.0;
			speed_control.speed_error = 0.0;
			speed_control.speed_error_int = 0.0;
			speed_control.speed_error_int_old = 0.0;
			speed_control.speed_error_pro = 0.0;
			speed_control.speed_error_pro_old = 0.0;
			speed_control.speed_Loop_Compensator = 0.0;
			speed_control.speed_ctrl_state = 0;
			GT_SpeedCtrl_Cnt = 0;
		  	SpdCtrl_Op_Cnt = 0;
			//OperationData.GT_SpeedCommand = 1950.0;
			if ((int)report_Engine_Speed_RPM > 1650) {
				OperationData.GT_SpeedCommand = report_Engine_Speed_RPM - 80.0;
			} else {
				OperationData.GT_SpeedCommand = 1600.0;
			}
		}

    } //--- end of while (TRUE)
}

/*
 *  ======== ECAP_Setup  ========
 *  Setup eCAP channels
 */
void ECAP_Setup() {

	//InitECap1Gpio();
	InitECap2Gpio();
	InitECap3Gpio();
	InitECap4Gpio();							// ECAP4 connects to ADC busy signal. Falling edge: ADC complete

	// **** ECAP1 Setup for line detection
	ECap1Regs.ECEINT.all = 0x0000;             	// Disable all capture interrupts
    ECap1Regs.ECCLR.all = 0xFFFF;              	// Clear all CAP interrupt flags
    ECap1Regs.ECCTL1.bit.CAPLDEN = 0;          	// Disable CAP1-CAP4 register loads
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;        	// Make sure the counter is stopped

    // Initialization Time
    //==========================================
    // ECAP module 1 configuration
    ECap1Regs.ECCTL1.bit.CAP1POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CAP3POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    ECap1Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
    ECap1Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap1Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap1Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap1Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap1Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap1Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; 	// Allow TSCTR to run
    // Run Time ( e.g. CEVT1 triggered ISR call)
    //==========================================
    // Note: here Time-stamp directly represents the Period value.
    eCAP1_Data.Period[0] = ECap1Regs.CAP1; 		// Fetch Time-Stamp captured at T1
    eCAP1_Data.Period[1] = ECap1Regs.CAP2; 		// Fetch Time-Stamp captured at T2
    eCAP1_Data.Period[2] = ECap1Regs.CAP3; 		// Fetch Time-Stamp captured at T3
    eCAP1_Data.Period[3] = ECap1Regs.CAP4; 		// Fetch Time-Stamp captured at T4

    //ECap1Regs.ECCTL2.bit.TSCTRSTOP = 1;       // Start Counter
    ECap1Regs.ECCTL2.bit.REARM = 1;            	// arm one-shot
    //ECap1Regs.ECCTL1.bit.CAPLDEN = 1;         // Enable CAP1-CAP4 register loads
    ECap1Regs.ECEINT.bit.CEVT1 = 1;             // every 1 events = interrupt

    PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
	PLL_Data.PLL_Enable = FALSE;
    eCAP1_Data.Capture = FALSE;
    eCAP1_Data.CaptureCnt = 0;
    eCAP1_Data.index = 0;
	eCAP1_Data.CaptureFilter = 0;
    eCAP1_Data.CaptureMonitor = 0;
	PLL_Operation_Cnt = 0;
	Phase_Lock = FALSE;
	PLL_Data.PLL_Compensator = 0.0;
	PLL_Data.PLL_error_int = 0.0;
	PLL_Data.PLL_error_int_old = 0.0;
	PLL_Data.PLL_Kp = 10;
	PLL_Data.PLL_Ki = 0.02;
	PLL_Data.Phase_Offset = -20;
	PLL_Data.Q_command = -10.0;

	Frequency_old = 0.0;
	ACLine.Frequency = 0.0;
	phase_detect_cnt = 0;
	LineMonitorState = 0;

	// **** ECAP4 Setup for ADC complete detection
	ECap4Regs.ECEINT.all = 0x0000;             	// Disable all capture interrupts
    ECap4Regs.ECCLR.all = 0xFFFF;              	// Clear all CAP interrupt flags
    ECap4Regs.ECCTL1.bit.CAPLDEN = 0;          	// Disable CAP1-CAP4 register loads
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = 0;        	// Make sure the counter is stopped

    // Initialization Time
    //==========================================
    // ECAP module 1 configuration
    ECap4Regs.ECCTL1.bit.CAP1POL = EC_FALLING;
    ECap4Regs.ECCTL1.bit.CAP2POL = EC_FALLING;
    ECap4Regs.ECCTL1.bit.CAP3POL = EC_FALLING;
    ECap4Regs.ECCTL1.bit.CAP4POL = EC_FALLING;
    ECap4Regs.ECCTL1.bit.CTRRST1 = EC_DELTA_MODE;
    ECap4Regs.ECCTL1.bit.CTRRST2 = EC_DELTA_MODE;
    ECap4Regs.ECCTL1.bit.CTRRST3 = EC_DELTA_MODE;
    ECap4Regs.ECCTL1.bit.CTRRST4 = EC_DELTA_MODE;
    ECap4Regs.ECCTL1.bit.CAPLDEN = EC_ENABLE;
    ECap4Regs.ECCTL1.bit.PRESCALE = EC_DIV1;
    ECap4Regs.ECCTL2.bit.CAP_APWM = EC_CAP_MODE;
    ECap4Regs.ECCTL2.bit.CONT_ONESHT = EC_CONTINUOUS;
    ECap4Regs.ECCTL2.bit.SYNCO_SEL = EC_SYNCO_DIS;
    ECap4Regs.ECCTL2.bit.SYNCI_EN = EC_DISABLE;
    ECap4Regs.ECCTL2.bit.TSCTRSTOP = EC_RUN; 	// Allow TSCTR to run

    //ECap4Regs.ECCTL2.bit.TSCTRSTOP = 1;       // Start Counter
    ECap4Regs.ECCTL2.bit.REARM = 1;            	// arm one-shot
    //ECap4Regs.ECCTL1.bit.CAPLDEN = 1;         // Enable CAP1-CAP4 register loads
    ECap4Regs.ECEINT.bit.CEVT1 = 1;             // every 1 events = interrupt

}

/*
 *  ======== ECAP_Vars_init  ========
 *  Initialize all variables
 */
void ECAP_Vars_init() {
    eCAP1_Data.Period[0] = ECap1Regs.CAP1; 		// Fetch Time-Stamp captured at T1
    eCAP1_Data.Period[1] = ECap1Regs.CAP2; 		// Fetch Time-Stamp captured at T2
    eCAP1_Data.Period[2] = ECap1Regs.CAP3; 		// Fetch Time-Stamp captured at T3
    eCAP1_Data.Period[3] = ECap1Regs.CAP4; 		// Fetch Time-Stamp captured at T4

    PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
	PLL_Data.PLL_Enable = FALSE;
    eCAP1_Data.Capture = FALSE;
    eCAP1_Data.CaptureCnt = 0;
    eCAP1_Data.index = 0;
	eCAP1_Data.CaptureFilter = 0;
    eCAP1_Data.CaptureMonitor = 0;
	PLL_Operation_Cnt = 0;
	Phase_Lock = FALSE;
	PLL_Data.PLL_Compensator = 0.0;
	PLL_Data.PLL_error_int = 0.0;
	PLL_Data.PLL_error_int_old = 0.0;
	SpdCtrl_Op_Cnt = 0;
}
/*
 *  ======== ecap1_isr  ========
 *  Capture 1 isr, use for frequency measurement.
 */
interrupt void ecap1_isr(void) {
	GpioDataRegs.GPASET.bit.GPIO31 = 1; 		// Profile isr
	//PLL_Data.PLL_error = OperationData.Period_Cnt - OperationData.EPwmTimerIntCount;
    EALLOW;
	GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;    // Configure GPIO24 as GPIO, de-bouncing
    EDIS;
	Swi_post(swi2);
	// Acknowledge this interrupt to receive more interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
//    // Clear interrupt flag, prepare for next interrupt.
//  ECap1Regs.ECCLR.bit.CEVT1 = 1;
//	ECap1Regs.ECCLR.bit.INT = 1;
//	ECap1Regs.ECCTL2.bit.REARM = 1;
}

/*
 *  ======== ecap4_isr  ========
 *  Capture 4 isr, use for ADC busy signal.
 */
interrupt void ecap4_isr(void) {
	ECap4Regs.ECCLR.bit.CEVT1 = 1;
    ECap4Regs.ECCLR.bit.INT = 1;
    ECap4Regs.ECCTL2.bit.REARM = 1;
	//GpioDataRegs.GPASET.bit.GPIO31 = 1; 		// Profile isr
	Swi_post(swi3);
	// Acknowledge this interrupt to receive more interrupts from group 4
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP4;
}


//
//  ======== eCAP_swi ========
//  Software interrupt function that to perform
//  real-time control functions.
//
Void eCAP_swi(UArg arg) {
Uint32 eCAP1_temp;
int  VAC_line, V_AVG, V_AB, V_BC, V_CA;

	//GpioDataRegs.GPBTOGGLE.bit.GPIO62 = 1;
    eCAP1_Data.Capture = TRUE;
    eCAP1_Data.CaptureMonitor = 0;
	eCAP1_Data.CaptureFilter = 0;
    eCAP1_Data.CaptureCnt ++ ;

	//--- Frequency Measurement ----------------------------------------
	eCAP1_temp = ECap1Regs.CAP1;
	if(eCAP1_temp > 4000000) {
		eCAP1_Data.Period[eCAP1_Data.index] = eCAP1_temp;
	}
	eCAP1_Data.index ++ ;
	if (eCAP1_Data.index >= 4) eCAP1_Data.index = 0;

	eCAP1_Data.avg_Period = eCAP1_Data.Period[0] + eCAP1_Data.Period[1]
	                      + eCAP1_Data.Period[2] + eCAP1_Data.Period[3];
	eCAP1_Data.avg_Period /= 4;		 // without avg filter: 4
	Frequency_new = CPU_FREQUENCY/eCAP1_Data.avg_Period;
	ACLine.Frequency = 0.4 * Frequency_new + 0.6 * Frequency_old; 
	Frequency_old = Frequency_new; 
	//------------------------------------------------------------------

	if (Inverter_SysControl.SystemOperationMode == GRID_TIE_MODE && Inverter_SysControl.InverterOn == TRUE) {
		EPWM1_On();
		Inverter_SysControl.InverterOn = FALSE;
	} // turn on inverter sametime every time
    // Clear interrupt flag, prepare for next interrupt.
    ECap1Regs.ECCLR.bit.CEVT1 = 1;
	ECap1Regs.ECCLR.bit.INT = 1;
	ECap1Regs.ECCTL2.bit.REARM = 1;
	if (PLL_Data.PLL_Enable) {
//		ELine_PLL();
		AC_Line_Measurement.Line_buff_index = 0;
		VAC_out_Measurement.Line_buff_index = 0;
	}

	//---- Phase rotation ----
	if (PLL_Data.PLL_State == dPHASE_UNLOCK) {
   		//V_A_at_cap1isr = (int)AC_Line_Measurement.V_A;
   		//V_B_at_cap1isr = (int)AC_Line_Measurement.V_B;
   		//V_C_at_cap1isr = (int)AC_Line_Measurement.V_C;
		VAC_line = (int)(10 * AC_Line_Measurement.V_Line);
		V_AB = (int)AC_Line_Measurement.V_AB_RMS;
		V_BC = (int)AC_Line_Measurement.V_BC_RMS;
		V_CA = (int)AC_Line_Measurement.V_CA_RMS;
		V_AVG = (V_AB + V_BC + V_CA)/3;

		if (VAC_line > V_AVG) {
			phase_detect_cnt ++ ;
			if (phase_detect_cnt >= 20) {
				PLL_Data.PhaseRotation_ABC = TRUE;
				system.setting |= LINE_PH_ROTATION_ABC;
			}
		} else {
			phase_detect_cnt ++ ;
			if (phase_detect_cnt >= 20) {
				PLL_Data.PhaseRotation_ABC = FALSE;
				system.setting &= ~LINE_PH_ROTATION_ABC;
				if (system.state != SYS_INIT) {
					system.SW_faults |= PHASE_ROTATION_FAULT;   // Make it fault for now.
				}
			}
		}
	}


}

//
//  ======== ADC_swi ========
//  Software interrupt function that to perform
//  ADC results loading functions.
//
Void ADC_swi(UArg arg) {
	ADC1_read();
	ADC2_read();
	ADC3_read();
	ADC4_read();

	BoostControl();

	//-- Phase Lock Loop
	if (PLL_Data.PLL_Enable) {
		ELine_PLL_QCtrl();
	}

//	ECap4Regs.ECCLR.bit.CEVT1 = 1;
//    ECap4Regs.ECCLR.bit.INT = 1;
//    ECap4Regs.ECCTL2.bit.REARM = 1;
    //GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;		// Profile isr

//	eCAP1_Data.CaptureFilter ++;
//	if (eCAP1_Data.CaptureFilter >= 110) {
//    	InitECap1Gpio();
//    	GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;		// Profile isr
//	}

	
}
//________________________________________________________________________________
// Type:	function
// Name:	ELine_PLL
//
// Description:
// ------------
// In stand-by state, if E line is present, PLL control will replace P or F vs
// frequency control.
// !!!!!!!!!!!!!!!!!!!!!! Note: this is temporary function.
void ELine_PLL(){
float	fTemp;
int		iTemp0, iTemp1, iTemp2;

   //V_A_at_cap1isr = (int)AC_Line_Measurement.V_AB_Line;
   //V_B_at_cap1isr = (int)AC_Line_Measurement.V_BC_Line;
   //V_C_at_cap1isr = (int)AC_Line_Measurement.V_CA_Line;
   //V_A_at_cap1isr = (int)AC_Line_Measurement.V_A;
   //V_B_at_cap1isr = (int)AC_Line_Measurement.V_B;
   //V_C_at_cap1isr = (int)AC_Line_Measurement.V_C;

	switch(PLL_Data.PLL_State) {
		case dFREQUENCY_UNLOCK:

			// Frequency tracking
			Phase_Lock = FALSE;
			PLL_Operation_Cnt ++ ;
			if (PLL_Operation_Cnt >= 100) {
				PLL_Operation_Cnt = 0;
				//--- Modify sampling frequency ---
				OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency;
				fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
				OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
				EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

				phase_detect_cnt = 0;
				PLL_Data.PLL_State = dFREQUENCY_LOCK;
			}
			break;

		case dFREQUENCY_LOCK:
			//--- Detect phase rotation ---
			//V_A_at_cap1isr = (int)AC_Line_Measurement.V_A;
			//V_B_at_cap1isr = (int)AC_Line_Measurement.V_B;
			//V_C_at_cap1isr = (int)AC_Line_Measurement.V_C;
			//if (V_A_at_cap1isr < -50 && V_B_at_cap1isr > 50 && V_C_at_cap1isr > 10) {
			//if (V_A_at_cap1isr < -50 && V_B_at_cap1isr > 50) {
			//	phase_detect_cnt ++ ;
			//	if (phase_detect_cnt >= 20) {
			//		PLL_Data.PhaseRotation_ABC = TRUE;
			//		system.setting |= LINE_PH_ROTATION_ABC;
			//	}
			//} else {
			//	phase_detect_cnt ++ ;
			//	if (phase_detect_cnt >= 20) {
			//		PLL_Data.PhaseRotation_ABC = FALSE;
			//		system.setting &= ~LINE_PH_ROTATION_ABC;
			//	}
			//}
			Phase_Lock = FALSE;
			PLL_Operation_Cnt ++ ;
			if (PLL_Operation_Cnt >= 100) {
				PLL_Operation_Cnt = 0;
				//--- Modify sampling frequency ---
				OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency;
				fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
				OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
				EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

				PLL_Data.PLL_State = dPHASE_UNLOCK;
			}
			break;

		case dPHASE_UNLOCK:
			Phase_Lock = FALSE;
			PLL_Operation_Cnt ++ ;
			if (PLL_Operation_Cnt >= OperationData.Period_Cnt || (int)OperationData.EPwmTimerIntCount <= 1) {
				PLL_Operation_Cnt = 0;
				//--- Modify sampling frequency ---
				OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency;
				fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
				OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
				EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

				PLL_Data.PLL_State = dPHASE_LOCK_WORKING;
				PLL_Operation_Cnt = 0;
			} else {
				if ((int)OperationData.EPwmTimerIntCount > 1) {
					OperationData.EPwmTimerIntCount -= 1.0;
				}
			}
			break;

		case dPHASE_LOCK_WORKING:
			Phase_Lock = TRUE;


			//--- PI control for PLL ---
			PLL_Data.PLL_Compensator = 10.0 * PLL_Data.PLL_error * PLL_Data.PLL_Kp;		// Proportional 
			PLL_Data.PLL_error_int = 10.0 * PLL_Data.PLL_error * PLL_Data.PLL_Ki;		// Integral
			PLL_Data.PLL_error_int += PLL_Data.PLL_error_int_old;						// Integral
			PLL_Data.PLL_Compensator += PLL_Data.PLL_error_int;

			PLL_Data.PLL_error_int_old = PLL_Data.PLL_error_int;

			//--- Modify sampling frequency ---
			OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency + PLL_Data.PLL_Compensator;
			fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
			OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
			EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
			EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
			EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

			OperationData.EPwmTimerIntCount = -PLL_Data.PLL_error;		  // ----!!!---
			//OperationData.EPwmTimerIntCount = PLL_Data.PLL_Compensator;		  // ----!!!---
			//OperationData.EPwmTimerIntCount = (unsigned int)(PLL_PHASE_ADJUSTMENT * OperationData.Period_Cnt);  OperationData.PhaseCommand

			if ((int)AC_Line_Measurement.V_Line_Q < 100 && (int)AC_Line_Measurement.V_Line_Q > -100) {
				PLL_Operation_Cnt ++;
				if ((int)AC_Line_Measurement.V_Line_Q > 0){
					OperationData.PhaseCommand += 0.1;
					if ((int)AC_Line_Measurement.V_Line_Q < 5) {
						PLL_Data.PLL_State = dPHASE_LOCK;
						PLL_Operation_Cnt = 0;
					}
				}
				if ((int)AC_Line_Measurement.V_Line_Q < 0){
					OperationData.PhaseCommand -= 0.1;
					if ((int)AC_Line_Measurement.V_Line_Q > -5) {
						PLL_Data.PLL_State = dPHASE_LOCK;
						PLL_Operation_Cnt = 0;
					}
				}
			} else {
				PLL_Operation_Cnt ++;
				if (PLL_Operation_Cnt > 1000) {
					PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
					PLL_Operation_Cnt = 0;
				}
			}

			break;

		case dPHASE_LOCK:
			Phase_Lock = TRUE;

			//--- PI control for PLL ---
			PLL_Data.PLL_Compensator = PLL_Data.PLL_error * PLL_Data.PLL_Kp;	// Proportional 
			PLL_Data.PLL_error_int = PLL_Data.PLL_error * PLL_Data.PLL_Ki;		// Integral
			PLL_Data.PLL_error_int += PLL_Data.PLL_error_int_old;				// Integral
			PLL_Data.PLL_Compensator += PLL_Data.PLL_error_int;

			PLL_Data.PLL_error_int_old = PLL_Data.PLL_error_int;

			//--- Modify sampling frequency ---
			OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency + PLL_Data.PLL_Compensator;
			fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
			OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
			EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
			EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
			EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

			OperationData.EPwmTimerIntCount = -PLL_Data.PLL_error;		  // ----!!!---
			//OperationData.EPwmTimerIntCount = PLL_Data.PLL_Compensator;		  // ----!!!---
			//OperationData.EPwmTimerIntCount = (unsigned int)(PLL_PHASE_ADJUSTMENT * OperationData.Period_Cnt);
        	system.status |= PLL_LOCK;

			iTemp0 = (int)AC_Line_Measurement.V_Line_Q; 
			iTemp1 = PLL_Data.Phase_Offset + 5;
			iTemp2 = PLL_Data.Phase_Offset - 5;
			if (iTemp0 > iTemp1){
				OperationData.PhaseCommand += 0.01;
			} 
			if (iTemp0 < iTemp2){
				OperationData.PhaseCommand -= 0.01;
			}

			iTemp1 = PLL_Data.Phase_Offset + 40;
			iTemp2 = PLL_Data.Phase_Offset - 40;
			if (iTemp0 > iTemp1 || iTemp0 < iTemp2) {
				PLL_Operation_Cnt ++;
				if (PLL_Operation_Cnt > 1000) {
					PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
        			system.status &= ~PLL_LOCK;
					PLL_Operation_Cnt = 0;
				}
			} else {
				PLL_Operation_Cnt = 0;
			}
			break;

	}
}
//________________________________________________________________________________
// Type:	function
// Name:	ELine_PLL_QCtrl
//
// Description:
// ------------
// In stand-by state, if E line is present, PLL control will replace P or F vs
// frequency control.

void ELine_PLL_QCtrl(){
float	fTemp;
int		iTemp0, iTemp1;

	switch(PLL_Data.PLL_State) {
		case dFREQUENCY_UNLOCK:

			// Frequency tracking
			Phase_Lock = FALSE;
			PLL_Operation_Cnt ++ ;
			if (PLL_Operation_Cnt >= 5000) {
				PLL_Operation_Cnt = 0;
				//--- Modify sampling frequency ---
				iACLIne_F_delta = (int)(100.0 * ACLine.Frequency - 100.0 * EERPOM_AC_Frequency);
				iACLIne_F_delta = abs(iACLIne_F_delta);
				if (iACLIne_F_delta	> 50) {
					OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency;
				} else {
					OperationData.PWM_Frequency = OperationData.Period_Cnt * (float)EERPOM_AC_Frequency;
				}
				fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
				OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
				EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

				phase_detect_cnt = 0;
				PLL_Data.PLL_State = dFREQUENCY_LOCK;
				PLL_Data.PLL_Kp = 10.0;
				PLL_Data.PLL_Ki = 0.02;
			}
			break;

		case dFREQUENCY_LOCK:
			Phase_Lock = FALSE;
			PLL_Operation_Cnt ++ ;
			if (PLL_Operation_Cnt >= 5000) {
				PLL_Operation_Cnt = 0;
				//--- Modify sampling frequency ---
				iACLIne_F_delta = (int)(100.0 * ACLine.Frequency - 100.0 * EERPOM_AC_Frequency);
				iACLIne_F_delta = abs(iACLIne_F_delta);
				if (iACLIne_F_delta	> 50) {
					OperationData.PWM_Frequency = OperationData.Period_Cnt * ACLine.Frequency;
				} else {
					OperationData.PWM_Frequency = OperationData.Period_Cnt * (float)EERPOM_AC_Frequency;
				}
				fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
				OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
				EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
				EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update

				PLL_Data.PLL_Compensator = 0.0;
				PLL_Data.PLL_error_int = 0.0;
				PLL_Data.PLL_error_int_old = 0.0;

				PLL_Data.PLL_State = dPHASE_UNLOCK;
			}
			break;

		case dPHASE_UNLOCK:
			PLL_Data.PLL_error = PLL_Data.Q_command - AC_Line_Measurement.V_Line_Q;
			fTemp = -0.0005 * PLL_Data.PLL_error;
			// PLL intergral term:
			PLL_Data.PLL_error_int = PLL_Data.PLL_Ki * fTemp;
			PLL_Data.PLL_error_int += PLL_Data.PLL_error_int_old;
			iTemp0 = (int)(10000.0 * PLL_Data.PLL_error_int);
			if (iTemp0 > 200)	PLL_Data.PLL_error_int = 0.02;
			if (iTemp0 < -200)	PLL_Data.PLL_error_int = -0.02;
			PLL_Data.PLL_error_int_old = PLL_Data.PLL_error_int;
			// PLL proportional term:
			PLL_Data.PLL_Compensator = PLL_Data.PLL_Kp * fTemp;
			// PLL output
			PLL_Data.PLL_Compensator += PLL_Data.PLL_error_int;
			iTemp1 = (int)(10000.0 * PLL_Data.PLL_Compensator);
			if (iTemp1 > 200)	PLL_Data.PLL_Compensator = 0.02;
			if (iTemp1 < -200)	PLL_Data.PLL_Compensator = -0.02;

			if ((int)PLL_Data.PLL_error < 40 && (int)PLL_Data.PLL_error > -40) {
				PLL_Operation_Cnt ++;
				if (PLL_Operation_Cnt >= 20000){
					PLL_Data.PLL_State = dPHASE_LOCK;
					Phase_Lock = TRUE;
        			system.status |= PLL_LOCK;
					PLL_Operation_Cnt = 0;
					PLL_Data.PLL_Kp = 1.0;			// 10/24/14
					PLL_Data.PLL_Ki = 0.001;
				}
			} else {
				PLL_Operation_Cnt = 0;
			}

			if ((int)PLL_Data.PLL_error > 60 && (int)PLL_Data.PLL_error < -60) {
				PLL_Operation_Cnt ++;
				if (PLL_Operation_Cnt >= 2000){
					PLL_Data.PLL_State = dFREQUENCY_LOCK;
					PLL_Operation_Cnt = 0;
				}
			}

			break;

		case dPHASE_LOCK:
			PLL_Data.PLL_error = PLL_Data.Q_command - AC_Line_Measurement.V_Line_Q;
			fTemp = -0.001 * PLL_Data.PLL_error;
			// PLL intergral term:
			PLL_Data.PLL_error_int = PLL_Data.PLL_Ki * fTemp;
			PLL_Data.PLL_error_int += PLL_Data.PLL_error_int_old;
			iTemp0 = (int)(10000.0 * PLL_Data.PLL_error_int);
			if (iTemp0 > 100)	PLL_Data.PLL_error_int = 0.01;
			if (iTemp0 < -100)	PLL_Data.PLL_error_int = -0.01;
			PLL_Data.PLL_error_int_old = PLL_Data.PLL_error_int;
			// PLL proportional term:
			PLL_Data.PLL_Compensator = PLL_Data.PLL_Kp * fTemp;
			// PLL output
			PLL_Data.PLL_Compensator += PLL_Data.PLL_error_int;
			iTemp1 = (int)(10000.0 * PLL_Data.PLL_Compensator);
			if (iTemp1 > 100)	PLL_Data.PLL_Compensator = 0.01;
			if (iTemp1 < -100)	PLL_Data.PLL_Compensator = -0.01;

			if ((int)PLL_Data.PLL_error > 20 && (int)PLL_Data.PLL_error < -20) {
				PLL_Operation_Cnt ++;
				if (PLL_Operation_Cnt >= 5000){
					PLL_Data.PLL_State = dFREQUENCY_UNLOCK;
					Phase_Lock = FALSE;
        			system.status &= ~PLL_LOCK;
					PLL_Operation_Cnt = 0;
				}
			} else {
				PLL_Operation_Cnt = 0;
			}

			break;

	}
}
//===========================================================================
// End of file
//===========================================================================


