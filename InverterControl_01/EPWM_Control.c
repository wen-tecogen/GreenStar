/**********************************************************************************
// File: EPWM_Control.c
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
// This module contains the PWM real-time control functions.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/8/12	|  J Wen 	| Original
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
#include "EPWM_control.h"       // Header file for ePWM module
#include "ECAP_VLine.h"     	// Module Include File
#include "SystemControl.h"      // Device Header file and Examples Include File
#include "ADC.h"				// Header file for A to D converter
#include "Grid_Control.h"		// Header file for grid control
#include "UL1741.h"				// Header file for UL1741
#include "DataMeasurement.h"       // Header file for ePWM module

/******* define grid-tie operation mode *******/
#define	GT_SINE_PWM			0
#define	GT_SV_PWM			1
//#define GT_OPEN_LOOP_TEST  	1
//#define GT_5TH_HARM_OPEN_LOOP_TEST  	1

/******* Swi handle defined in swi.cfg *******/
extern const Swi_Handle swi0;

/******* Function prototype *******/
interrupt void epwm_isr(void);
interrupt void epwm_tzint_isr(void);
extern interrupt void ecap1_isr(void);
extern interrupt void ecap4_isr(void);
extern interrupt void ecan0intb_isr(void);    // eCAN-B system
extern interrupt void ecan1intb_isr(void);    // eCAN-B message
extern void DAC_output(void);
extern void MicroGrid_ctrl(void);
extern void DO_1_On(void);

void EPWM_Setup(void);
void InitEPwm1Setup(void);
void InitEPwm2Setup(void);
void InitEPwm3Setup(void);
void InitEPwm4Setup(void);
void InitEPwm5Setup(void);
void InitEPwm6Setup(void);
void update_compare(EPWM_INFO*);
void Inverter_Control_MG(void);
void Inverter_Control_GT(void);
void InitEPwm1TripZone(void);
void InitEPwm2TripZone(void);
void InitEPwm2TripZone_Boost(void);
void InitEPwm2TripZone_2ndBoost(void);
void InitEPwm2TripZone_Brake(void);
void EPWM1_On(void);
void EPWM1_Off(void);
void EPWM_Boost_On(void);
void EPWM_Boost_Off(void);
void EPWM_Boost2_On(void);
void EPWM_Boost2_Off(void);
void EPWM_Brake_On(void);
void EPWM_Brake_Off(void);
void BoostControl(void);
void Inverter_V_CloseLoop(void);
void Inverter_I_CloseLoop_GT(void);
void Sine_PWM_GT(void);
void SV_PWM_GT(void);
void GT_Harmonic_Ctrl(void);
void Inverter_I_CloseLoop_MG(void);
void GT_Anti_Islanding(void);
void GT_Anti_Islanding_init(void);

// Global variables used in this module

EPWM_INFO 		epwm1_info;
EPWM_INFO 		epwm2_info;
EPWM_INFO 		epwm3_info;
EPWM_INFO 		epwm4_info;
EPWM_INFO 		epwm5_info;
OPERATION_DATA 	OperationData;
BOOST_CONTROL	Boost_Control;
INVERTER_CONTROL Inverter_SysControl;
ANTI_ISLANDING	Anti_Islanding;

GRID_TIE		iGrid_Tie;
I_PARK			iPark;
SV_GEN			svgen1;

//float sine_value, sine120_value, sine240_value, sine_ptr, sine120_ptr, sine240_ptr;
float 	softstart_cmd;
float 	SV_softstart;
Uint16 	softstart_inc;
Uint16 	GT_softstart_inc;
Uint16 	GT_CtrlLoop_Cnt;
Uint16 	GT_CtrlLoop_TIME;

Uint16 	MG_Ctrl;
int     iTemp_var;
int		Boost_Currentx10;
int		Boost_OI_Count;
// Grid-Tie individual phase control
float GT_cl3_int_PhaseA, GT_cl3_int_PhaseB, GT_cl3_int_PhaseC;
float I_cmd_PhaseA, I_cmd_PhaseB, I_cmd_PhaseC;
float GT_cl3_I_error_PhaseA, GT_cl3_I_error_PhaseB, GT_cl3_I_error_PhaseC;
float GT_cl3_I_int1_PhaseA, GT_cl3_I_int1_PhaseB, GT_cl3_I_int1_PhaseC;
float I_PhaseC;

// External variables
extern float DC_BUS_1;
extern float Boost_Current;
extern SINE_REF			sine_ref;
extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern AC_MEASURE_DATA 	VAC_out_Measurement;		// Inverter Output data
extern PLL_DATA	PLL_Data;
extern GT_CURRENT_HARMONICS	GT_5th_harm_ctrl;
extern GT_CURRENT_HARMONICS	GT_2nd_harm_ctrl;
extern MICRO_GRID	MicroGrid;
extern SYSTEM_INFO 	system;
extern SPEED_CONTROL speed_control;
extern int	 iGT_ID_command;
extern int	 InputStageFilter;

//
//  ======== EPWM_Setup ========
//  Setup PWM channels
//
Void EPWM_Setup() {
    // Initialize GPIO pins for ePWM1, ePWM2, ePWM3 for inverter drive
    // Initialize GPIO pins for ePWM4, ePWM5, ePWM6 for DC/DC converter drive, and brake
    // These functions are in the DSP2834x_EPwm_Setup.c file
    InitEPwm1Gpio();
    InitEPwm2Gpio();
    InitEPwm3Gpio();
    InitEPwm4Gpio();
    InitEPwm5Gpio();
    InitEPwm6Gpio();
    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Interrupts that are used in this project are re-mapped to
    // ISR functions found within this file.
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.EPWM1_INT = &epwm_isr;
    //PieVectTable.EPWM2_INT = &epwm2_isr;
    //PieVectTable.EPWM3_INT = &epwm3_isr;
    PieVectTable.ECAP1_INT = &ecap1_isr;
    PieVectTable.ECAP4_INT = &ecap4_isr;
    PieVectTable.EPWM1_TZINT = &epwm_tzint_isr;
    PieVectTable.ECAN1INTB = &ecan1intb_isr;
    PieVectTable.ECAN0INTB = &ecan0intb_isr;
    //PieVectTable.I2CINT1A = &i2c_int1a_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 0;
    EDIS;

    //===== Variables Initialization =====
    OperationData.Frequency = 60.0;	//60Hz, use for initialize only
    OperationData.PWM_Frequency = CPU_FREQUENCY / (2*EPWM_TIMER_TBPRD) + 1;
    //OperationData.Period_Cnt = (unsigned int)OperationData.PWM_Frequency / OperationData.Frequency;
    OperationData.Period_Cnt = NUMBER_OF_STEP_PER_60HZ_CYCLE;
    OperationData.PWM_Frequency = OperationData.Period_Cnt * OperationData.Frequency;
    OperationData.DeltaPhase = 2*PI/3;
    OperationData.EPwmTZ_Trip = FALSE;
    //OperationData.VoltageCommand = 480.0;	 // Read from EEPROM
    OperationData.ID_out_cmd = 25.0;
    OperationData.IQ_out_cmd = 0.0;
	iGT_ID_command = (int)OperationData.ID_out_cmd;
    //OperationData.VoltageCommand = SYSTEM_OUTPUT_VOLTAGE_NOMINAL;
	OperationData.EPwmTimerIntCount = 0.0;

    //===== Boost control initialization =====
    Boost_Control.DCBUS_cmd = 750.0;
    Boost_Control.DCBUS_cmd_Active = 0.0;
    Boost_Control.DCBUS_I_Limit_x10 = 3700;
    Boost_Control.DCBUS_OverCurrent_x10 = 1700;
    Boost_Control.V_Control_Kp = 10.0;		// For vloop only, Kp start with 30
    Boost_Control.V_Control_Ki = 0.4;		// For vloop only, Ki start with 0.4
    Boost_Control.I_Control_Kp = 15.0;		//8.0;		15.0;	   For 8.2KHz PWM frequency
    Boost_Control.I_Control_Ki = 0.05;		//0.5;		0.01;	   12/2/14
    Boost_Control.DCBUS_V_error_int = 0.0;
    Boost_Control.DCBUS_V_error_int_old = 0.0;
    Boost_Control.DCBUS_I_error_int = 0.0;
    Boost_Control.DCBUS_I_error_int_old = 0.0;
	Boost_OI_Count = 0;
    
    //===== Voltage regulator control initialization =====
	Inverter_SysControl.V_error_int	= 0.0;
	Inverter_SysControl.V_error_int_old	= 0.0;
	Inverter_SysControl.I_error_int = 0.0;
	Inverter_SysControl.I_error_int_old = 0.0;
	Inverter_SysControl.I_Control_Kp = 0.2;
	Inverter_SysControl.I_Control_Ki = 0.08;
	Inverter_SysControl.V_Control_Kp = 0.1;
	Inverter_SysControl.V_Control_Ki = 0.002;
	Inverter_SysControl.InverterOn = FALSE;

	MG_Ctrl = 0;

    //===== Grid-Tie control initialization =====
	Inverter_SysControl.GT_I_Control_Kp = 2.0;				//10.0;
	Inverter_SysControl.GT_I_Control_Ki = 0.1;				//1.0;
	Inverter_SysControl.GT_I_Control_Kc = 0.5;				
	Inverter_SysControl.GT_I_Control_Kd = 2.5;				
	Inverter_SysControl.GT_ID_error_int	= 0.0;
	Inverter_SysControl.GT_ID_error_int_old = 0.0;
	Inverter_SysControl.GT_IQ_error_int	= 0.0;
	Inverter_SysControl.GT_IQ_error_int_old = 0.0;
	Inverter_SysControl.GT_IDout_error = 0.0;
	Inverter_SysControl.GT_IQout_error = 0.0;
	Inverter_SysControl.GT_ID_error_der = 0.0;
	Inverter_SysControl.GT_ID_error_pro = 0.0;
	Inverter_SysControl.GT_ID_error_pro1 = 0.0;
	Inverter_SysControl.GT_IDout_SatErr = 0.0;
	Inverter_SysControl.GT_IQ_error_der = 0.0;
	Inverter_SysControl.GT_IQ_error_pro = 0.0;
	Inverter_SysControl.GT_IQ_error_pro1 = 0.0;
	Inverter_SysControl.GT_IQout_SatErr = 0.0;

    //===== Grid-Tie 5th harmonic control initialization =====
	GT_5th_harm_ctrl.I_D_integral = 0.0;
	GT_5th_harm_ctrl.I_Q_integral = 0.0;
	GT_5th_harm_ctrl.I_D_err = 0.0;
	GT_5th_harm_ctrl.I_Q_err = 0.0;
	GT_5th_harm_ctrl.I_D_Fil = 0.0;
	GT_5th_harm_ctrl.I_Q_Fil = 0.0;
	GT_5th_harm_ctrl.Kp = 0.8;
	GT_5th_harm_ctrl.Ki = 0.01;
	GT_5th_harm_ctrl.Filter_K1 = 0.5;
	GT_5th_harm_ctrl.Filter_K2 = 0.5;

    //===== Grid-Tie 2nd harmonic control initialization =====
	GT_2nd_harm_ctrl.I_D_integral = 0.0;
	GT_2nd_harm_ctrl.I_Q_integral = 0.0;
	GT_2nd_harm_ctrl.I_D_err = 0.0;
	GT_2nd_harm_ctrl.I_Q_err = 0.0;
	GT_2nd_harm_ctrl.I_D_Fil = 0.0;
	GT_2nd_harm_ctrl.I_Q_Fil = 0.0;
	GT_2nd_harm_ctrl.Kp = 0.02;
	GT_2nd_harm_ctrl.Ki = 0.001;
	GT_2nd_harm_ctrl.Filter_K1 = 0.05;
	GT_2nd_harm_ctrl.Filter_K2 = 0.95;

	iGrid_Tie.Feed_Forward = 83;
	iGrid_Tie.SoftStart_cnt = 500;
	softstart_inc = 0;
	GT_softstart_inc = 0;
	GT_CtrlLoop_Cnt = 0;
	GT_CtrlLoop_TIME = 2;
	//iGrid_Tie.SV_sine = GT_SINE_PWM;
	iGrid_Tie.HarmonicControl = 1;		// GT PWM mode: 1 = enable, 0 = disable
	SV_softstart = 1.0;
	iGrid_Tie.ThirdHarmonic = 18;
	iGrid_Tie.D_init = 1.0;					//0.16;
	iGrid_Tie.Q_init = 0.5;					//0.05;
	// individual phase control
	GT_cl3_I_error_PhaseA = 0.0;
	GT_cl3_I_error_PhaseB = 0.0;
	GT_cl3_I_error_PhaseC = 0.0;
	GT_cl3_int_PhaseA = 0.0;
	GT_cl3_int_PhaseB = 0.0;
	GT_cl3_int_PhaseC = 0.0;
	GT_cl3_I_int1_PhaseA = 0.0;
	GT_cl3_I_int1_PhaseB = 0.0;
	GT_cl3_I_int1_PhaseC = 0.0;

	GT_Anti_Islanding_init();
    //===== Setup PWM channels =====
    EALLOW;
    InitEPwm1Setup();
    InitEPwm2Setup();
    InitEPwm3Setup();
    InitEPwm4Setup();
    InitEPwm5Setup();
    InitEPwm6Setup();
    EDIS;
    InitEPwm1TripZone();
    InitEPwm2TripZone();
    EPWM1_Off();
    EPWM_Boost_Off();

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    EDIS;

    // Enable CPU INT2 which is connected to EPWM1-3 INT:
    IER |= M_INT2;
    // Enable CPU INT3 which is connected to EPWM1-3 INT:
    IER |= M_INT3;
    // Enable CPU INT4 which is connected to ECAP1-4 INT:
    IER |= M_INT4;
    // Enable CPU INT8 which is connected to PIE group 8
    //IER |= M_INT8;
    // Enable CPU INT9 which is connected to ECANB1-0 INT:
    IER |= M_INT9;

    // Enable EPWM INTn in the PIE: Group 3 interrupt 1-3
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx2 = 1;
    PieCtrlRegs.PIEIER3.bit.INTx3 = 1;
    // Enable eCAP INTn in the PIE: Group 4 interrupt 1
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;
    // Enable EPWM ZT INTn in the PIE: Group 2 interrupt 1
    PieCtrlRegs.PIEIER2.bit.INTx1 = 1;
    // Enable I2C interrupt 1 in the PIE: Group 8 interrupt 1
    //PieCtrlRegs.PIEIER8.bit.INTx1 = 1;
    // Enable ECANB INTn in the PIE: Group 9 interrupt 8-7
    PieCtrlRegs.PIEIER9.bit.INTx8 = 1;
    PieCtrlRegs.PIEIER9.bit.INTx7 = 1;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global real-time interrupt DBGM

}

//
//  ======== PWM_swi ========
//  Software interrupt function that to perform
//  real-time control functions.
//
Void PWM_swi(UArg arg) {
	DAC_output();
	if (Inverter_SysControl.SystemOperationMode == MICRO_GRID_MODE){
   		Inverter_Control_MG();
	} else {
   		Inverter_Control_GT();
	}
//   GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;		// Profile isr
}

//
//  ======== epwm_isr ========
//  EPWM isr function to post a swi real-time control functions.
//
interrupt void epwm_isr(void) {
//	GpioDataRegs.GPASET.bit.GPIO30 = 1; 		// Profile isr
	// Update the CMPA and CMPB values
	//update_compare(&epwm1_info);
	Swi_post(swi0);

	// Clear INT flag for this timer
	EPwm1Regs.ETCLR.bit.INT = 1;

	// Acknowledge this interrupt to receive more interrupts from group 3
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
}

//
//  ======== epwm1_tzint_isr ========
//  Trip-Zone isr function.
//
interrupt void epwm_tzint_isr(void) {
	// Leave these flags set so we only take this
	// interrupt once
    OperationData.EPwmTZ_Trip = TRUE;

    // Acknowledge this interrupt to receive more interrupts from group 2
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP2;
}

//
//  ======== InitEPwm1Setup ========
//  Setup PWM 1.
//
void InitEPwm1Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm1Regs.TBPRD = EPWM_TIMER_TBPRD; 			// Period = 2*14481 TBCLK counts (10.08KHz)
	EPwm1Regs.CMPA.half.CMPA = EPWM_MAX_CMPA; 		// Compare A = 350 TBCLK counts
	EPwm1Regs.CMPB = EPWM_MIN_CMPB; 				// Compare B = 400 TBCLK counts

	EPwm1Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm1Regs.TBCTR = 0; 							// clear TB counter

	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm1Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE; 		// TB_SHADOW;

	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO;		// TB_SYNC_DISABLE;
	EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	//---- PWM proiority setup ----
	// PWMxA drives top switch, active high
	// PWMxB drives bottom switch, active low
	EPwm1Regs.AQCTLB.bit.CAU = AQ_SET;
	EPwm1Regs.AQCTLB.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.CBU = AQ_CLEAR;
	EPwm1Regs.AQCTLA.bit.CBD = AQ_SET;
	//EPwm1Regs.AQCTLA.bit.CAU = AQ_SET;
	//EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	//EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	//EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	EPwm1Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;	// enable Dead-band module
	EPwm1Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 		// Active Hi complementary
	EPwm1Regs.DBFED = 450; 							// FED = 150 TBCLKs (0.5usec)
	EPwm1Regs.DBRED = 450; 							// RED = 150 TBCLKs (0.5usec)

	//---- ADC SoC setup ----
	EPwm1Regs.ETSEL.bit.SOCBEN = 0x1;   			// Enable extsoc1b event
	EPwm1Regs.ETSEL.bit.SOCBSEL = 0x2;  			// Set event to happen on TBCTR = TBPRD
	EPwm1Regs.ETPS.bit.SOCBPRD = 0x1;   			// Generate SoC on first event

	SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1BEN = 0x1;     // Enable extsoc1b
	SysCtrlRegs.EXTSOCCFG.bit.EXTSOC1BPOLSEL= 0x1;  // Set inverted polarity (CONVST is active low)
	// Setup SOCADCCLK to run at 25Mhz
	//SysCtrlRegs.HISPCP.bit.HSPCLK = 0x5;			// HSPCLK=SYSCLKOUT/12 (25Mhz)

	// Interrupt where we will change the Compare Values
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     	// Select INT on Zero event
	EPwm1Regs.ETSEL.bit.INTEN = 1;                	// Enable INT
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;           	// Generate INT on every event

	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm1Regs.CMPA.half.CMPA = EPWM_MIN_CMPA; 		// adjust duty for output EPWM1A
	EPwm1Regs.CMPB = EPWM_MIN_CMPB; // adjust duty for output EPWM1B


   // Information this example uses to keep track
   // of the direction the CMPA/CMPB values are
   // moving, the min and max allowed values and
   // a pointer to the correct ePWM registers
   epwm1_info.EPwmTimerIntCount = 0;                // Zero the interrupt counter
   epwm1_info.EPwmRegHandle = &EPwm1Regs;           // Set the pointer to the ePWM module
   epwm1_info.EPwmCMP = EPWM_MIN_CMPA;
   epwm1_info.Phase = 0.0;
   epwm1_info.EPwmGAIN = 0.5;
   //epwm1_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}

//
//  ======== InitEPwm2Setup ========
//  Setup PWM 2.
//
void InitEPwm2Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm2Regs.TBPRD = EPWM_TIMER_TBPRD; 			// Period = 2*14481 TBCLK counts (10.08KHz)
	EPwm2Regs.CMPA.half.CMPA = EPWM_MAX_CMPA; 		// Compare A = 350 TBCLK counts
	EPwm2Regs.CMPB = EPWM_MIN_CMPB; 				// Compare B = 400 TBCLK counts

	EPwm2Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm2Regs.TBCTR = 0; 							// clear TB counter

	EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm2Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm2Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;		// TB_SHADOW;

	EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN;		// TB_SYNC_DISABLE;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm2Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	//---- PWM proiority setup ----
	// PWMxA drives top switch, active high
	// PWMxB drives bottom switch, active low
	EPwm2Regs.AQCTLB.bit.CAU = AQ_SET;
	EPwm2Regs.AQCTLB.bit.CAD = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.CBU = AQ_CLEAR;
	EPwm2Regs.AQCTLA.bit.CBD = AQ_SET;
	//EPwm2Regs.AQCTLA.bit.CAU = AQ_SET;
	//EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	//EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	//EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	EPwm2Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// enable Dead-band module
	EPwm2Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 	// Active Hi complementary
	EPwm2Regs.DBFED = 450; 						// FED = 150 TBCLKs (0.5usec)
	EPwm2Regs.DBRED = 450; 						// RED = 150 TBCLKs (0.5usec)

	// Interrupt where we will change the Compare Values
	//EPwm2Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	//EPwm2Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	//EPwm2Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event

	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm2Regs.CMPA.half.CMPA = EPWM_MIN_CMPA; 		// adjust duty for output EPWM1A
	EPwm2Regs.CMPB = EPWM_MIN_CMPB; 				// adjust duty for output EPWM1B

	// Information this example uses to keep track
	// of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and
	// a pointer to the correct ePWM registers
	epwm2_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	epwm2_info.EPwmRegHandle = &EPwm2Regs;          // Set the pointer to the ePWM module
	epwm2_info.EPwmCMP = EPWM_MIN_CMPA;
	epwm2_info.Phase = 2 * OperationData.Period_Cnt / 3;
	epwm2_info.EPwmGAIN = 0.5;
	//epwm2_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}

//
//  ======== InitEPwm3Setup ========
//  Setup PWM 3.
//
void InitEPwm3Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm3Regs.TBPRD = EPWM_TIMER_TBPRD; 			// Period = 2*14481 TBCLK counts (10.08KHz)
	EPwm3Regs.CMPA.half.CMPA = EPWM_MAX_CMPA; 		// Compare A = 350 TBCLK counts
	EPwm3Regs.CMPB = EPWM_MIN_CMPB; 				// Compare B = 400 TBCLK counts

	EPwm3Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm3Regs.TBCTR = 0; 							// clear TB counter

	EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm3Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm3Regs.TBCTL.bit.PRDLD = TB_IMMEDIATE;		// TB_SHADOW;

	EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; 		// TB_SYNC_DISABLE;
	EPwm3Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm3Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	//---- PWM proiority setup ----
	// PWMxA drives top switch, active high
	// PWMxB drives bottom switch, active low
	EPwm3Regs.AQCTLB.bit.CAU = AQ_SET;
	EPwm3Regs.AQCTLB.bit.CAD = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.CBU = AQ_CLEAR;
	EPwm3Regs.AQCTLA.bit.CBD = AQ_SET;
	//EPwm3Regs.AQCTLA.bit.CAU = AQ_SET;
	//EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	//EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	//EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// enable Dead-band module
	EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 	// Active Hi complementary
	EPwm3Regs.DBFED = 450; 						// FED = 150 TBCLKs (0.5usec)
	EPwm3Regs.DBRED = 450; 						// RED = 150 TBCLKs (0.5usec)

	// Interrupt where we will change the Compare Values
	//EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	//EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	//EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm3Regs.CMPA.half.CMPA = EPWM_MIN_CMPA; 		// adjust duty for output EPWM1A
	EPwm3Regs.CMPB = EPWM_MIN_CMPB; 				// adjust duty for output EPWM1B

	// Information this example uses to keep track
	// of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and
	// a pointer to the correct ePWM registers
	epwm3_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	epwm3_info.EPwmRegHandle = &EPwm3Regs;          // Set the pointer to the ePWM module
	epwm3_info.EPwmCMP = EPWM_MIN_CMPA;
	epwm3_info.Phase = OperationData.Period_Cnt / 3;
	epwm3_info.EPwmGAIN = 0.5;
	//epwm3_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}

//
//  ======== InitEPwm4Setup ========
//  Setup PWM 4.
//
void InitEPwm4Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm4Regs.TBPRD = BOOST_PWM_PERIOD;		// Period = 2*14481 TBCLK counts (10.08KHz)
	EPwm4Regs.CMPA.half.CMPA = 9000; 		// Compare A = 350 TBCLK counts
	EPwm4Regs.CMPB = 9000; 					// Compare B = 400 TBCLK counts

	EPwm4Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm4Regs.TBCTR = 0; 							// clear TB counter

	EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm4Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;

	EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm4Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm4Regs.AQCTLA.bit.CAU = 0;					// set top IGBT off
	EPwm4Regs.AQCTLA.bit.CAD = 0;					// set top IGBT off
	EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	//EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// enable Dead-band module
	//EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 	// Active Hi complementary
	//EPwm3Regs.DBFED = 150; 						// FED = 150 TBCLKs (0.5usec)
	//EPwm3Regs.DBRED = 150; 						// RED = 150 TBCLKs (0.5usec)

	// Interrupt where we will change the Compare Values
	//EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	//EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	//EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm4Regs.CMPA.half.CMPA = 14000; 		// adjust duty for output EPWM1A
	EPwm4Regs.CMPB = 14000; 				// adjust duty for output EPWM1B

	// Information this example uses to keep track
	// of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and
	// a pointer to the correct ePWM registers
	epwm4_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	epwm4_info.EPwmRegHandle = &EPwm4Regs;          // Set the pointer to the ePWM module
	epwm4_info.EPwmCMP = EPWM_MIN_CMPA;
	epwm4_info.Phase = OperationData.Period_Cnt / 3;
	epwm4_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}

//
//  ======== InitEPwm5Setup ========
//  Setup PWM 5.
//
void InitEPwm5Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm5Regs.TBPRD = EPWM_TIMER_TBPRD; 			// Period = 2*14481 TBCLK counts (10.08KHz)
	EPwm5Regs.CMPA.half.CMPA = 9000; 		// Compare A = 350 TBCLK counts
	EPwm5Regs.CMPB = 9000; 				// Compare B = 400 TBCLK counts

	EPwm5Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm5Regs.TBCTR = 0; 							// clear TB counter

	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm5Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm5Regs.TBCTL.bit.PRDLD = TB_SHADOW;

	EPwm5Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm5Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm5Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm5Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm5Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm5Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm5Regs.AQCTLA.bit.CAU = 0;					// set top IGBT off
	EPwm5Regs.AQCTLA.bit.CAD = 0;					// set top IGBT off
	EPwm5Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm5Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	//EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// enable Dead-band module
	//EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 	// Active Hi complementary
	//EPwm3Regs.DBFED = 150; 						// FED = 150 TBCLKs (0.5usec)
	//EPwm3Regs.DBRED = 150; 						// RED = 150 TBCLKs (0.5usec)

	// Interrupt where we will change the Compare Values
	//EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	//EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	//EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm5Regs.CMPA.half.CMPA = 12000; 		// adjust duty for output EPWM1A
	EPwm5Regs.CMPB = 12000; 				// adjust duty for output EPWM1B

	// Information this example uses to keep track
	// of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and
	// a pointer to the correct ePWM registers
	epwm5_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	epwm5_info.EPwmRegHandle = &EPwm5Regs;          // Set the pointer to the ePWM module
	epwm5_info.EPwmCMP = EPWM_MIN_CMPA;
	epwm5_info.Phase = OperationData.Period_Cnt / 3;
	epwm5_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}

//
//  ======== InitEPwm6Setup ========
//  Setup PWM 6.
//
void InitEPwm6Setup() {

	// Initialization Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm6Regs.TBPRD = 60000; 						// Period = 0.4msec	  2.5KHz
	EPwm6Regs.CMPA.half.CMPA = 50000; 				// Compare A 
	EPwm6Regs.CMPB = 50000; 						// Compare B 

	EPwm6Regs.TBPHS.all = 0; 						// Set Phase register to zero
	EPwm6Regs.TBCTR = 0; 							// clear TB counter

	EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Symmetric
	EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE; 		// Phase loading disabled
	EPwm6Regs.TBCTL.bit.PRDLD = TB_SHADOW;

	EPwm6Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
	EPwm6Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1; 		// TBCLK = SYSCLKOUT
	EPwm6Regs.TBCTL.bit.CLKDIV = TB_DIV1;

	EPwm6Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm6Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; 	// load on CTR = Zero
	EPwm6Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; 	// load on CTR = Zero

	EPwm6Regs.AQCTLA.bit.CAU = 0;					// set top IGBT off
	EPwm6Regs.AQCTLA.bit.CAD = 0;					// set top IGBT off
	EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;
	EPwm6Regs.AQCTLB.bit.CBD = AQ_SET;

	//---- Dead band setup ----
	//EPwm3Regs.DBCTL.bit.OUT_MODE = DB_FULL_ENABLE;// enable Dead-band module
	//EPwm3Regs.DBCTL.bit.POLSEL = DB_ACTV_HIC; 	// Active Hi complementary
	//EPwm3Regs.DBFED = 150; 						// FED = 150 TBCLKs (0.5usec)
	//EPwm3Regs.DBRED = 150; 						// RED = 150 TBCLKs (0.5usec)

	// Interrupt where we will change the Compare Values
	//EPwm3Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;     // Select INT on Zero event
	//EPwm3Regs.ETSEL.bit.INTEN = 1;                // Enable INT
	//EPwm3Regs.ETPS.bit.INTPRD = ET_1ST;           // Generate INT on 3rd event


	// Run Time
	// = = = = = = = = = = = = = = = = = = = = = = = =
	EPwm6Regs.CMPA.half.CMPA = 50000; 		// adjust duty for output EPWM1A
	EPwm6Regs.CMPB = 50000; 				// adjust duty for output EPWM1B

	// Information this example uses to keep track
	// of the direction the CMPA/CMPB values are
	// moving, the min and max allowed values and
	// a pointer to the correct ePWM registers
	//epwm6_info.EPwmTimerIntCount = 0;               // Zero the interrupt counter
	//epwm6_info.EPwmRegHandle = &EPwm5Regs;          // Set the pointer to the ePWM module
	//epwm6_info.EPwmCMP = EPWM_MIN_CMPA;
	//epwm6_info.Phase = OperationData.Period_Cnt / 3;
	//epwm6_info.EPwmGAIN = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;

}


/*
 *  ======== InitEPwm1TripZone ========
 *  Setup PWM Trip-Zone.
 *  Use /TZ1(GPIO12) to trip all 3x2 PWM channels.
 *  Use PWM1 to generate interrupt.
 */
void InitEPwm1TripZone() {
	//---- PWM1 Trip-Zone Configurations ----
	EALLOW;
	EPwm1Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM1A will be forced low on a trip event
	EPwm1Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM1B will be forced high on a trip event

	//---- Enable PWM1 TZ interrupt ----
	EPwm1Regs.TZEINT.bit.OST = 1;
	EPwm1Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm1Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	//---- PWM2 Trip-Zone Configurations ----
	EPwm2Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM2A will be forced low on a trip event
	EPwm2Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM2B will be forced high on a trip event

	EPwm2Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm2Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	//---- PWM3 Trip-Zone Configurations ----
	EPwm3Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM3A will be forced low on a trip event
	EPwm3Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM3B will be forced high on a trip event

	EPwm3Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm3Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	EDIS;

	// Initial PWM variables:
    softstart_cmd = 0.0;
    softstart_inc = 0;

}

/*
 *  ======== InitEPwm2TripZone ========
 *  Setup PWM Trip-Zone for PWM4, 5 and PWM6.
 *  Use /TZ1(GPIO12) to trip all 5x2 PWM channels.
 *  Use PWM1 to generate interrupt.
 */
void InitEPwm2TripZone() {
	//---- PWM4 Trip-Zone Configurations ----
	EALLOW;
	EPwm4Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM4 TZ interrupt ----
	EPwm4Regs.TZEINT.bit.OST = 1;
	EPwm4Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm4Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	//---- PWM5 Trip-Zone Configurations ----
	EPwm5Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM5 TZ interrupt ----
	EPwm5Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm5Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	//---- PWM6 Trip-Zone Configurations ----
	EPwm6Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM6 TZ interrupt ----
	EPwm6Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm6Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	EDIS;
}

/*
 *  ======== InitEPwm2TripZone_Boost ========
 *  Setup PWM Trip-Zone for PWM4.
 */
void InitEPwm2TripZone_Boost() {
	//---- PWM4 Trip-Zone Configurations ----
	EALLOW;
	EPwm4Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm4Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm4Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM4 TZ interrupt ----
	EPwm4Regs.TZEINT.bit.OST = 1;
	EPwm4Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm4Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	EDIS;
}

/*
 *  ======== InitEPwm2TripZone_2ndBoost ========
 *  Setup PWM Trip-Zone for PWM5.
 */
void InitEPwm2TripZone_2ndBoost() {
	//---- PWM4 Trip-Zone Configurations ----
	EALLOW;
	EPwm5Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm5Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm5Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM4 TZ interrupt ----
	EPwm5Regs.TZEINT.bit.OST = 1;
	EPwm5Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm5Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	EDIS;
}

/*
 *  ======== InitEPwm2TripZone_Brake ========
 *  Setup PWM Trip-Zone for PWM6.
 */
void InitEPwm2TripZone_Brake() {
	//---- PWM4 Trip-Zone Configurations ----
	EALLOW;
	EPwm6Regs.TZSEL.bit.OSHT1 = TZ_ENABLE;			// enables TZ1 as a one-shot event
	EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// EPWM4A will be forced low on a trip event
	EPwm6Regs.TZCTL.bit.TZB = TZ_FORCE_LO;			// EPWM4B will be forced high on a trip event

	//---- Enable PWM4 TZ interrupt ----
	EPwm6Regs.TZEINT.bit.OST = 1;
	EPwm6Regs.TZCLR.bit.INT = 1;					// Clear interrupt flag
	EPwm6Regs.TZCLR.bit.OST = 1;					// Clear OST flag

	EDIS;
}

/*
 *  ======== GT_Anti_Islanding_init ========
 *  Initial anti-islanding variables.
 */
void GT_Anti_Islanding_init() {
	Anti_Islanding.Vref = 480.0;
	Anti_Islanding.err_cnt = 0;
	Anti_Islanding.V_error = 0.0;
	Anti_Islanding.ref_update = TRUE;
	Anti_Islanding.sensitivity = 100;
	Anti_Islanding.trip_time = 9000;
}
/*
 *  ======== Inverter_SysControl ========
 *  Inverter control in Micro-Grid mode.
 */
void Inverter_Control_MG() {
float	fTemp, fTemp_gain;
	OperationData.EPwmTimerIntCount += 1.0;
	OperationData.EPwmTimerIntCount += PLL_Data.PLL_Compensator;			// Use edge detect PLL, delete this line
	if ((unsigned int)OperationData.EPwmTimerIntCount >= OperationData.Period_Cnt) {
		OperationData.EPwmTimerIntCount = 0.0;
	}

//	GpioDataRegs.GPASET.bit.GPIO29 = 1; 		// LOAD DAC

	//===== sine reference generation =====
	sine_ref.ptr = 2 * PI * OperationData.EPwmTimerIntCount / (float)OperationData.Period_Cnt;
	sine_ref.ptr += OperationData.PhaseCommand;
	sine_ref.sin_0_a = sin(sine_ref.ptr);
	sine_ref.sin_3rd_harmonic = 0.18 * sin(3.0*sine_ref.ptr);
	sine_ref.cos_0_a = cos(sine_ref.ptr);

	sine_ref.ptr += OperationData.DeltaPhase;
	sine_ref.sin_P120_c = sin(sine_ref.ptr);
	sine_ref.cos_P120_c = cos(sine_ref.ptr);

	sine_ref.ptr += OperationData.DeltaPhase;
	sine_ref.sin_N120_b = sin(sine_ref.ptr);
	sine_ref.cos_N120_b = cos(sine_ref.ptr);
	//======================================

//	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1; 		// LOAD DAC
	//====== Stand-alone PWM soft start ======
	if (softstart_inc <= EPWM_SOFTSTART_CNT) {
		softstart_inc++;
		softstart_cmd += 1.0/EPWM_SOFTSTART_CNT;
		Inverter_SysControl.Vout_cmd_Active = softstart_cmd * Inverter_SysControl.Vout_cmd;
	} else {
		Inverter_SysControl.Vout_cmd_Active = Inverter_SysControl.Vout_cmd;
	}

	if (MG_Ctrl) {							   // Devide the control timing as 5KHz, PWM stay 2x5KHz.
		Inverter_V_CloseLoop();
		MG_Ctrl = 0;
	} else {
		MicroGrid_ctrl();
		MG_Ctrl = 1;
	}

	// Run Time: PWM update phase A
	// = = = = = = = = = = = = = = = = = = = = = = = =
	fTemp = sine_ref.sin_0_a + sine_ref.sin_3rd_harmonic;

	fTemp_gain = epwm1_info.EPwmGAIN + Inverter_SysControl.Loop_Compensator;	  // <<-- for regulation
	fTemp *= fTemp_gain;
	fTemp *= softstart_cmd;

	epwm1_info.EPwmCMP = fTemp * (OperationData.PWM_Period_Reg/2) + (OperationData.PWM_Period_Reg/2);

	EPwm1Regs.CMPA.half.CMPA = epwm1_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm1Regs.CMPB = epwm1_info.EPwmCMP; 			// Update PWM Counter-Compare B Register
	//------ Test: output zero ---------
	//EPwm1Regs.CMPA.half.CMPA = 0; 	
	//EPwm1Regs.CMPB = OperationData.PWM_Period_Reg/2; 			

	// Run Time: PWM update phase B
	// = = = = = = = = = = = = = = = = = = = = = = = =
	fTemp = sine_ref.sin_N120_b + sine_ref.sin_3rd_harmonic;

	fTemp_gain = epwm2_info.EPwmGAIN + Inverter_SysControl.Loop_Compensator;	  // <<-- for regulation
	fTemp *= fTemp_gain;
	fTemp *= softstart_cmd;

	epwm2_info.EPwmCMP = fTemp * (OperationData.PWM_Period_Reg/2) + (OperationData.PWM_Period_Reg/2);

	EPwm2Regs.CMPA.half.CMPA = epwm2_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm2Regs.CMPB = epwm2_info.EPwmCMP; 			// Update PWM Counter-Compare B Register
	//------ Test ---------
	//EPwm2Regs.CMPA.half.CMPA = 0; 	
	//EPwm2Regs.CMPB = 0; 			

	// Run Time: PWM update phase C
	// = = = = = = = = = = = = = = = = = = = = = = = =
	fTemp = sine_ref.sin_P120_c + sine_ref.sin_3rd_harmonic;

	fTemp_gain = epwm3_info.EPwmGAIN + Inverter_SysControl.Loop_Compensator;	  // <<-- for regulation
	fTemp *= fTemp_gain;
	fTemp *= softstart_cmd;

	epwm3_info.EPwmCMP = fTemp * (OperationData.PWM_Period_Reg/2) + (OperationData.PWM_Period_Reg/2);

	EPwm3Regs.CMPA.half.CMPA = epwm3_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm3Regs.CMPB = epwm3_info.EPwmCMP; 			// Update PWM Counter-Compare B Register
	//------ Test ---------
	//EPwm3Regs.CMPA.half.CMPA = 0; 	
	//EPwm3Regs.CMPB = 0; 			
}

/*
 *  ======== Inverter_SysControl ========
 *  Inverter control in Grid-Tie mode.
 */
void Inverter_Control_GT() {
	OperationData.EPwmTimerIntCount += 1.0;
	//OperationData.EPwmTimerIntCount += PLL_Data.PLL_Compensator;
	if ((unsigned int)(10.0 * OperationData.EPwmTimerIntCount) >= 200 * OperationData.Period_Cnt) {
		OperationData.EPwmTimerIntCount = 0.0;
	}
	OperationData.EPwmTimerIntCount += PLL_Data.PLL_Compensator;

//	GpioDataRegs.GPASET.bit.GPIO29 = 1; 		// LOAD DAC

	//===== sine reference generation =====
	sine_ref.ptr = 2 * PI * OperationData.EPwmTimerIntCount / (float)OperationData.Period_Cnt;
	sine_ref.ptr += OperationData.PhaseCommand;
	sine_ref.sin_0_a = sin(sine_ref.ptr);
	sine_ref.sin_3rd_harmonic = 0.01 * (float)iGrid_Tie.ThirdHarmonic * sin(3.0*sine_ref.ptr);
	sine_ref.cos_0_a = cos(sine_ref.ptr);
	sine_ref.cos_3rd_harmonic = 0.01 * (float)iGrid_Tie.ThirdHarmonic * cos(3.0*sine_ref.ptr);
	sine_ref.sin_2nd_harmonic = sin(2.0*sine_ref.ptr);
	sine_ref.cos_2nd_harmonic = cos(2.0*sine_ref.ptr);
	sine_ref.sin_5th_harmonic = sin(5.0*sine_ref.ptr);
	sine_ref.cos_5th_harmonic = cos(5.0*sine_ref.ptr);

	sine_ref.ptr += OperationData.DeltaPhase;
	sine_ref.sin_P120_c = sin(sine_ref.ptr);
	sine_ref.cos_P120_c = cos(sine_ref.ptr);

	sine_ref.ptr += OperationData.DeltaPhase;
	sine_ref.sin_N120_b = sin(sine_ref.ptr);
	sine_ref.cos_N120_b = cos(sine_ref.ptr);
	//======================================

	//	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1; 		// LOAD DAC
	//====== Grid-tie PWM soft start ======
	if (GT_softstart_inc <= iGrid_Tie.SoftStart_cnt) {
		GT_softstart_inc++;
		softstart_cmd = 1.0;		  // none soft-start
		if (GT_softstart_inc <= 1650) {
			SV_softstart -= 1.0/1650.0;		// soft satrt for SV
		}
		//softstart_cmd += 1.0/330.0;		  // soft-start
		//softstart_cmd += 1.0/(float)iGrid_Tie.SoftStart_cnt;		  // soft-start
		Inverter_SysControl.GT_ID_cmd_Active = softstart_cmd * Inverter_SysControl.GT_ID_cmd;
		Inverter_SysControl.GT_IQ_cmd_Active = softstart_cmd * Inverter_SysControl.GT_IQ_cmd;
	} else {
		if (iGrid_Tie.SoftStart_cnt != 500) {
			EPWM1_Off();		// For test !!!!!
		}
		Inverter_SysControl.GT_ID_cmd_Active = Inverter_SysControl.GT_ID_cmd;
		Inverter_SysControl.GT_IQ_cmd_Active = Inverter_SysControl.GT_IQ_cmd;
	}
//	GT_CloseLoop3();
	GT_CtrlLoop_Cnt ++ ;
	if (GT_CtrlLoop_Cnt >= GT_CtrlLoop_TIME) { // =2: half PWM frequency to control
		Inverter_I_CloseLoop_GT();
		GT_CtrlLoop_Cnt = 0;
		GT_Anti_Islanding();
	} else {
		GT_Harmonic_Ctrl();
	}
//	Inverter_I_CloseLoop_GT();
//	SV_PWM_GT();
	Sine_PWM_GT();

}

/*
 *  ======== Sine_PWM_GT ========
 *  Inverter sine PWM.
 */
void Sine_PWM_GT() {
float	fTemp;
float	sin_a, sin_b, sin_c;
float	cos_a, cos_b, cos_c;
float   FeedForward, FFWa, FFWb, FFWc;
float	Third_Harmonic;

	OperationData.PWM_Period_Reg_Half = OperationData.PWM_Period_Reg >> 1;
	//-----------------------------------------------------------------------
	//  DQ to a, b, c
	//-----------------------------------------------------------------------
	sin_a = sine_ref.sin_0_a;
	cos_a = sine_ref.cos_0_a;
	sin_b = sine_ref.sin_N120_b;
	cos_b = sine_ref.cos_N120_b;
	sin_c = sine_ref.sin_P120_c;
	cos_c = sine_ref.cos_P120_c;
	//------ Open Loop Test ------
	#ifdef GT_OPEN_LOOP_TEST
	Inverter_SysControl.GT_ID_out = Inverter_SysControl.GT_ID_cmd / 100.0;
	Inverter_SysControl.GT_IQ_out = Inverter_SysControl.GT_IQ_cmd / 100.0;
	#endif
	iGrid_Tie.Ta = Inverter_SysControl.GT_ID_out * sin_a + Inverter_SysControl.GT_IQ_out * cos_a;
	iGrid_Tie.Tb = Inverter_SysControl.GT_ID_out * sin_b + Inverter_SysControl.GT_IQ_out * cos_b;
	iGrid_Tie.Tc = Inverter_SysControl.GT_ID_out * sin_c + Inverter_SysControl.GT_IQ_out * cos_c;
	//iGrid_Tie.Tc = - iGrid_Tie.Ta - iGrid_Tie.Tb;

	//DQ_TO_ABC_float(iGrid_Tie.Ta, iGrid_Tie.Tb, iGrid_Tie.Tc, sin_a, cos_a,  
	//			   Inverter_SysControl.GT_ID_out, Inverter_SysControl.GT_IQ_out) //Note: phase shift?

	//--- add 5th harmonic control to Ta, Tb, Tc ---
	iGrid_Tie.Ta +=	GT_5th_harm_ctrl.Ia_out;
	iGrid_Tie.Tb +=	GT_5th_harm_ctrl.Ib_out;
	iGrid_Tie.Tc +=	GT_5th_harm_ctrl.Ic_out;

	//--- add 3rd harmonic to Ta, Tb, Tc ---
	#ifdef GT_OPEN_LOOP_TEST
	iGrid_Tie.Ta +=	Inverter_SysControl.GT_ID_cmd / 100.0 * sine_ref.sin_3rd_harmonic;
	iGrid_Tie.Tb +=	Inverter_SysControl.GT_ID_cmd / 100.0 * sine_ref.sin_3rd_harmonic;
	iGrid_Tie.Tc +=	Inverter_SysControl.GT_ID_cmd / 100.0 * sine_ref.sin_3rd_harmonic;
	#else
	Third_Harmonic = Inverter_SysControl.GT_ID_out * sine_ref.sin_3rd_harmonic 
				   + Inverter_SysControl.GT_IQ_out * sine_ref.cos_3rd_harmonic; 
	iGrid_Tie.Ta +=	Inverter_SysControl.GT_ID_out * Third_Harmonic;
	iGrid_Tie.Tb +=	Inverter_SysControl.GT_ID_out * Third_Harmonic;
	iGrid_Tie.Tc +=	Inverter_SysControl.GT_ID_out * Third_Harmonic;
	#endif

	//-----------------------------------------------------------------------
	//  Feed froward terms
	//-----------------------------------------------------------------------
	FeedForward = 0.01 * (float)iGrid_Tie.Feed_Forward;
	FFWa = FeedForward * sin_a;
	FFWb = FeedForward * sin_b;
	FFWc = -1.0 * (FFWa + FFWb);
	//--- add 3rd harmonic to FFWa, FFWb, FFWc ---
	FFWa +=	FeedForward * sine_ref.sin_3rd_harmonic;
	FFWb +=	FeedForward * sine_ref.sin_3rd_harmonic;
	FFWc +=	FeedForward * sine_ref.sin_3rd_harmonic;

	// prevent PWM counts over flow:
	iGrid_Tie.Ta += FFWa;
	iGrid_Tie.Tb += FFWb;
	iGrid_Tie.Tc += FFWc;

	iTemp_var = (int)(iGrid_Tie.Ta * 1000.0);
	if (iTemp_var >= 1000) {
		iGrid_Tie.Ta = 1.0;
	}
	if (iTemp_var <= -1000){
		iGrid_Tie.Ta = -1.0;
	}

	iTemp_var = (int)(iGrid_Tie.Tb * 1000.0);
	if (iTemp_var >= 1000) {
		iGrid_Tie.Tb = 1.0;
	}
	if (iTemp_var <= -1000){
		iGrid_Tie.Tb = -1.0;
	}

	iTemp_var = (int)(iGrid_Tie.Tc * 1000.0);
	if (iTemp_var >= 1000) {
		iGrid_Tie.Tc = 1.0;
	}
	if (iTemp_var <= -1000){
		iGrid_Tie.Tc = -1.0;
	}
	//-----------------------------------------------------------------------
	//  Update PWM registers
	//-----------------------------------------------------------------------
	fTemp = iGrid_Tie.Ta * (float)OperationData.PWM_Period_Reg_Half;
	epwm1_info.EPwmCMP = (int)fTemp + OperationData.PWM_Period_Reg_Half;
	EPwm1Regs.CMPA.half.CMPA = epwm1_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm1Regs.CMPB = epwm1_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

	fTemp = iGrid_Tie.Tb * (float)OperationData.PWM_Period_Reg_Half;
	epwm2_info.EPwmCMP = (int)fTemp + OperationData.PWM_Period_Reg_Half;
	EPwm2Regs.CMPA.half.CMPA = epwm2_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm2Regs.CMPB = epwm2_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

	fTemp = iGrid_Tie.Tc * (float)OperationData.PWM_Period_Reg_Half;
	epwm3_info.EPwmCMP = (int)fTemp + OperationData.PWM_Period_Reg_Half;
	EPwm3Regs.CMPA.half.CMPA = epwm3_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm3Regs.CMPB = epwm3_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

}

/*
 *  ======== SV_PWM_GT ========
 *  Inverter space vetor PWM.
 */
void SV_PWM_GT() {
float   D_FeedForward;
float   Q_FeedForward;
int		Ds_int, Qs_int;

	OperationData.PWM_Period_Reg_Half = OperationData.PWM_Period_Reg >> 1;
	//D_FeedForward = 0.0;
	//D_FeedForward = 0.01 * (float)iGrid_Tie.Feed_Forward * SV_softstart;
	D_FeedForward = 0.01 * (float)iGrid_Tie.Feed_Forward;
	Q_FeedForward = 0.05;

	//----------------------------------------------
	//	 impletement the inverse park transformation
	//----------------------------------------------
    //****  Open Loop test ****
	#ifdef GT_OPEN_LOOP_TEST
    iPark.Ds = Inverter_SysControl.GT_ID_cmd / 100.0;
    iPark.Qs = Inverter_SysControl.GT_IQ_cmd / 100.0;
	#else 	
	//**** Close loop ****
    iPark.Ds = 0.5 * Inverter_SysControl.GT_ID_out + D_FeedForward;
    iPark.Qs = 0.5 * Inverter_SysControl.GT_IQ_out + Q_FeedForward;
	#endif

	Ds_int = (int)(1000.0 * iPark.Ds);
	Qs_int = (int)(1000.0 * iPark.Qs);
	if (Ds_int >= 1000) iPark.Ds = 1.0;
	if (Ds_int <= -1000) iPark.Ds = -1.0;
	if (Qs_int >= 1000) iPark.Qs = 1.0;
	if (Qs_int <= -1000) iPark.Qs = -1.0;

	iPark.Sine   = sine_ref.sin_0_a;
    iPark.Cosine = sine_ref.cos_0_a;

	// Modified Park transformation to line up sine ref
	iPark.Alpha  = iPark.Qs * iPark.Cosine + iPark.Ds * iPark.Sine;
	iPark.Beta = iPark.Ds * iPark.Cosine - iPark.Qs * iPark.Sine;

	// Original Park transformation
	//iPark.Alpha = iPark.Ds * iPark.Cosine - iPark.Qs * iPark.Sine;
	//iPark.Beta  = iPark.Qs * iPark.Cosine + iPark.Ds * iPark.Sine;

	// ----------------------------------------------
	//	 space vector PWM sector
	// ----------------------------------------------
	svgen1.tmp1 = iPark.Beta;															
	svgen1.tmp2 = 0.5 * iPark.Beta + 0.8660254 * iPark.Alpha;
    svgen1.tmp3 = svgen1.tmp2 - svgen1.tmp1;		   // = -0.5 * Beta + 0.866 * Alpha
    
    svgen1.i_tmp1 = (int)(10000.0 * svgen1.tmp1);
    svgen1.i_tmp2 = (int)(10000.0 * svgen1.tmp2);													
	svgen1.i_tmp3 = (int)(10000.0 * svgen1.tmp3);
																				
	svgen1.VecSector = 3;														   
	svgen1.VecSector = (svgen1.i_tmp2 > 0)?( svgen1.VecSector - 1 ):svgen1.VecSector;  
	svgen1.VecSector = (svgen1.i_tmp3 > 0)?( svgen1.VecSector - 1 ):svgen1.VecSector;  
	svgen1.VecSector = (svgen1.i_tmp1 < 0)?( 7 - svgen1.VecSector ) :svgen1.VecSector;  
	
	// Modified switching table	to line up reference																	   
	if (svgen1.VecSector == 1 || svgen1.VecSector == 4){                        
    	svgen1.Ta =  svgen1.tmp2; 										   
    	svgen1.Tc =  svgen1.tmp1-svgen1.tmp3; 							   
    	svgen1.Tb = -svgen1.tmp2;											   
    } else if (svgen1.VecSector == 2 || svgen1.VecSector == 5){                        
    	svgen1.Ta =  svgen1.tmp3+svgen1.tmp2; 							   
    	svgen1.Tc =  svgen1.tmp1; 										   
    	svgen1.Tb = -svgen1.tmp1;							   
    } else {                                                       
    	svgen1.Ta =  svgen1.tmp3; 						   
    	svgen1.Tc = -svgen1.tmp3; 						   
    	svgen1.Tb = -(svgen1.tmp1+svgen1.tmp2);			   
    }  

	/*// Original switching table																		   
	if (svgen1.VecSector == 1 || svgen1.VecSector == 4){                        
    	svgen1.Ta =  svgen1.tmp2; 					  // Ta = 0.5 * Beta + 0.866 * Alpha					   
    	svgen1.Tb =  svgen1.tmp1-svgen1.tmp3; 		  // Tb = 1.5 * Beta - 0.866 * Alpha					   
    	svgen1.Tc = -svgen1.tmp2;					  // Tc = -0.5 * Beta - 0.866 * Alpha						   
    } else if (svgen1.VecSector == 2 || svgen1.VecSector == 5){                        
    	svgen1.Ta =  svgen1.tmp3+svgen1.tmp2; 		  // Ta = 1.732 * Alpha					   
    	svgen1.Tb =  svgen1.tmp1; 					  // Tb = Beta					   
    	svgen1.Tc = -svgen1.tmp1;					  // Tc = -Beta		   
    } else {                                                       
    	svgen1.Ta =  svgen1.tmp3; 					  // Ta = - 0.5 * Beta + 0.866 * Alpha	   
    	svgen1.Tb = -svgen1.tmp3; 					  // Tb = 0.5 * Beta - 0.866 * Alpha	   
    	svgen1.Tc = -(svgen1.tmp1+svgen1.tmp2);		  // Tc = -1.5 * Beta - 0.866 * Alpha	   
    } */
	
	if ((int)(svgen1.Ta * 1000.0) >= 1000)
		svgen1.Ta = 1.0;
	if ((int)(svgen1.Ta * 1000.0) <= -1000)
		svgen1.Ta = -1.0;

	if ((int)(svgen1.Tb * 1000.0) >= 1000)
		svgen1.Tb = 1.0;
	if ((int)(svgen1.Tb * 1000.0) <= -1000)
		svgen1.Tb = -1.0;

	if ((int)(svgen1.Tc * 1000.0) >= 1000)
		svgen1.Tc = 1.0;
	if ((int)(svgen1.Tc * 1000.0) <= -1000)
		svgen1.Tc = -1.0; 
	// ----------------------------------------------
	//	 update PWM registers
	// ----------------------------------------------
	epwm1_info.EPwmCMP = svgen1.Ta * OperationData.PWM_Period_Reg_Half + OperationData.PWM_Period_Reg_Half;
	EPwm1Regs.CMPA.half.CMPA = epwm1_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm1Regs.CMPB = epwm1_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

	epwm2_info.EPwmCMP = svgen1.Tb * OperationData.PWM_Period_Reg_Half + OperationData.PWM_Period_Reg_Half;
	EPwm2Regs.CMPA.half.CMPA = epwm2_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm2Regs.CMPB = epwm2_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

	epwm3_info.EPwmCMP = svgen1.Tc * OperationData.PWM_Period_Reg_Half + OperationData.PWM_Period_Reg_Half;
	EPwm3Regs.CMPA.half.CMPA = epwm3_info.EPwmCMP; 	// Update PWM Counter-Compare A Register
	EPwm3Regs.CMPB = epwm3_info.EPwmCMP; 			// Update PWM Counter-Compare B Register

}

/*
 *  ======== Inverter_V_CloseLoop ========
 *  Inverter voltage source control loop.	
 */
void Inverter_V_CloseLoop() {
int	PI_output;
	Inverter_SysControl.Vout_cmd = OperationData.VoltageCommand;
	//Inverter_SysControl.Vout_error = Inverter_SysControl.Vout_cmd_Active - AC_Line_Measurement.V_Line;
	Inverter_SysControl.Vout_error = Inverter_SysControl.Vout_cmd_Active - VAC_out_Measurement.V_Line;
	Inverter_SysControl.Vout_error *= 0.001;					 // changed from 0.01
	
	Inverter_SysControl.Loop_Compensator = Inverter_SysControl.V_Control_Kp * Inverter_SysControl.Vout_error;
	Inverter_SysControl.V_error_int = Inverter_SysControl.V_Control_Ki * Inverter_SysControl.Vout_error;
	Inverter_SysControl.V_error_int += Inverter_SysControl.V_error_int_old;

	// Integral term limit with +/- 100V
	PI_output = (int)(Inverter_SysControl.V_error_int * 1000.0);
    if (PI_output >= 700) Inverter_SysControl.V_error_int = 0.7;
    if (PI_output <= -700) Inverter_SysControl.V_error_int = -0.7;
	Inverter_SysControl.V_error_int_old = Inverter_SysControl.V_error_int;

	Inverter_SysControl.Loop_Compensator += Inverter_SysControl.V_error_int;

}

/*
 *  ======== Inverter_I_CloseLoop: Voltage source mode ========
 *  Inverter voltage source control loop.	
 */
void Inverter_I_CloseLoop_MG() {
int	PI_output;
	Inverter_SysControl.Iout_cmd = OperationData.ID_out_cmd;
	// I out error
	Inverter_SysControl.Iout_error = Inverter_SysControl.Iout_cmd - VAC_out_Measurement.I_Line;	// Use switching current sensor
	Inverter_SysControl.Iout_error *= 0.001;
	
	Inverter_SysControl.Loop_Compensator = Inverter_SysControl.I_Control_Kp * Inverter_SysControl.Iout_error;
	Inverter_SysControl.I_error_int = Inverter_SysControl.I_Control_Ki * Inverter_SysControl.Iout_error;
	Inverter_SysControl.I_error_int += Inverter_SysControl.I_error_int_old;

	// Integral term limit with +/- 300A   Need further test!!!
	PI_output = (int)(Inverter_SysControl.I_error_int * 1000.0);
    if (PI_output >= 700) Inverter_SysControl.I_error_int = 0.7;
    if (PI_output <= -700) Inverter_SysControl.I_error_int = -0.7;
	Inverter_SysControl.I_error_int_old = Inverter_SysControl.I_error_int;

	Inverter_SysControl.Loop_Compensator += Inverter_SysControl.I_error_int;

}

/*
 *  ======== Inverter_I_CloseLoop: Grid-tie mode ========
 *  Inverter voltage source control loop.	
 */
void Inverter_I_CloseLoop_GT() {
int	PI_output;

	//=== output current control: D component ===
	Inverter_SysControl.GT_ID_cmd = OperationData.ID_out_cmd - speed_control.speed_Loop_Compensator;
	//---- ID out error ----
	//Inverter_SysControl.GT_IDout_error = Inverter_SysControl.GT_ID_cmd - VAC_out_Measurement.I_Line_D;		// Use switching current sensor
	Inverter_SysControl.GT_IDout_error = Inverter_SysControl.GT_ID_cmd - AC_Line_Measurement.I_Line_D;	// Use output current sensor
	Inverter_SysControl.GT_IDout_error *= 0.001;
	
	//---- Proportional ----
	Inverter_SysControl.GT_ID_error_pro = Inverter_SysControl.GT_I_Control_Kp * Inverter_SysControl.GT_IDout_error;
	//---- Integral ----
	Inverter_SysControl.GT_ID_error_int = Inverter_SysControl.GT_ID_error_int + 
										  Inverter_SysControl.GT_I_Control_Ki * Inverter_SysControl.GT_ID_error_pro +
										  Inverter_SysControl.GT_I_Control_Kc * Inverter_SysControl.GT_IDout_SatErr;
	//---- Derivative ----
	Inverter_SysControl.GT_ID_error_der = Inverter_SysControl.GT_I_Control_Kd * 
										  (Inverter_SysControl.GT_ID_error_pro - Inverter_SysControl.GT_ID_error_pro1);
	//---- Pre-saturated output ----
	Inverter_SysControl.GT_IDout_PreSat = Inverter_SysControl.GT_ID_error_pro + Inverter_SysControl.GT_ID_error_int +
										  Inverter_SysControl.GT_ID_error_der;
	//---- Saturate output ----
	Inverter_SysControl.GT_ID_out = Inverter_SysControl.GT_IDout_PreSat;
	PI_output = (int)(Inverter_SysControl.GT_IDout_PreSat * 1000.0);
    if (PI_output >= 1000) Inverter_SysControl.GT_ID_out = 1.0;		  		//for sv_PWM_GT
    if (PI_output <= -1000) Inverter_SysControl.GT_ID_out = -1.0;		  	//for sv_PWM_GT
	//---- Saturated difference ----
	Inverter_SysControl.GT_IDout_SatErr = Inverter_SysControl.GT_ID_out - Inverter_SysControl.GT_IDout_PreSat;
	Inverter_SysControl.GT_ID_error_pro1 = Inverter_SysControl.GT_ID_error_pro;

	//=== output current control: Q component ===
	Inverter_SysControl.GT_IQ_cmd = OperationData.IQ_out_cmd + iGrid_Tie.Q_inject;
	//---- IQ out error ----
	//Inverter_SysControl.GT_IQout_error = Inverter_SysControl.GT_IQ_cmd - VAC_out_Measurement.I_Line_Q;		// Use switching current sensor
	Inverter_SysControl.GT_IQout_error = Inverter_SysControl.GT_IQ_cmd - AC_Line_Measurement.I_Line_Q;	// Use output current sensor
	Inverter_SysControl.GT_IQout_error *= 0.001;
	
	//---- Proportional ----
	Inverter_SysControl.GT_IQ_error_pro = Inverter_SysControl.GT_I_Control_Kp * Inverter_SysControl.GT_IQout_error;
	//---- Integral ----
	Inverter_SysControl.GT_IQ_error_int = Inverter_SysControl.GT_IQ_error_int + 
										  Inverter_SysControl.GT_I_Control_Ki * Inverter_SysControl.GT_IQ_error_pro +
										  Inverter_SysControl.GT_I_Control_Kc * Inverter_SysControl.GT_IQout_SatErr;
	//---- Derivative ----
	Inverter_SysControl.GT_IQ_error_der = Inverter_SysControl.GT_I_Control_Kd * 
										  (Inverter_SysControl.GT_IQ_error_pro - Inverter_SysControl.GT_IQ_error_pro1);
	//---- Pre-saturated output ----
	Inverter_SysControl.GT_IQout_PreSat = Inverter_SysControl.GT_IQ_error_pro + Inverter_SysControl.GT_IQ_error_int +
										  Inverter_SysControl.GT_IQ_error_der;
	//---- Saturate output ----
	Inverter_SysControl.GT_IQ_out = Inverter_SysControl.GT_IQout_PreSat;
	PI_output = (int)(Inverter_SysControl.GT_IQout_PreSat * 1000.0);
    if (PI_output >= 1000) Inverter_SysControl.GT_IQ_out = 1.0;		  		//for sv_PWM_GT
    if (PI_output <= -1000) Inverter_SysControl.GT_IQ_out = -1.0;		  	//for sv_PWM_GT
	//---- Saturated difference ----
	Inverter_SysControl.GT_IQout_SatErr = Inverter_SysControl.GT_IQ_out - Inverter_SysControl.GT_IQout_PreSat;
	Inverter_SysControl.GT_IQ_error_pro1 = Inverter_SysControl.GT_IQ_error_pro;

}

/*
 *  ======== GT_Harmonic_Ctrl ========
 */
void GT_Harmonic_Ctrl() {
float	alpha, beta;
	//---   Harmonic control     ---
	//if (iGrid_Tie.HarmonicControl) {
	if (1) {
		// Current command reference
		//DQ_TO_ABC_float(GT_5th_harm_ctrl.Ia_cmd_ref, GT_5th_harm_ctrl.Ib_cmd_ref, GT_5th_harm_ctrl.Ic_cmd_ref, 
		//		sine_ref.sin_0_a, sine_ref.cos_0_a, Inverter_SysControl.GT_ID_cmd, Inverter_SysControl.GT_IQ_cmd)
		GT_5th_harm_ctrl.Ia_cmd_ref = Inverter_SysControl.GT_ID_cmd * sine_ref.sin_0_a + Inverter_SysControl.GT_IQ_cmd * sine_ref.cos_0_a;
		GT_5th_harm_ctrl.Ia_cmd_ref *= DQ_AC_CURRENT_CONVERT_FACTOR; 
		GT_5th_harm_ctrl.Ib_cmd_ref = Inverter_SysControl.GT_ID_cmd * sine_ref.sin_N120_b + Inverter_SysControl.GT_IQ_cmd * sine_ref.cos_N120_b;
		GT_5th_harm_ctrl.Ib_cmd_ref *= DQ_AC_CURRENT_CONVERT_FACTOR;
		GT_5th_harm_ctrl.Ic_cmd_ref	= -1.0 * (GT_5th_harm_ctrl.Ia_cmd_ref + GT_5th_harm_ctrl.Ib_cmd_ref);

		// Output current measurement error
		GT_5th_harm_ctrl.Ia_in =  AC_Line_Measurement.I_A - GT_5th_harm_ctrl.Ia_cmd_ref;
		GT_5th_harm_ctrl.Ib_in =  AC_Line_Measurement.I_B - GT_5th_harm_ctrl.Ib_cmd_ref;
		GT_5th_harm_ctrl.Ic_in =  AC_Line_Measurement.I_C - GT_5th_harm_ctrl.Ic_cmd_ref;

		// ******************************
		// 5th harmonic DQ
		ABC_TO_DQ_float(GT_5th_harm_ctrl.Ia_in, GT_5th_harm_ctrl.Ic_in, GT_5th_harm_ctrl.Ib_in, 
				sine_ref.sin_5th_harmonic, sine_ref.cos_5th_harmonic, alpha, beta, GT_5th_harm_ctrl.I_D, GT_5th_harm_ctrl.I_Q)
		// Low pass (average) filter for I_D, I_Q
		GT_5th_harm_ctrl.I_D_Fil = GT_5th_harm_ctrl.Filter_K1 * GT_5th_harm_ctrl.I_D + GT_5th_harm_ctrl.Filter_K2 * GT_5th_harm_ctrl.I_D_Fil;
		GT_5th_harm_ctrl.I_Q_Fil = GT_5th_harm_ctrl.Filter_K1 * GT_5th_harm_ctrl.I_Q + GT_5th_harm_ctrl.Filter_K2 * GT_5th_harm_ctrl.I_Q_Fil;

		// ******************************
		// 2nd harmonic DQ
		ABC_TO_DQ_float(GT_5th_harm_ctrl.Ia_in, GT_5th_harm_ctrl.Ic_in, GT_5th_harm_ctrl.Ib_in, 
				sine_ref.sin_2nd_harmonic, sine_ref.cos_2nd_harmonic, alpha, beta, GT_2nd_harm_ctrl.I_D, GT_2nd_harm_ctrl.I_Q)
		// Low pass (average) filter for I_D, I_Q
		GT_2nd_harm_ctrl.I_D_Fil = GT_2nd_harm_ctrl.Filter_K1 * GT_2nd_harm_ctrl.I_D + GT_2nd_harm_ctrl.Filter_K2 * GT_2nd_harm_ctrl.I_D_Fil;
		GT_2nd_harm_ctrl.I_Q_Fil = GT_2nd_harm_ctrl.Filter_K1 * GT_2nd_harm_ctrl.I_Q + GT_2nd_harm_ctrl.Filter_K2 * GT_2nd_harm_ctrl.I_Q_Fil;

		// ******************************
		// 5th harmonic regulation ....	Kp = 1.0, Ki = 0.001, Max = 0.1, Min = -0.1
		PI_REG_PF(0.0, 0.001*GT_5th_harm_ctrl.I_D_Fil, GT_5th_harm_ctrl.I_D_err, GT_5th_harm_ctrl.I_D_integral, 
				GT_5th_harm_ctrl.I_D_com, GT_5th_harm_ctrl.Kp, GT_5th_harm_ctrl.Ki, 0.1, -0.1)
		PI_REG_PF(0.0, 0.001*GT_5th_harm_ctrl.I_Q_Fil, GT_5th_harm_ctrl.I_Q_err, GT_5th_harm_ctrl.I_Q_integral, 
				GT_5th_harm_ctrl.I_Q_com, GT_5th_harm_ctrl.Kp, GT_5th_harm_ctrl.Ki, 0.1, -0.1)

		// 5th harmonic regulation back to ABC
		DQ_TO_ABC_float(GT_5th_harm_ctrl.Ia_out, GT_5th_harm_ctrl.Ic_out, GT_5th_harm_ctrl.Ib_out, 
				sine_ref.sin_5th_harmonic, sine_ref.cos_5th_harmonic, GT_5th_harm_ctrl.I_D_com, GT_5th_harm_ctrl.I_Q_com)

		// ******************************
		// 2nd harmonic regulation ....	Kp = 0.01, Ki = 0.001, Max = 0.1, Min = -0.1
		PI_REG_PF(0.0, 0.001*GT_2nd_harm_ctrl.I_D_Fil, GT_2nd_harm_ctrl.I_D_err, GT_2nd_harm_ctrl.I_D_integral, 
				GT_2nd_harm_ctrl.I_D_com, GT_2nd_harm_ctrl.Kp, GT_2nd_harm_ctrl.Ki, 0.1, -0.1)
		PI_REG_PF(0.0, 0.001*GT_2nd_harm_ctrl.I_Q_Fil, GT_2nd_harm_ctrl.I_Q_err, GT_2nd_harm_ctrl.I_Q_integral, 
				GT_2nd_harm_ctrl.I_Q_com, GT_2nd_harm_ctrl.Kp, GT_2nd_harm_ctrl.Ki, 0.1, -0.1)

		// 2nd harmonic regulation back to ABC
		DQ_TO_ABC_float(GT_2nd_harm_ctrl.Ia_out, GT_2nd_harm_ctrl.Ic_out, GT_2nd_harm_ctrl.Ib_out, 
				sine_ref.sin_2nd_harmonic, sine_ref.cos_2nd_harmonic, GT_2nd_harm_ctrl.I_D_com, GT_2nd_harm_ctrl.I_Q_com)

		GT_5th_harm_ctrl.Ia_out += GT_2nd_harm_ctrl.Ia_out;
		GT_5th_harm_ctrl.Ib_out += GT_2nd_harm_ctrl.Ib_out;
		GT_5th_harm_ctrl.Ic_out += GT_2nd_harm_ctrl.Ic_out;
	} 

	if (!iGrid_Tie.HarmonicControl) {
		GT_5th_harm_ctrl.Ia_out = 0.0;
		GT_5th_harm_ctrl.Ib_out = 0.0;
		GT_5th_harm_ctrl.Ic_out = 0.0;
	
	}

}


/*
 *  ======== GT_Anti_Islanding ========
 *  Anti-islanding
 */
void GT_Anti_Islanding() {
float	fErr;
	//**** Anti-islanding reference ****
	if (Anti_Islanding.ref_update == TRUE) {
		if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){
			Anti_Islanding.Vref = 0.00001 * AC_Line_Measurement.V_Line + 0.99999 * Anti_Islanding.Vref;
		} else {
			Anti_Islanding.Vref = 0.1 * AC_Line_Measurement.V_Line + 0.9 * Anti_Islanding.Vref;
		}
	}
	//Anti_Islanding.Vref = 480.0;
	Anti_Islanding.Va = Anti_Islanding.Vref * sine_ref.sin_0_a;
	Anti_Islanding.Vb = Anti_Islanding.Vref * sine_ref.sin_N120_b;
	Anti_Islanding.Vc = Anti_Islanding.Vref * sine_ref.sin_P120_c;

	//**** AC line error ****
	fErr = abs(Anti_Islanding.Va - AC_Line_Measurement.V_A) 
		 + abs(Anti_Islanding.Vb - AC_Line_Measurement.V_B) 
		 + abs(Anti_Islanding.Vc - AC_Line_Measurement.V_C);

	Anti_Islanding.V_error = 0.005 * fErr + 0.995 *	Anti_Islanding.V_error;
	if ((unsigned int)Anti_Islanding.V_error > Anti_Islanding.sensitivity){
		Anti_Islanding.ref_update = FALSE;
		Anti_Islanding.err_cnt ++ ;
		if (Anti_Islanding.err_cnt >= Anti_Islanding.trip_time) {
			Anti_Islanding.ref_update = TRUE;
			Anti_Islanding.err_cnt = Anti_Islanding.trip_time;
			Anti_Islanding.Grid_Lost = TRUE;
			if (system.state == GRID_TIE_MODE || system.state == GRID_TIE_STANDBY){ 
				system.command = SysCtrlCmd_MainOutputOff;
				iGrid_Tie.Trip |= dANTI_ISLANDING_TRIP;
			}
		}		
	} else {
		Anti_Islanding.ref_update = TRUE;
		Anti_Islanding.err_cnt = 0;
		Anti_Islanding.Grid_Lost = FALSE;
	}

}

/*
 *  ======== Boost control ========
 *  PWM4 output  update.
 */
void BoostControl() {
int16	PI_VL_out, PI_IL_out;
int32	PI_output;
#ifdef BOOST_WITH_I_LOOP
	//**** V loop control ****
	Boost_Control.DCBUS_V_error = Boost_Control.DCBUS_cmd_Active - DC_BUS_1;
	Boost_Control.DCBUS_I_cmd = Boost_Control.V_Control_Kp * Boost_Control.DCBUS_V_error;
	Boost_Control.DCBUS_V_error_int = Boost_Control.V_Control_Ki * Boost_Control.DCBUS_V_error;
	Boost_Control.DCBUS_V_error_int += Boost_Control.DCBUS_V_error_int_old;
	// Integral term limit for V loop
	PI_VL_out = (int)(Boost_Control.DCBUS_V_error_int * 10.0);
	if (PI_VL_out >=  Boost_Control.DCBUS_I_Limit_x10) Boost_Control.DCBUS_V_error_int = 0.1 * Boost_Control.DCBUS_I_Limit_x10;
	if (PI_VL_out <= -1*Boost_Control.DCBUS_I_Limit_x10) Boost_Control.DCBUS_V_error_int = -0.1 * Boost_Control.DCBUS_I_Limit_x10;
	// Boost over current
	Boost_Currentx10 = (int)(Boost_Current * 10.0);
	if (Boost_Currentx10 >	Boost_Control.DCBUS_I_Limit_x10 + Boost_Control.DCBUS_OverCurrent_x10) {
		Boost_OI_Count++;
		if (Boost_OI_Count > InputStageFilter){
			DO_1_On();
			EPWM_Boost_Off();
			EPWM1_Off();
			system.SW_faults |= BOOST_OVER_CURRENT;
		}
	} else {
		Boost_OI_Count = 0;
	}

	Boost_Control.DCBUS_I_cmd += Boost_Control.DCBUS_V_error_int;
	Boost_Control.DCBUS_V_error_int_old = Boost_Control.DCBUS_V_error_int;

	// current limit for V loop			 // !!!!! back to limit integral term	!!!!
//	PI_VL_out = (int)(Boost_Control.DCBUS_I_cmd * 10.0);
//	if (PI_VL_out >=  Boost_Control.DCBUS_I_Limit_x10) Boost_Control.DCBUS_I_cmd = 0.1 * Boost_Control.DCBUS_I_Limit_x10;
//	if (PI_VL_out <= -1*Boost_Control.DCBUS_I_Limit_x10) Boost_Control.DCBUS_I_cmd = -0.1 * Boost_Control.DCBUS_I_Limit_x10;

	//**** I loop control ****
	Boost_Control.DCBUS_I_error = Boost_Control.DCBUS_I_cmd - Boost_Current;
	//Boost_Control.DCBUS_I_error = 0.1*(float)Boost_Control.DCBUS_I_Limit_x10 - Boost_Current;
	Boost_Control.DCBUS_I_Loopout = Boost_Control.I_Control_Kp * Boost_Control.DCBUS_I_error;
	Boost_Control.DCBUS_I_error_int = Boost_Control.I_Control_Ki * Boost_Control.DCBUS_I_error;
	Boost_Control.DCBUS_I_error_int += Boost_Control.DCBUS_I_error_int_old;

	// Integral term limit for I loop
	PI_output = (long)(100.0 * Boost_Control.DCBUS_I_error_int);
	if (PI_output >= BOOST_HALF_DUTY_X100) Boost_Control.DCBUS_I_error_int = BOOST_HALF_DUTY;
	if (PI_output <= -1 * BOOST_HALF_DUTY_X100) Boost_Control.DCBUS_I_error_int = -1 * BOOST_HALF_DUTY;

	Boost_Control.DCBUS_I_Loopout += Boost_Control.DCBUS_I_error_int;
	Boost_Control.DCBUS_I_error_int_old = Boost_Control.DCBUS_I_error_int;

    // normalize and update PWM compare register
	PI_IL_out = (int)Boost_Control.DCBUS_I_Loopout;
    if (PI_IL_out >= (BOOST_HALF_DUTY - BOOST_DUTY_LIMIT)) Boost_Control.DCBUS_I_Loopout = BOOST_HALF_DUTY - BOOST_DUTY_LIMIT; //!!!
    if (PI_IL_out <= -1 * BOOST_HALF_DUTY) Boost_Control.DCBUS_I_Loopout = -1 * BOOST_HALF_DUTY;

	Boost_Control.EPwmDuty = (int)(Boost_Control.DCBUS_I_Loopout) + BOOST_HALF_DUTY;
	EPwm4Regs.CMPA.half.CMPA = Boost_Control.EPwmDuty; 	// Update PWM Counter-Compare A Register
	EPwm4Regs.CMPB = Boost_Control.EPwmDuty; 			// Update PWM Counter-Compare B Register

#else
	Boost_Control.DCBUS_V_error = Boost_Control.DCBUS_cmd_Active - DC_BUS_1;
	Boost_Control.DCBUS_I_cmd = Boost_Control.V_Control_Kp * Boost_Control.DCBUS_V_error;
	Boost_Control.DCBUS_V_error_int = Boost_Control.V_Control_Ki * Boost_Control.DCBUS_V_error;
	Boost_Control.DCBUS_V_error_int += Boost_Control.DCBUS_V_error_int_old;
	// Integral term limit
    PI_output = (long)(Boost_Control.DCBUS_V_error_int * 100.0);

    if (PI_output >=  HALF_DUTY_X100) Boost_Control.DCBUS_V_error_int = HALF_DUTY;
    if (PI_output <= -1 * HALF_DUTY_X100) Boost_Control.DCBUS_V_error_int = -1 * HALF_DUTY;

	Boost_Control.DCBUS_I_cmd += Boost_Control.DCBUS_V_error_int;
    Boost_Control.DCBUS_V_error_int_old = Boost_Control.DCBUS_V_error_int;
    // normalize and update PWM compare register
    PI_output = (long)Boost_Control.DCBUS_I_cmd;

    if (PI_output >= HALF_DUTY) Boost_Control.DCBUS_I_cmd = HALF_DUTY;
    if (PI_output <= -1 * HALF_DUTY) Boost_Control.DCBUS_I_cmd = -1 * HALF_DUTY;

    Boost_Control.EPwmDuty = (int)(Boost_Control.DCBUS_I_cmd) + HALF_DUTY;
	EPwm4Regs.CMPA.half.CMPA = Boost_Control.EPwmDuty; 	// Update PWM Counter-Compare A Register
	EPwm4Regs.CMPB = Boost_Control.EPwmDuty; 			// Update PWM Counter-Compare B Register
#endif
}


/*
 *  ======== EPWM1_On ========
 *  Turn on ePWM.
 */
void EPWM1_On() {
    //===== Stand-alone control initialization =====
	Inverter_SysControl.V_error_int_old	= 0.0;
	Inverter_SysControl.I_error_int_old = 0.0;
	Inverter_SysControl.Loop_Compensator = 0.0;
	Inverter_SysControl.Vout_cmd = OperationData.VoltageCommand;

    //===== Grid-Tie control initialization =====
	Inverter_SysControl.GT_ID_error_int	= 0.0;
	Inverter_SysControl.GT_ID_error_int_old = iGrid_Tie.D_init;
	Inverter_SysControl.GT_IQ_error_int	= 0.0;
	Inverter_SysControl.GT_IQ_error_int_old = iGrid_Tie.Q_init;
	Inverter_SysControl.GT_IDout_error = 0.0;
	Inverter_SysControl.GT_IQout_error = 0.0;
	Inverter_SysControl.GT_ID_error_der = 0.0;
	Inverter_SysControl.GT_ID_error_pro = 0.0;
	Inverter_SysControl.GT_ID_error_pro1 = 0.0;
	Inverter_SysControl.GT_IDout_SatErr = 0.0;
	Inverter_SysControl.GT_IQ_error_der = 0.0;
	Inverter_SysControl.GT_IQ_error_pro = 0.0;
	Inverter_SysControl.GT_IQ_error_pro1 = 0.0;
	Inverter_SysControl.GT_IQout_SatErr = 0.0;
	SV_softstart = 1.0;

	GT_cl3_I_int1_PhaseA = 0.0;
	GT_cl3_I_int1_PhaseB = 0.0;
	GT_cl3_I_int1_PhaseC = 0.0;

    //===== Grid-Tie 5th harmonic control initialization =====
	GT_5th_harm_ctrl.I_D_integral = 0.0;
	GT_5th_harm_ctrl.I_Q_integral = 0.0;
	GT_5th_harm_ctrl.I_D_err = 0.0;
	GT_5th_harm_ctrl.I_Q_err = 0.0;
	GT_5th_harm_ctrl.I_D_Fil = 0.0;
	GT_5th_harm_ctrl.I_Q_Fil = 0.0;
    //===== Grid-Tie 2nd harmonic control initialization =====
	GT_2nd_harm_ctrl.I_D_integral = 0.0;
	GT_2nd_harm_ctrl.I_Q_integral = 0.0;
	GT_2nd_harm_ctrl.I_D_err = 0.0;
	GT_2nd_harm_ctrl.I_Q_err = 0.0;
	GT_2nd_harm_ctrl.I_D_Fil = 0.0;
	GT_2nd_harm_ctrl.I_Q_Fil = 0.0;

	GT_CtrlLoop_Cnt = 0;


	// soft start variables:
    softstart_cmd = 0.0;
    softstart_inc = 0;
    GT_softstart_inc = 0;
	EPwm1Regs.CMPA.half.CMPA = OperationData.PWM_Period_Reg/2; 		// adjust duty for output EPWM1A   
	EPwm1Regs.CMPB = OperationData.PWM_Period_Reg/2; 				// adjust duty for output EPWM1B   
	EPwm2Regs.CMPA.half.CMPA = OperationData.PWM_Period_Reg/2; 		// adjust duty for output EPWM1A
	EPwm2Regs.CMPB = OperationData.PWM_Period_Reg/2; 				// adjust duty for output EPWM1B
	EPwm3Regs.CMPA.half.CMPA = OperationData.PWM_Period_Reg/2; 		// adjust duty for output EPWM1A
	EPwm3Regs.CMPB = OperationData.PWM_Period_Reg/2; 				// adjust duty for output EPWM1B
	InitEPwm1TripZone();
}

/*
 *  ======== EPWM1_Off ========
 *  Turn off ePWM.
 */
void EPWM1_Off() {
	EALLOW;
	EPwm1Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EPwm2Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EPwm3Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EDIS;
}


/*
 *  ======== EPWM_Boost_On ========
 *  Turn on ePWM.
 */
void EPWM_Boost_On() {
    Boost_Control.DCBUS_V_error_int_old = -10000.0;
    Boost_Control.DCBUS_I_error_int_old = -10000.0;
	EPwm4Regs.CMPA.half.CMPA = 500; 					// Update PWM Counter-Compare A Register
	EPwm4Regs.CMPB = 500; 							// Update PWM Counter-Compare B Register  
    Boost_Control.DCBUS_cmd_Active = DC_BUS_1;
    //Boost_Control.DCBUS_cmd_Active = Boost_Control.DCBUS_cmd;
	InitEPwm2TripZone_Boost();
}

/*
 *  ======== EPWM_Boost_Off ========
 *  Turn off ePWM.
 */
void EPWM_Boost_Off() {
	EALLOW;
	EPwm4Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EDIS;
    Boost_Control.DCBUS_cmd_Active = 0.0;
}


/*
 *  ======== EPWM_Boost2_On ========
 *  Turn on ePWM.
 */
void EPWM_Boost2_On() {
	EPwm5Regs.CMPA.half.CMPA = 9000; 					// Update PWM Counter-Compare A Register
	EPwm5Regs.CMPB = 9000; 							// Update PWM Counter-Compare B Register  
	InitEPwm2TripZone_2ndBoost();
}

/*
 *  ======== EPWM_Boost2_Off ========
 *  Turn off ePWM.
 */
void EPWM_Boost2_Off() {
	EALLOW;
	EPwm5Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EDIS;
}


/*
 *  ======== EPWM_Brake_On ========
 *  Turn on ePWM.
 */
void EPWM_Brake_On() {
	EPwm6Regs.CMPA.half.CMPA = 50000; 					// Update PWM Counter-Compare A Register
	EPwm6Regs.CMPB = 50000; 							// Update PWM Counter-Compare B Register  
	InitEPwm2TripZone_Brake();
}

/*
 *  ======== EPWM_Brake_Off ========
 *  Turn off ePWM.
 */
void EPWM_Brake_Off() {
	EALLOW;
	EPwm6Regs.TZFRC.bit.OST = TRUE;	// force PWM trip
	EDIS;
}



/********* End of file ***********/
