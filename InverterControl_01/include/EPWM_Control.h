/**********************************************************************************
// File: EPWM_Control.h
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
// This is the header file for EPWM_Control.c.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/13/12	|  J Wen 	| Original
**********************************************************************************/

#ifndef EPWM_CONTROL_H_
#define EPWM_CONTROL_H_

typedef struct {
   volatile struct EPWM_REGS *EPwmRegHandle;
   Uint16 EPwmTimerIntCount;
   Uint16 EPwmCMP;
   float  EPwmGAIN;
   float  Phase;
}EPWM_INFO;

typedef struct {
   float EPwmTimerIntCount;
   Uint16 EPwmTZ_Trip;
   float  Frequency;
   float  DeltaPhase;
   float  PWM_Frequency;
   float  VoltageCommand;
   float  ID_out_cmd;
   float  IQ_out_cmd;
   Uint16  Period_Cnt;
   Uint16  PWM_Period_Reg;
   Uint16  PWM_Period_Reg_Half;
   float   PhaseCommand;
   float   GT_SpeedCommand;
   int	   int_GT_SpeedCommand;
   Uint16  GT_SpeedControl;
}OPERATION_DATA;

typedef struct {
   float  DCBUS_cmd;
   float  DCBUS_cmd_Active;
   float  DCBUS_V_error;
   float  DCBUS_V_error_int;
   float  DCBUS_V_error_int_old;
   float  DCBUS_I_cmd;
   int16  DCBUS_I_Limit_x10;
   int16  DCBUS_OverCurrent_x10;
   float  DCBUS_I_error;
   float  DCBUS_I_error_int;
   float  DCBUS_I_error_int_old;
   float  DCBUS_I_Loopout;
   float  V_Control_Ki;
   float  V_Control_Kp;
   float  I_Control_Ki;
   float  I_Control_Kp;
   int16  EPwmDuty;
   int16  Boost_Duty_Limit;
   int16  Half_Duty;
   long   Half_Duty_x100;
}BOOST_CONTROL;

typedef struct {
   float  Vout_cmd;
   float  Vout_cmd_Active;
   float  Vout_error;
   float  V_error_int;
   float  V_error_int_old;
   float  Iout_cmd;
   float  Iout_cmd_Active;
   float  Iout_error;
   float  I_error_int;
   float  I_error_int_old;
   int16  VLoop_I_Limit_x10;
   float  Loop_Compensator;
   float  V_Control_Ki;
   float  V_Control_Kp;
   float  I_Control_Ki;
   float  I_Control_Kp;
   int16  SystemOperationMode;
   Uint16 InverterOn;
   float  GT_ID_cmd;
   float  GT_ID_out;
   float  GT_ID_cmd_Active;
   float  GT_IQ_cmd;
   float  GT_IQ_out;
   float  GT_IQ_cmd_Active;
   float  GT_IDout_error;
   float  GT_IDout_SatErr;
   float  GT_IDout_PreSat;
   float  GT_IQout_error;
   float  GT_IQout_SatErr;
   float  GT_IQout_PreSat;
   float  GT_ID_error_pro;
   float  GT_ID_error_pro1;
   float  GT_ID_error_int;
   float  GT_ID_error_der;
   float  GT_IQ_error_pro;
   float  GT_IQ_error_pro1;
   float  GT_IQ_error_int;
   float  GT_IQ_error_der;
   float  GT_ID_error_int_old;
   float  GT_IQ_error_int_old;
   float  GT_I_Control_Ki;
   float  GT_I_Control_Kp;
   float  GT_I_Control_Kc;
   float  GT_I_Control_Kd;
}INVERTER_CONTROL;

typedef struct {
   Uint16  SoftStart_cnt;
   int	   Feed_Forward;
   Uint16  SV_sine;
   Uint16  ThirdHarmonic;
   Uint16  HarmonicControl;
   float   Ta;
   float   Tb;
   float   Tc;
   float   D_init;
   float   Q_init;
   float   Q_inject;
   float   Q_inject_Setting;
   Uint16  Q_inject_switch;
   Uint16  Trip;
}GRID_TIE;

typedef struct {
	float Ds;
	float Qs;
	float Sine;
	float Cosine;
	float Alpha;
	float Beta;
}I_PARK;

typedef struct {
	float 	tmp1;
	float 	tmp2;
	float 	tmp3;
	int 	i_tmp1;
	int 	i_tmp2;
	int 	i_tmp3;
	Uint16 	VecSector;
	float 	Ta;
	float 	Tb;
	float 	Tc;
}SV_GEN;

typedef struct {
	float 	Vref;
	float 	Va;
	float 	Vb;
	float 	Vc;
	float 	Vab;
	float 	Vbc;
	float 	Vca;
	float 	V_error;
	Uint16  err_cnt;
	Uint16  ref_update;
	Uint16  sensitivity;
	Uint16  trip_time;
	int		Grid_Lost;
}ANTI_ISLANDING	;


// Configure the period for each timer
#define EPWM_TIMER_TBPRD  14881  // Period register
#define EPWM_MAX_CMPA     14476
#define EPWM_MIN_CMPA       0
#define EPWM_MAX_CMPB     14476
#define EPWM_MIN_CMPB       0

// To keep track of which way the compare value is moving
#define EPWM_CMP_UP   1
#define EPWM_CMP_DOWN 0
#define EPWM_SOFTSTART_CNT 2000
#define NUMBER_OF_STEP_PER_60HZ_CYCLE 168	// PWM frequency: 168*60Hz = 10080Hz
#define NUMBER_OF_STEP_PER_50HZ_CYCLE 198	// PWM frequency: 198*50Hz = 9900Hz

#define	PI		3.141592654

// Constants define for boost control
#define	HALF_DUTY  EPWM_TIMER_TBPRD/2
#define	HALF_DUTY_X100  744050

#define	BOOST_WITH_I_LOOP	1
//#define	V_CONTROL_WITH_I_LOOP	1
#define	I_LIMIT		2000		// 0.1A/bit

//----------------------------------
// Boost PWM define
/***********************************
#define	BOOST_PWM_PERIOD		18292					// 300MHz /2 /18292 = 8200Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 18292 /2 = 9146
#define	BOOST_HALF_DUTY_X100   	914600					// Current loop Kp = 10.0, Ki = 0.1
#define	BOOST_DUTY_LIMIT		4500		
***********************************/

/***********************************
#define	BOOST_PWM_PERIOD		20000					// 300MHz /2 /20000 = 7500Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 20000 /2 = 10000
#define	BOOST_HALF_DUTY_X100   	1000000
#define	BOOST_DUTY_LIMIT		4500		
***********************************/

/***********************************/
#define	BOOST_PWM_PERIOD		23076					// 300MHz /2 /23076 = 6500Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 23076 /2 = 11538
#define	BOOST_HALF_DUTY_X100   	1153800					// Current loop Kp = 5.0, Ki = 0.08
#define	BOOST_DUTY_LIMIT		5000		
/***********************************/

/***********************************
#define	BOOST_PWM_PERIOD		15152					// 300MHz /2 /15152 = 9900Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 15152 /2 = 7576
#define	BOOST_HALF_DUTY_X100   	757600
#define	BOOST_DUTY_LIMIT		3500		
***********************************/

/***********************************
#define	BOOST_PWM_PERIOD		30304					// 300MHz /2 /30304 = 4950Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 30304 /2 = 15152
#define	BOOST_HALF_DUTY_X100   	1515200					// Current loop Kp = 10.0, Ki = 0.5
#define	BOOST_DUTY_LIMIT		7000
***********************************/

/***********************************
#define	BOOST_PWM_PERIOD		28962					// 300MHz /2 /28962 = 5040Hz
#define	BOOST_HALF_DUTY			BOOST_PWM_PERIOD/2		// 28962 /2 = 14481
#define	BOOST_HALF_DUTY_X100   	1448100					// Current loop Kp = 10.0, Ki = 0.5
#define	BOOST_DUTY_LIMIT		7000
***********************************/


#endif /* EPWM_CONTROL_H_ */

//===========================================================================
// End of file.
//===========================================================================
