/**********************************************************************************
// File: ECAP_VLine.h
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

#ifndef ECAP_VLINE_H_
#define ECAP_VLINE_H_

typedef struct {
   float  Frequency;
   float  Phase;
   float  Voltage;
   float  Current;
}AC_LINE_DATA;

typedef struct {
   float  avg_frequency;
}AC_LINE_FREQUENCY;

typedef struct {
   Uint16	PLL_State;
   Uint16	PLL_Enable;
   Uint16   PWM_Period;
   Uint16   AC_Line_Measure_OK;
   float    PLL_error;
   float    PLL_Compensator;
   float    PLL_error_int;
   float    PLL_error_int_old;
   float    PLL_Kp;
   float    PLL_Ki;
   float    Q_command;
   Uint16	PhaseRotation_ABC;
   int	 Phase_Offset;
}PLL_DATA;

typedef struct {
	Uint32	Period[4];
	float	avg_Period;
	Uint16	index;
	Uint16	Capture;
	Uint16	CaptureCnt;
	Uint16	CaptureFilter;
	Uint16	CaptureMonitor;
}ECAP_DATA;

// eCAP constants define
#define CPU_FREQUENCY		300000000.0
#define NUMBER_OF_SAMPLES_PER_CYCLE	168
#define EC_RISING 0x0
#define EC_FALLING 0x1
// CTRRSTx bits
#define EC_ABS_MODE 0x0
#define EC_DELTA_MODE 0x1
// PRESCALE bits
#define EC_BYPASS 0x0
#define EC_DIV1 0x0
#define EC_DIV2 0x1
#define EC_DIV4 0x2
#define EC_DIV6 0x3
#define EC_DIV8 0x4
#define EC_DIV10 0x5
// ECCTL2 ( ECAP Control Reg 2)
//==========================
// CONT/ONESHOT bit
#define EC_CONTINUOUS 0x0
#define EC_ONESHOT 0x1
// STOPVALUE bit
#define EC_EVENT1 0x0
#define EC_EVENT2 0x1
#define EC_EVENT3 0x2
#define EC_EVENT4 0x3
// RE-ARM bit
#define EC_ARM 0x1
// TSCTRSTOP bit
#define EC_FREEZE 0x0
#define EC_RUN 0x1
// SYNCO_SEL bit
#define EC_SYNCIN 0x0
#define EC_CTR_PRD 0x1
#define EC_SYNCO_DIS 0x2
// CAP/APWM mode bit
#define EC_CAP_MODE 0x0
#define EC_APWM_MODE 0x1
// APWMPOL bit
#define EC_ACTV_HI 0x0
#define EC_ACTV_LO 0x1
// Generic
#define EC_DISABLE 0x0
#define EC_ENABLE 0x1
#define EC_FORCE 0x1

/*-----------------------------------------------------------------------------
Define PLL state.
-----------------------------------------------------------------------------*/
#define dFREQUENCY_UNLOCK		   	0
#define dFREQUENCY_LOCK		   		1
#define dPHASE_UNLOCK		   		2
#define dPHASE_LOCK_WORKING			3
#define dPHASE_LOCK		   			4

#define PLL_PHASE_ADJUSTMENT		0.98

#endif /* ECAP_VLINE_H_ */

//===========================================================================
// End of file.
//===========================================================================
