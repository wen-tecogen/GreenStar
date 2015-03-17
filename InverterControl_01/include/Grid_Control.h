/**********************************************************************************
// File: Grid_Control.h
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
// This is the header file define FPGA location.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/9/13	|  J Wen 	| Original
**********************************************************************************/
#ifndef GRID_CONTROL_H_
#define GRID_CONTROL_H_

//---------------------------------------------------------------------------------
// MicroGrid control constants define

#define P_VS_FREQ_MP		-0.005				// Power vs frequency Unit: -0.005Hz/KW
#define Q_VS_VOUT_MQ		0.2					// Reactive power vs output voltage Unit: 0.2V/KVA
#define P_MAX				1000				// Maximum power setting 125KW 0.1KW/bit
#define P_MIN				0					// Maximum power setting 0KW 0.1KW/bit
#define FREQ_UPPER_LIMIT	300					// 3Hz, 0.01Hz/bit		for 60Hz operation, 63Hz as max and 45Hz as min
#define FREQ_LOWER_LIMIT	1500				// 15Hz, 0.01Hz/bit
#define	PWR_CMD_INIT		80					// 8Kw, 0.1KW/bit

#define E0_MAX				5300				// 530V line-to-line voltage 0.1V/bit
#define E0_MIN				350					// 350V line-to-line voltage 0.1V/bit

#define	POWER_SCALING_FACTOR	0.00312/1.732		// convert to KW or KVA	 and convert line-line voltage to line-neutral

//---------------------------------------------------------------------------------
// Grid control data structure
// note: 

// Micro-Grid control data structure
typedef struct {
   float P;
   float Q;
   float KVA;
   float mP;
   float mQ;
   float V_cmd;
   float Freq_Delta;
   float PvsPmax_Delta;
   float PvsPmin_Delta;
   float Frequency;
   int	 Freq_Upper_Limit;
   int	 Freq_Lower_Limit;
   int	 P_cmd;				// Power command 0.1KW/bit
   int	 P_cmd_init;			// Power initial value
   int	 P_max_setting;		// Maximum power setting P_max 0.1KW/bit
   int	 P_min_setting;		// Minimum power setting P_min 0.1KW/bit
   Uint16 enable;
}MICRO_GRID;

typedef struct {
   float err;
   float integral;
   float int_old;
   float Ki;
   float Kp;
}PI_CONTROL;


//---------------------------------------------------------------------------------

#endif /* GRID_CONTROL_H_ */

