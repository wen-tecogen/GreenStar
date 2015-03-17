/**********************************************************************************
// File: DataMeasurement.h
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
// This is the header file define FPGA location.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 2/1/13	|  J Wen 	| Original
**********************************************************************************/
#ifndef DATAMEASUREMENT_H_
#define DATAMEASUREMENT_H_

typedef struct {
   float  speed_error;
   float  speed_error_pro;
   float  speed_error_pro_old;
   float  speed_error_int;
   float  speed_error_int_old;
   float  speed_error_der;
   float  speed_Loop_Compensator;
   float  S_Control_Ki;
   float  S_Control_Kp;
   float  S_Control_Kd;
   int 	  speed_ctrl_state;
}SPEED_CONTROL;

#define VAC_RMS_SCALE_FACTOR	0.064117		//0.07685
#define IAC_RMS_SCALE_FACTOR	0.007685
#define GEN_I_RMS_SCALE_FACTOR	0.7745
#define I_BOOST_SCALE_FACTOR	0.66667
#define POWER_SCALE_FACTOR		1.0/13000/3.0

#define OVER_LOAD_1SEC			10
#define OVER_LOAD_2SEC			20
#define OVER_LOAD_10SEC			100
#define OVER_LOAD_30SEC			300

#endif /* DATAMEASUREMENT_H_ */

