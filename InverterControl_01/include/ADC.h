/**********************************************************************************
// File: ADC.h
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
// This is the header file define ADC access.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/20/12	|  J Wen 	| Original
**********************************************************************************/
#ifndef ADC_H_
#define ADC_H_

//---------------------------------------------------------------------------------
//--- ADS7865 access through zone_7 @ address: 0x200000

#define pADC_1 (Uint16 volatile *)	0x200000	//Zone 7 address for ADC1 (U17) (U21)
#define pADC_2 (Uint16 volatile *)	0x200020	//Zone 7 address for ADC2 (U18) (U24)
#define pADC_3 (Uint16 volatile *)	0x200040	//Zone 7 address for ADC3 (U7)  (U9)
#define pADC_4 (Uint16 volatile *)	0x200041	//Zone 7 address for ADC4 (U8)  

//--- DAC7725 access through zone_7 @ address: 0x20006x

#define pDAC_0 (Uint16 volatile *)	0x200060	//Zone 7 address for DAC0
#define pDAC_1 (Uint16 volatile *)	0x200061	//Zone 7 address for DAC1
#define pDAC_2 (Uint16 volatile *)	0x200062	//Zone 7 address for DAC2
#define pDAC_3 (Uint16 volatile *)	0x200063	//Zone 7 address for DAC3

#define		pDAC_Ptr	((volatile struct sDAC_RegWR *) 0x200060)

struct sDAC_RegWR {
	int16	Data_1;
	int16	Data_2;
	int16	Data_3;
	int16	Data_4;
	int16	Data_5;
};
// For AC channels
#define AC_CH_FACTOR	2.5/2047/16				// 2.5V per 0x7FF count, with 4 bit left shift

// ADC result data structure
typedef struct {
   float ADC_ch0;
   float ADC_ch1;
   float ADC_ch2;
   float ADC_ch3;
   float ADC_ch4;
   float ADC_ch5;
}ADC_RESULT;


// AC Line data structure
typedef struct {
   float V_AB_Line;
   float V_BC_Line;
   float V_CA_Line;
   float V_A;
   float V_B;
   float V_C;
   float V_Line_D;
   float V_Line_Q;
   float V_Line_Zero;
   float V_Line;
   int V_AB_Line_buff[210];
   int V_BC_Line_buff[210];
   int V_CA_Line_buff[210];
   float V_AB_RMS;
   float V_BC_RMS;
   float V_CA_RMS;
   Uint16 Line_buff_index;
   Uint16 V_CA_rising;

   float I_A;
   float I_B;
   float I_C;
   float I_Line_D;
   float I_Line_Q;
   float I_Line_D_filter;
   float I_Line_Q_filter;
   float I_Line_Zero;
   float I_Line;
   float I_Line_filter;
   int I_A_buff[210];
   int I_B_buff[210];
   int I_C_buff[210];
   float I_A_RMS;
   float I_B_RMS;
   float I_C_RMS;
   float Power_A;
   float Power_B;
   float Power_C;
}AC_MEASURE_DATA;

// sine refernce data structure
typedef struct {
   float sin_0_a;
   float sin_N120_b;
   float sin_P120_c;
   float cos_0_a;
   float cos_N120_b;
   float cos_P120_c;
   float sin_2nd_harmonic;
   float cos_2nd_harmonic;
   float sin_3rd_harmonic;
   float cos_3rd_harmonic;
   float sin_5th_harmonic;
   float cos_5th_harmonic;
   float ptr;
}SINE_REF;

// Grid-tie line current harmonic data structure
typedef struct {
   float I_D;
   float I_Q;
   float I_D_Fil;
   float I_Q_Fil;
   float I_D_com;
   float I_Q_com;
   float I_D_err;
   float I_Q_err;
   float I_D_integral;
   float I_Q_integral;
   float Ia;
   float Ib;
   float Ic;
   float Ia_cmd_ref;
   float Ib_cmd_ref;
   float Ic_cmd_ref;
   float Ia_in;
   float Ib_in;
   float Ic_in;
   float Ia_out;
   float Ib_out;
   float Ic_out;
   float Ki;
   float Kp;
   float Filter_K1;
   float Filter_K2;
}GT_CURRENT_HARMONICS;

// Selective filter data structure
typedef struct {
   float input_n1;
   float input_n2;
   float output_n1;
   float output_n2;
   float input;
   float output;
}SELETIVE_FILTER;

// Selective filter data structure
typedef struct {
   float input_n1;
   float output_n1;
   float input;
   float output;
}LOWPASS_FILTER;

// Calibration data structure
typedef struct {
   float V_ab;
   float V_bc;
   float V_ca;
   float I_a;
   float I_b;
   float I_c;
}CALIBRATION;


// ADC channels scaling factors define:
#define DC_BUS_1_SCALE_FACTOR 	323.43// old board 248.31
#define BOOST_CURRENT_SCALE_FACTOR -2.0*1200.0/8.0	// SKiiP1213 1200A/8V, circuit gain = 1/2
#define AC_LINE_SCALE_FACTOR	388.857				//100.0/0.31
#define AC_LINE_DQ_CONVERT_FACTOR	0.67266			//0.81;
#define INV_CURRENT_DQ_CONVERT_FACTOR	0.1464;
#define AC_CURRENT_DQ_CONVERT_FACTOR	0.1464;
#define DQ_AC_CURRENT_CONVERT_FACTOR	4.58664;

// Low pass filter: 10KHz sampling rate, 
// F(n) = K1 * F(n-1) + K2 * [ I(n) + I(n-1)]
// 2 msec time constant
//#define LOWPASS_FILTER_K1	0.95122
//#define LOWPASS_FILTER_K2	0.02439
// 10 msec time constant
//#define LOWPASS_FILTER_K1	0.99005
//#define LOWPASS_FILTER_K2	0.004975
// 30 msec time constant
#define LOWPASS_FILTER_K1	0.996672
#define LOWPASS_FILTER_K2	0.001664

#define OUTPUT_CURRENT_SCALE_FACTOR	 629.54		//604.36
#define ACLINE_CURRENT_SCALE_FACTOR	 -629.54	//604.36
#define GEN_CURRENT_SCALE_FACTOR	 201.1		// Note: CT 2011:1, load R = 10ohm, 100A - 0.497V, 201.1A/1V, //629.54		//604.36

//	alpha=(2*a-b-c)/3
//	beta=(c-b)/SQRT3
//	d=alpha*cos - beta*sin
//	q=alpha*sin + beta*cos		
#define ABC_TO_DQ(a,b,c,angle,sin,cos,alpha,beta,d,q)\
	do{sin=sin_table[angle];\
	cos=sin_table[(angle+NINETY_DEGREES)&ANGLE_MASK];\
	alpha=(long)((a<<1)-b-c)*(int)(4096/3.0)>>12;\
	beta=(long)(c-b)*(int)(4096/SQRT3)>>12;\
	d=((long)alpha*cos-(long)beta*sin)>>15;\
	q=((long)alpha*sin+(long)beta*cos)>>15;}while(0);

#define ABC_TO_DQ_float(a,b,c,sin,cos,alpha,beta,d,q)\
	do{alpha = (2.0*a - b - c)/3.0;\
	beta = 0.57735*(c - b);\
	d = alpha*cos - beta*sin;\
	q = alpha*sin + beta*cos;}while(0);
				
#define DQ_TO_ABC(a,b,c,sin,cos,d,q)\
	do{a=((long)d*cos + (long)q*sin)>>15;\
	c=((long)q*cos - (long)d*sin)>>15;\
	c=(long)c*(int)(4096*SQRT3)>>12;\
	b=(-a-c)>>1;c=(-a+c)>>1;}while(0);

#define DQ_TO_ABC_float(a,b,c,sin,cos,d,q)\
	do{a = d*cos + q*sin;\
	c = q*cos - d*sin;\
	c = c*1.732;\
	b = (-a-c)/2.0;\
	c = (-a+c)/2.0;}while(0);

#define PI_REG_PF(cmd,fbk,err,i_term,out,kp,ki,max,min)\
	{\
		err = cmd-fbk;\
		i_term += ki*err;\
		LIMIT_MAX_MIN(i_term,max,min)\
		out = kp*err+i_term;\
		LIMIT_MAX_MIN(out,max,min)\
	}

#define LIMIT_MAX_MIN(x,max,min) do{if((int)(x*1000.0)>(int)(max*1000.0)) x=max; else if((int)(x*1000.0)<(int)(min*1000.0)) x=min;}while(0);

#endif /* ADC_H_ */

