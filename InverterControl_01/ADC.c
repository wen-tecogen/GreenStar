/**********************************************************************************
// File: ADC.c
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
// This is the c file to access ADS7865 12bit A to D converter.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 12/14/12	|  J Wen 	| Original
**********************************************************************************/
#include <xdc/std.h>
#include <math.h>

#include "DSP28x_Project.h"     // Device Header-file and Examples Include File

#include "SystemControl.h"      // Header file for system control	   
#include "ADC.h"				// Header file for A to D converter	   
#include "EPWM_control.h"       // Header file for ePWM module		   
#include "ECAP_VLine.h"     	// Module Include File				   

// Variables used in this module
//float fTemp1, fTemp2, fTemp3;
volatile Uint16* address_ptr;
Uint16 DAC_val;

ADC_RESULT	ADC1;				// ADC chip (U17)	U21
ADC_RESULT	ADC2;				// ADC chip (U18)	U24
ADC_RESULT	ADC3;				// ADC chip (U7) 	U9
ADC_RESULT	ADC4;				// ADC chip (U8) 	

AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
AC_MEASURE_DATA 	VAC_out_Measurement;		// Inverter Output data
SINE_REF			sine_ref;
GT_CURRENT_HARMONICS	GT_5th_harm_ctrl;
GT_CURRENT_HARMONICS	GT_2nd_harm_ctrl;

//------ Selective filters constants ------
float	I_Line_SF_K1, I_Line_SF_K2, I_Line_SF_K3;
#define		SFILTER_K1	0.0098975				// Notes: 
#define		SFILTER_K2	1.9787983				// fundemantal frequency = 60Hz
#define		SFILTER_K3	-0.9802050				// sampling rate = 10KHz
// Selective filters for switching current	 		
SELETIVE_FILTER		InverterOut_Ia_SFilter;
SELETIVE_FILTER		InverterOut_Ib_SFilter;
SELETIVE_FILTER		InverterOut_Ic_SFilter;
// Selective filters for AC line voltage	 		
SELETIVE_FILTER		ACline_Va_SFilter;
SELETIVE_FILTER		ACline_Vb_SFilter;
SELETIVE_FILTER		ACline_Vc_SFilter;
// Selective filters for AC line current	 		
SELETIVE_FILTER		ACline_Ia_SFilter;
SELETIVE_FILTER		ACline_Ib_SFilter;
SELETIVE_FILTER		ACline_Ic_SFilter;

//------ Lowpass filters for measurement ------
float	I_line_filter_K1, I_line_filter_K2;	
LOWPASS_FILTER	AC_Line_ID_Lowpass;
LOWPASS_FILTER	AC_Line_IQ_Lowpass;
LOWPASS_FILTER	AC_Line_I_Lowpass;

CALIBRATION		Calibration;

Uint16	Buff_cnt;
int 	V_CA_old;
Uint16	V_CA_rising_cnt;
Uint16	V_CA_falling_cnt;

Uint16	DAC_OutputCtrl;

// variables for ADC channels
float	DC_BUS_1;
float	Boost_Current;

// variables for V_Line D/Q filtering
float	VAC_Line_DI, VAC_Line_DI_Old;
float	VAC_Line_DF, VAC_Line_DF_Old;
float	VAC_Line_QI, VAC_Line_QI_Old;
float	VAC_Line_QF, VAC_Line_QF_Old;

// Generator current ADC channels
float	Generator_IA[210];
float	Generator_IB[210];
float	Generator_IC[210];
int     Generator_Current_Limit;
int     Generator_Current_Limit_cnt;

// AC Line current offset and adjustment
float	AC_Line_A_Offset;
float	AC_Line_B_Offset;
float	AC_Line_C_Offset;
float	AC_Line_A_Zero;
float	AC_Line_B_Zero;
float	AC_Line_C_Zero;


// External variables used in this module
extern EPWM_INFO 		epwm1_info;
extern EPWM_INFO 		epwm2_info;
extern EPWM_INFO 		epwm3_info;

extern ECAP_DATA	eCAP1_Data;
extern OPERATION_DATA 	OperationData;
extern SYSTEM_INFO 	system;
extern INVERTER_CONTROL Inverter_SysControl;
extern GRID_TIE	iGrid_Tie;
extern SV_GEN			svgen1;
extern ANTI_ISLANDING	Anti_Islanding;

extern Uint16		Phase_Lock;
extern float	Engine_Speed_RPM;
extern int	 	InputStageFilter;

/******* Function prototype *******/
void ADC1_init(void);
void ADC1_read(void);
void ADC2_init(void);
void ADC2_read(void);
void ADC3_init(void);
void ADC3_read(void);
void ADC4_init(void);
void ADC4_read(void);
void ADC_vars_init(void);

void DAC_output(void);


/**********************************************************************
// Function: ADC1_init()
// Description: initialize ADC1 (U21).
// Revisions:
// ----------
**********************************************************************/
void ADC1_init() {

	// Configures the ADS7865 ADC1 (U17) on ControlPCB  by writing commands to the ADCs.

	// Setup Vref on DAC
	address_ptr = pADC_1;			// ADC1 is connected to Zone7, which is at address 0x200000
	*address_ptr = 0x0105;			// RESET DEVICE
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_1;				// Read Vref DAC value
	*address_ptr = 0x0101;			// Setup new Vref DAC value write
	*address_ptr = 1023;			// Write new value to Vref DAC (value for 3.3V/2)
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_1;				// Read NEW Vref DAC value

	// ADC1
	// Setup SEQUENCER to convert pseudo-diff mode in the following sequence:
	// individual convst and busy for each conversion for all channels in
	// the following order.
	// CH(A/B)0+, CH(A/B)1+, then CH(A/B)1-
	address_ptr = pADC_1;
	*address_ptr = 0x104;			// Setup new sequencer value write
	*address_ptr = 0xF24;			// Configure sequencer (S = 11: single. convst and busy for all conversion, SL = 11: sequence length 3)
									// Sequence is as follows: (CH1 = 0: CHA0+/CHB0+, CM1 = 0:CHA0-/CHB0-; (pin B0)
									//							CH2 = 1: CHA1+/CHB1+, CM2 = 0:CHA0-/CHB0-; (pin B1)
									//							CH3 = 0: CHA1-/CHB1-, CM3 = 1:CHA0-/CHB0-; (pin B2)
	*address_ptr = 0x106;			// Setup sequencer value readback
	DAC_val = *pADC_1;	// Read sequencer value

	// Selective filter initialization
	InverterOut_Ia_SFilter.input_n1 = 0.0;
	InverterOut_Ia_SFilter.input_n2 = 0.0;
	InverterOut_Ia_SFilter.output_n1 = 0.0;
	InverterOut_Ia_SFilter.output_n2 = 0.0;
		
	InverterOut_Ib_SFilter.input_n1 = 0.0;
	InverterOut_Ib_SFilter.input_n2 = 0.0;
	InverterOut_Ib_SFilter.output_n1 = 0.0;
	InverterOut_Ib_SFilter.output_n2 = 0.0;
		
	InverterOut_Ic_SFilter.input_n1 = 0.0;
	InverterOut_Ic_SFilter.input_n2 = 0.0;
	InverterOut_Ic_SFilter.output_n1 = 0.0;
	InverterOut_Ic_SFilter.output_n2 = 0.0;

	DAC_OutputCtrl = 0;
}

/**********************************************************************
// Function: ADC2_init()
// Description: initialize ADC2 (U18).
// Revisions:
// ----------
**********************************************************************/
void ADC2_init() {
int i;

	// Configures the ADS7865 ADC2 (U18) on ControlPCB  by writing commands to the ADCs.

	// Setup Vref on DAC
	address_ptr = pADC_2;			// ADC1 is connected to Zone7, which is at address 0x200020
	*address_ptr = 0x0105;			// RESET DEVICE
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_2;				// Read Vref DAC value
	*address_ptr = 0x0101;			// Setup new Vref DAC value write
	*address_ptr = 1023;			// Write new value to Vref DAC (value for 3.3V/2)
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_2;				// Read NEW Vref DAC value

	// ADC1
	// Setup SEQUENCER to convert pseudo-diff mode in the following sequence:
	// individual convst and busy for each conversion for all channels in
	// the following order.
	// CH(A/B)0+, CH(A/B)1+, then CH(A/B)1-
	address_ptr = pADC_2;
	*address_ptr = 0x104;			// Setup new sequencer value write
	*address_ptr = 0xF24;			// Configure sequencer (S = 11: single. convst and busy for all conversion, SL = 11: sequence length 3)
									// Sequence is as follows: (CH1 = 0: CHA0+/CHB0+, CM1 = 0:CHA0-/CHB0-; (pin B0)
									//							CH2 = 1: CHA1+/CHB1+, CM2 = 0:CHA0-/CHB0-; (pin B1)
									//							CH3 = 0: CHA1-/CHB1-, CM3 = 1:CHA0-/CHB0-; (pin B2)
	*address_ptr = 0x106;			// Setup sequencer value readback
	DAC_val = *pADC_2;	// Read sequencer value

	AC_Line_Measurement.Line_buff_index = 0;
	VAC_out_Measurement.Line_buff_index = 0;
	Buff_cnt = 0;
	for (i = 0; i<200; i++) {
		AC_Line_Measurement.V_AB_Line_buff[i] = 0;
		AC_Line_Measurement.V_BC_Line_buff[i] = 0;
		AC_Line_Measurement.V_CA_Line_buff[i] = 0;
	}
	V_CA_old = 0;
	AC_Line_Measurement.V_CA_rising = FALSE;
	V_CA_rising_cnt	  = 0;
	V_CA_falling_cnt  = 0;

	VAC_Line_DI = 0.0; 
	VAC_Line_DI_Old = 0.0;
	VAC_Line_DF = 0.0; 
	VAC_Line_DF_Old = 0.0;
	VAC_Line_QI = 0.0; 
	VAC_Line_QI_Old = 0.0;
	VAC_Line_QF = 0.0; 
	VAC_Line_QF_Old = 0.0;

	// Selective filter initialization
	ACline_Va_SFilter.input_n1 = 0.0;
	ACline_Va_SFilter.input_n2 = 0.0;
	ACline_Va_SFilter.output_n1 = 0.0;
	ACline_Va_SFilter.output_n2 = 0.0;
		
	ACline_Vb_SFilter.input_n1 = 0.0;
	ACline_Vb_SFilter.input_n2 = 0.0;
	ACline_Vb_SFilter.output_n1 = 0.0;
	ACline_Vb_SFilter.output_n2 = 0.0;
		
	ACline_Vc_SFilter.input_n1 = 0.0;
	ACline_Vc_SFilter.input_n2 = 0.0;
	ACline_Vc_SFilter.output_n1 = 0.0;
	ACline_Vc_SFilter.output_n2 = 0.0;

}

/**********************************************************************
// Function: ADC3_init()
// Description: initialize ADC3 (U7).
// Revisions:
// ----------
**********************************************************************/
void ADC3_init() {

	// Configures the ADS7865 ADC3 (U9) on ControlPCB  by writing commands to the ADCs.

	// Setup Vref on DAC
	address_ptr = pADC_3;			// ADC1 is connected to Zone7, which is at address 0x200040
	*address_ptr = 0x0105;			// RESET DEVICE
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_3;				// Read Vref DAC value
	*address_ptr = 0x0101;			// Setup new Vref DAC value write
	*address_ptr = 1023;			// Write new value to Vref DAC (value for 3.3V/2)
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_3;				// Read NEW Vref DAC value

	// ADC1
	// Setup SEQUENCER to convert pseudo-diff mode in the following sequence:
	// individual convst and busy for each conversion for all channels in
	// the following order.
	// CH(A/B)0+, CH(A/B)1+, then CH(A/B)1-
	address_ptr = pADC_3;
	*address_ptr = 0x104;			// Setup new sequencer value write
	*address_ptr = 0xF24;			// Configure sequencer (S = 11: single. convst and busy for all conversion, SL = 11: sequence length 3)
									// Sequence is as follows: (CH1 = 0: CHA0+/CHB0+, CM1 = 0:CHA0-/CHB0-; (pin B0)
									//							CH2 = 1: CHA1+/CHB1+, CM2 = 0:CHA0-/CHB0-; (pin B1)
									//							CH3 = 0: CHA1-/CHB1-, CM3 = 1:CHA0-/CHB0-; (pin B2)
	*address_ptr = 0x106;			// Setup sequencer value readback
	DAC_val = *pADC_3;	// Read sequencer value

}

/**********************************************************************
// Function: ADC4_init()
// Description: initialize ADC4 (U8).
// Revisions:
// ----------
**********************************************************************/
void ADC4_init() {

	// Configures the ADS7865 ADC4 (U8) on ControlPCB  by writing commands to the ADCs.

	// Setup Vref on DAC
	address_ptr = pADC_4;			// ADC4 is connected to Zone7, which is at address 0x200041
	*address_ptr = 0x0105;			// RESET DEVICE
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_4;				// Read Vref DAC value
	*address_ptr = 0x0101;			// Setup new Vref DAC value write
	*address_ptr = 1023;			// Write new value to Vref DAC (value for 3.3V/2)
	*address_ptr = 0x0103;			// Setup Vref DAC value read
	DAC_val = *pADC_4;				// Read NEW Vref DAC value

	// ADC4
	// Setup SEQUENCER to convert pseudo-diff mode in the following sequence:
	// individual convst and busy for each conversion for all channels in
	// the following order.
	// CH(A/B)0+, CH(A/B)1+, then CH(A/B)1-
	address_ptr = pADC_4;
	*address_ptr = 0x104;			// Setup new sequencer value write
	*address_ptr = 0xF24;			// Configure sequencer (S = 11: single. convst and busy for all conversion, SL = 11: sequence length 3)
									// Sequence is as follows: (CH1 = 0: CHA0+/CHB0+, CM1 = 0:CHA0-/CHB0-; (pin B0)
									//							CH2 = 1: CHA1+/CHB1+, CM2 = 0:CHA0-/CHB0-; (pin B1)
									//							CH3 = 0: CHA1-/CHB1-, CM3 = 1:CHA0-/CHB0-; (pin B2)
	*address_ptr = 0x106;			// Setup sequencer value readback
	DAC_val = *pADC_4;	// Read sequencer value

	// Selective filter initialization
	ACline_Ia_SFilter.input_n1 = 0.0;
	ACline_Ia_SFilter.input_n2 = 0.0;
	ACline_Ia_SFilter.output_n1 = 0.0;
	ACline_Ia_SFilter.output_n2 = 0.0;
		
	ACline_Ib_SFilter.input_n1 = 0.0;
	ACline_Ib_SFilter.input_n2 = 0.0;
	ACline_Ib_SFilter.output_n1 = 0.0;
	ACline_Ib_SFilter.output_n2 = 0.0;
		
	ACline_Ic_SFilter.input_n1 = 0.0;
	ACline_Ic_SFilter.input_n2 = 0.0;
	ACline_Ic_SFilter.output_n1 = 0.0;
	ACline_Ic_SFilter.output_n2 = 0.0;

	Generator_Current_Limit = 315;
	Generator_Current_Limit_cnt = 0;

	AC_Line_A_Offset = 0.0;
	AC_Line_B_Offset = 0.0;
	AC_Line_C_Offset = 0.0;
	AC_Line_A_Zero = 0.0;
	AC_Line_B_Zero = 0.0;
	AC_Line_C_Zero = 0.0;

//	AC_Line_ID_Lowpass.input_n1 = 0.0;	
//	AC_Line_IQ_Lowpass.input_n1 = 0.0;	
//	AC_Line_I_Lowpass.input_n1 = 0.0;
//	AC_Line_ID_Lowpass.output_n1 = 0.0;	
//	AC_Line_IQ_Lowpass.output_n1 = 0.0;	
//	AC_Line_I_Lowpass.output_n1 = 0.0;	
//	I_line_filter_K1 = 0.9;
//	I_line_filter_K2 = 0.05;

}

/**********************************************************************
// Function: ADC_vars_init()
// Description: initialize ADC2 (U18).
// Revisions:
// ----------
**********************************************************************/
void ADC_vars_init() {
int i;
	AC_Line_Measurement.Line_buff_index = 0;
	VAC_out_Measurement.Line_buff_index = 0;
	Buff_cnt = 0;
	for (i = 0; i<200; i++) {
		AC_Line_Measurement.V_AB_Line_buff[i] = 0;
		AC_Line_Measurement.V_BC_Line_buff[i] = 0;
		AC_Line_Measurement.V_CA_Line_buff[i] = 0;
		Generator_IA[i] = 0;
		Generator_IB[i] = 0;
		Generator_IC[i] = 0;
		AC_Line_Measurement.I_A_buff[i] = 0;
		AC_Line_Measurement.I_B_buff[i] = 0;
		AC_Line_Measurement.I_C_buff[i] = 0;
	}
	V_CA_old = 0;
	AC_Line_Measurement.V_CA_rising = FALSE;
	V_CA_rising_cnt	  = 0;
	V_CA_falling_cnt  = 0;

	VAC_Line_DI = 0.0; 
	VAC_Line_DI_Old = 0.0;
	VAC_Line_DF = 0.0; 
	VAC_Line_DF_Old = 0.0;
	VAC_Line_QI = 0.0; 
	VAC_Line_QI_Old = 0.0;
	VAC_Line_QF = 0.0; 
	VAC_Line_QF_Old = 0.0;

	// Selective filter initialization
	InverterOut_Ia_SFilter.input_n1 = 0.0;
	InverterOut_Ia_SFilter.input_n2 = 0.0;
	InverterOut_Ia_SFilter.output_n1 = 0.0;
	InverterOut_Ia_SFilter.output_n2 = 0.0;
		
	InverterOut_Ib_SFilter.input_n1 = 0.0;
	InverterOut_Ib_SFilter.input_n2 = 0.0;
	InverterOut_Ib_SFilter.output_n1 = 0.0;
	InverterOut_Ib_SFilter.output_n2 = 0.0;
		
	InverterOut_Ic_SFilter.input_n1 = 0.0;
	InverterOut_Ic_SFilter.input_n2 = 0.0;
	InverterOut_Ic_SFilter.output_n1 = 0.0;
	InverterOut_Ic_SFilter.output_n2 = 0.0;

	// Selective filter initialization
	ACline_Va_SFilter.input_n1 = 0.0;
	ACline_Va_SFilter.input_n2 = 0.0;
	ACline_Va_SFilter.output_n1 = 0.0;
	ACline_Va_SFilter.output_n2 = 0.0;
		
	ACline_Vb_SFilter.input_n1 = 0.0;
	ACline_Vb_SFilter.input_n2 = 0.0;
	ACline_Vb_SFilter.output_n1 = 0.0;
	ACline_Vb_SFilter.output_n2 = 0.0;
		
	ACline_Vc_SFilter.input_n1 = 0.0;
	ACline_Vc_SFilter.input_n2 = 0.0;
	ACline_Vc_SFilter.output_n1 = 0.0;
	ACline_Vc_SFilter.output_n2 = 0.0;

	// Selective filter initialization
	ACline_Ia_SFilter.input_n1 = 0.0;
	ACline_Ia_SFilter.input_n2 = 0.0;
	ACline_Ia_SFilter.output_n1 = 0.0;
	ACline_Ia_SFilter.output_n2 = 0.0;
		
	ACline_Ib_SFilter.input_n1 = 0.0;
	ACline_Ib_SFilter.input_n2 = 0.0;
	ACline_Ib_SFilter.output_n1 = 0.0;
	ACline_Ib_SFilter.output_n2 = 0.0;
		
	ACline_Ic_SFilter.input_n1 = 0.0;
	ACline_Ic_SFilter.input_n2 = 0.0;
	ACline_Ic_SFilter.output_n1 = 0.0;
	ACline_Ic_SFilter.output_n2 = 0.0;

	I_Line_SF_K1 = 0.49991;
	I_Line_SF_K2 = 0.99947;
	I_Line_SF_K3 = -0.00018;

}


/**********************************************************************
// Function: ADC1_read()
// Description: get ADC result from ADC1 (U21).
// Revisions:
// ----------
**********************************************************************/
void ADC1_read() {
float fTemp0, fTemp1, fTemp2;
//	while(GpioDataRegs.GPADAT.bit.GPIO27==1){	// Read after busy
//		asm(" NOP");
//    }
	ADC1.ADC_ch0 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHA0+ (B0)
	ADC1.ADC_ch3 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHB0+ (B3)
	ADC1.ADC_ch1 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHA1+ (B1)
	ADC1.ADC_ch4 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHB1+ (B4)
	ADC1.ADC_ch2 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHA1- (B2)
	ADC1.ADC_ch5 = (float) AC_CH_FACTOR * (int)(*pADC_1<<4);				// Read CHB1- (B6)

	Boost_Current = ADC1.ADC_ch3 * BOOST_CURRENT_SCALE_FACTOR;
	// Inverter switching current:
	VAC_out_Measurement.I_A	= ADC1.ADC_ch0 * OUTPUT_CURRENT_SCALE_FACTOR;
	VAC_out_Measurement.I_B	= ADC1.ADC_ch1 * OUTPUT_CURRENT_SCALE_FACTOR;
	VAC_out_Measurement.I_C	= ADC1.ADC_ch2 * OUTPUT_CURRENT_SCALE_FACTOR;

	// Add selective filter for all 3 channels
	// u(n)=K1*[i(n)-i(n-2)] + K2*u(n-1) + K3*u(n-2)
	// phase A current:
	fTemp0 = SFILTER_K1 * (VAC_out_Measurement.I_A - InverterOut_Ia_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * InverterOut_Ia_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * InverterOut_Ia_SFilter.output_n2;
	InverterOut_Ia_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	InverterOut_Ia_SFilter.input_n2 = InverterOut_Ia_SFilter.input_n1;
	InverterOut_Ia_SFilter.input_n1 = VAC_out_Measurement.I_A;
	InverterOut_Ia_SFilter.output_n2 = InverterOut_Ia_SFilter.output_n1;
	InverterOut_Ia_SFilter.output_n1 = InverterOut_Ia_SFilter.output;

	// phase B current:
	fTemp0 = SFILTER_K1 * (VAC_out_Measurement.I_B - InverterOut_Ib_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * InverterOut_Ib_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * InverterOut_Ib_SFilter.output_n2;
	InverterOut_Ib_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	InverterOut_Ib_SFilter.input_n2 = InverterOut_Ib_SFilter.input_n1;
	InverterOut_Ib_SFilter.input_n1 = VAC_out_Measurement.I_B;
	InverterOut_Ib_SFilter.output_n2 = InverterOut_Ib_SFilter.output_n1;
	InverterOut_Ib_SFilter.output_n1 = InverterOut_Ib_SFilter.output;

	// phase C current:
	fTemp0 = SFILTER_K1 * (VAC_out_Measurement.I_C - InverterOut_Ic_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * InverterOut_Ic_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * InverterOut_Ic_SFilter.output_n2;
	InverterOut_Ic_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	InverterOut_Ic_SFilter.input_n2 = InverterOut_Ic_SFilter.input_n1;
	InverterOut_Ic_SFilter.input_n1 = VAC_out_Measurement.I_C;
	InverterOut_Ic_SFilter.output_n2 = InverterOut_Ic_SFilter.output_n1;
	InverterOut_Ic_SFilter.output_n1 = InverterOut_Ic_SFilter.output;
	

	// AC line current D value
	VAC_out_Measurement.I_Line_D = VAC_out_Measurement.I_A * sine_ref.sin_0_a 
								 + VAC_out_Measurement.I_B * sine_ref.sin_N120_b 
								 + VAC_out_Measurement.I_C * sine_ref.sin_P120_c;
	VAC_out_Measurement.I_Line_D *= INV_CURRENT_DQ_CONVERT_FACTOR;

	// AC line current Q value
	VAC_out_Measurement.I_Line_Q = VAC_out_Measurement.I_A * sine_ref.cos_0_a 
								 + VAC_out_Measurement.I_B * sine_ref.cos_N120_b
								 + VAC_out_Measurement.I_C * sine_ref.cos_P120_c;
	VAC_out_Measurement.I_Line_Q *= INV_CURRENT_DQ_CONVERT_FACTOR;

	// AC line current Zero value
	VAC_out_Measurement.I_Line_Zero = VAC_out_Measurement.I_A + VAC_out_Measurement.I_B + VAC_out_Measurement.I_C;
	VAC_out_Measurement.I_Line_Zero *= INV_CURRENT_DQ_CONVERT_FACTOR;

	// instant AC line current
	VAC_out_Measurement.I_Line = sqrt(VAC_out_Measurement.I_Line_D * VAC_out_Measurement.I_Line_D 
								+ VAC_out_Measurement.I_Line_Q * VAC_out_Measurement.I_Line_Q);


//	// Fill data buffer
//	AC_Line_Measurement.Line_buff_index ++;
//	//if (Phase_Lock != TRUE) {
//	if (TRUE) {
//		if(AC_Line_Measurement.Line_buff_index >= 167) {  // For 60Hz only!!!
//			AC_Line_Measurement.Line_buff_index = 0;
//		}
//	}
//	AC_Line_Measurement.I_A_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_A * 10.0);
//	AC_Line_Measurement.I_B_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_B * 10.0);
//	AC_Line_Measurement.I_C_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_C * 10.0);
}

/**********************************************************************
// Function: ADC2_read()
// Description: get ADC result from ADC2 (U24).
// Revisions:
// ----------
**********************************************************************/
void ADC2_read() {
float fTemp0, fTemp1, fTemp2;
//	while(GpioDataRegs.GPADAT.bit.GPIO27==1){	// Read after busy
//		asm(" NOP");
//    }
	ADC2.ADC_ch0 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHA0+ (B0)
	ADC2.ADC_ch3 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHB0+ (B3)
	ADC2.ADC_ch1 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHA1+ (B1)
	ADC2.ADC_ch4 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHB1+ (B4)
	ADC2.ADC_ch2 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHA1- (B2)
	ADC2.ADC_ch5 = (float) AC_CH_FACTOR * (int)(*pADC_2<<4);				// Read CHB1- (B6)

	VAC_out_Measurement.V_BC_Line = ADC2.ADC_ch1 * AC_LINE_SCALE_FACTOR;
	VAC_out_Measurement.V_CA_Line = ADC2.ADC_ch0 * AC_LINE_SCALE_FACTOR;
	VAC_out_Measurement.V_AB_Line = ADC2.ADC_ch2 * AC_LINE_SCALE_FACTOR;

	AC_Line_Measurement.V_BC_Line = ADC2.ADC_ch4 * AC_LINE_SCALE_FACTOR;
	AC_Line_Measurement.V_CA_Line = ADC2.ADC_ch3 * AC_LINE_SCALE_FACTOR;
	AC_Line_Measurement.V_AB_Line = ADC2.ADC_ch5 * AC_LINE_SCALE_FACTOR;

	VAC_out_Measurement.V_A	= (VAC_out_Measurement.V_AB_Line - VAC_out_Measurement.V_CA_Line)/3.0;
	VAC_out_Measurement.V_B	= (VAC_out_Measurement.V_BC_Line - VAC_out_Measurement.V_AB_Line)/3.0;
	VAC_out_Measurement.V_C	= (VAC_out_Measurement.V_CA_Line - VAC_out_Measurement.V_BC_Line)/3.0;

	AC_Line_Measurement.V_A	= (AC_Line_Measurement.V_AB_Line - AC_Line_Measurement.V_CA_Line)/3.0;
	AC_Line_Measurement.V_B	= (AC_Line_Measurement.V_BC_Line - AC_Line_Measurement.V_AB_Line)/3.0;
	AC_Line_Measurement.V_C	= (AC_Line_Measurement.V_CA_Line - AC_Line_Measurement.V_BC_Line)/3.0;

	// Add selective filter for all 3 line voltage channels
	// u(n)=K1*[i(n)-i(n-2)] + K2*u(n-1) + K3*u(n-2)
	// phase A voltage:
	fTemp0 = SFILTER_K1 * (AC_Line_Measurement.V_A - ACline_Va_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * ACline_Va_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * ACline_Va_SFilter.output_n2;
	ACline_Va_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Va_SFilter.input_n2 = ACline_Va_SFilter.input_n1;
	ACline_Va_SFilter.input_n1 = AC_Line_Measurement.V_A;
	ACline_Va_SFilter.output_n2 = ACline_Va_SFilter.output_n1;
	ACline_Va_SFilter.output_n1 = ACline_Va_SFilter.output;

	// phase B voltage:
	fTemp0 = SFILTER_K1 * (AC_Line_Measurement.V_B - ACline_Vb_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * ACline_Vb_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * ACline_Vb_SFilter.output_n2;
	ACline_Vb_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Vb_SFilter.input_n2 = ACline_Vb_SFilter.input_n1;
	ACline_Vb_SFilter.input_n1 = AC_Line_Measurement.V_B;
	ACline_Vb_SFilter.output_n2 = ACline_Vb_SFilter.output_n1;
	ACline_Vb_SFilter.output_n1 = ACline_Vb_SFilter.output;

	// phase C voltage:
	fTemp0 = SFILTER_K1 * (AC_Line_Measurement.V_C - ACline_Vc_SFilter.input_n2);
	fTemp1 = SFILTER_K2 * ACline_Vc_SFilter.output_n1;
	fTemp2 = SFILTER_K3 * ACline_Vc_SFilter.output_n2;
	ACline_Vc_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Vc_SFilter.input_n2 = ACline_Vc_SFilter.input_n1;
	ACline_Vc_SFilter.input_n1 = AC_Line_Measurement.V_C;
	ACline_Vc_SFilter.output_n2 = ACline_Vc_SFilter.output_n1;
	ACline_Vc_SFilter.output_n1 = ACline_Vc_SFilter.output;
	
//************************************************************************************************
// Inverter output:
	VAC_Line_DI = VAC_out_Measurement.V_A * sine_ref.sin_0_a 
				+ VAC_out_Measurement.V_B * sine_ref.sin_N120_b 
				+ VAC_out_Measurement.V_C * sine_ref.sin_P120_c;
	VAC_Line_DI *= AC_LINE_DQ_CONVERT_FACTOR;
	//--- Low pass filter ---
	// F(n) = K1 * F(n-1) + K2 * [ I(n) + I(n-1)]
	//fTemp1 = VAC_Line_DF_Old * LOWPASS_FILTER_K1;
	//fTemp2 = (VAC_Line_DI + VAC_Line_DI_Old) * LOWPASS_FILTER_K2;
	//VAC_Line_DF = fTemp1 + fTemp2;
	//VAC_Line_DI_Old = VAC_Line_DI;
	//VAC_Line_DF_Old = VAC_Line_DF;
	VAC_out_Measurement.V_Line_D = VAC_Line_DI;
//	VAC_out_Measurement.V_Line_D = VAC_Line_DF;

	// AC line voltage Q value
	VAC_Line_QI = VAC_out_Measurement.V_A * sine_ref.cos_0_a 
				+ VAC_out_Measurement.V_B * sine_ref.cos_N120_b 
				+ VAC_out_Measurement.V_C * sine_ref.cos_P120_c;
	VAC_Line_QI *= AC_LINE_DQ_CONVERT_FACTOR;
	//--- Low pass filter ---
	// F(n) = K1 * F(n-1) + K2 * [ I(n) + I(n-1)]
	//fTemp1 = VAC_Line_QF_Old * LOWPASS_FILTER_K1;
	//fTemp2 = (VAC_Line_QI + VAC_Line_QI_Old) * LOWPASS_FILTER_K2;
	//VAC_Line_QF = fTemp1 + fTemp2;
	//VAC_Line_QI_Old = VAC_Line_QI;
	//VAC_Line_QF_Old = VAC_Line_QF;
	VAC_out_Measurement.V_Line_Q = VAC_Line_QI;
	//VAC_out_Measurement.V_Line_Q = VAC_Line_QF;

	// VAC_out voltage Zero value
	VAC_out_Measurement.V_Line_Zero = VAC_out_Measurement.V_A + VAC_out_Measurement.V_B + VAC_out_Measurement.V_C;
	VAC_out_Measurement.V_Line_Zero *= AC_LINE_DQ_CONVERT_FACTOR;

// instant VAC out voltage
	VAC_out_Measurement.V_Line = sqrt(VAC_out_Measurement.V_Line_D * VAC_out_Measurement.V_Line_D 
								+ VAC_out_Measurement.V_Line_Q * VAC_out_Measurement.V_Line_Q);

	// Fill data buffer
	VAC_out_Measurement.V_AB_Line_buff[VAC_out_Measurement.Line_buff_index] = (int)(VAC_out_Measurement.V_AB_Line * 10.0);
	VAC_out_Measurement.V_BC_Line_buff[VAC_out_Measurement.Line_buff_index] = (int)(VAC_out_Measurement.V_BC_Line * 10.0);
	VAC_out_Measurement.V_CA_Line_buff[VAC_out_Measurement.Line_buff_index] = (int)(VAC_out_Measurement.V_CA_Line * 10.0);
//************************************************************************************************
//	// AC line voltage D value

	VAC_Line_DI = AC_Line_Measurement.V_A * sine_ref.sin_0_a 
				+ AC_Line_Measurement.V_B * sine_ref.sin_N120_b 
				+ AC_Line_Measurement.V_C * sine_ref.sin_P120_c;
	VAC_Line_DI *= AC_LINE_DQ_CONVERT_FACTOR;
	//--- Low pass filter ---
	// F(n) = K1 * F(n-1) + K2 * [ I(n) + I(n-1)]
	fTemp1 = VAC_Line_DF_Old * LOWPASS_FILTER_K1;
	fTemp2 = (VAC_Line_DI + VAC_Line_DI_Old) * LOWPASS_FILTER_K2;
	VAC_Line_DF = fTemp1 + fTemp2;
	VAC_Line_DI_Old = VAC_Line_DI;
	VAC_Line_DF_Old = VAC_Line_DF;
//	AC_Line_Measurement.V_Line_D = VAC_Line_DI;
	AC_Line_Measurement.V_Line_D = VAC_Line_DF;

	// AC line voltage Q value
	VAC_Line_QI = AC_Line_Measurement.V_A * sine_ref.cos_0_a 
				+ AC_Line_Measurement.V_B * sine_ref.cos_N120_b 
				+ AC_Line_Measurement.V_C * sine_ref.cos_P120_c;
	VAC_Line_QI *= AC_LINE_DQ_CONVERT_FACTOR;
	//--- Low pass filter ---
	// F(n) = K1 * F(n-1) + K2 * [ I(n) + I(n-1)]
	fTemp1 = VAC_Line_QF_Old * LOWPASS_FILTER_K1;
	fTemp2 = (VAC_Line_QI + VAC_Line_QI_Old) * LOWPASS_FILTER_K2;
	VAC_Line_QF = fTemp1 + fTemp2;
	VAC_Line_QI_Old = VAC_Line_QI;
	VAC_Line_QF_Old = VAC_Line_QF;
//	AC_Line_Measurement.V_Line_Q = VAC_Line_QI;
	AC_Line_Measurement.V_Line_Q = VAC_Line_QF;

	// AC line voltage Zero value
	AC_Line_Measurement.V_Line_Zero = AC_Line_Measurement.V_A + AC_Line_Measurement.V_B + AC_Line_Measurement.V_C;
	AC_Line_Measurement.V_Line_Zero *= AC_LINE_DQ_CONVERT_FACTOR;

	// instant AC line voltage
	AC_Line_Measurement.V_Line = sqrt(AC_Line_Measurement.V_Line_D * AC_Line_Measurement.V_Line_D 
								+ AC_Line_Measurement.V_Line_Q * AC_Line_Measurement.V_Line_Q);

	// Fill data buffer
	AC_Line_Measurement.V_AB_Line_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.V_AB_Line * 10.0);
	AC_Line_Measurement.V_BC_Line_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.V_BC_Line * 10.0);
	AC_Line_Measurement.V_CA_Line_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.V_CA_Line * 10.0);

	//****** CAP 1 filtering ******
	if (AC_Line_Measurement.V_CA_Line_buff[AC_Line_Measurement.Line_buff_index] > V_CA_old) {
		AC_Line_Measurement.V_CA_rising = TRUE;
		GpioDataRegs.GPASET.bit.GPIO30 = 1; 		// Profile AC falling
		V_CA_falling_cnt = 0;
		V_CA_rising_cnt ++ ;
		if (V_CA_rising_cnt >= 5) {
	    	if (eCAP1_Data.Capture == FALSE) {
	    		InitECap1Gpio();
	    		GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;		// Profile isr
			}
		}
	} else {
		AC_Line_Measurement.V_CA_rising = FALSE;
		GpioDataRegs.GPACLEAR.bit.GPIO30 = 1; 		// Profile AC rising
		V_CA_rising_cnt = 0;
		V_CA_falling_cnt ++ ;
		if (V_CA_falling_cnt >= 5){
			eCAP1_Data.Capture = FALSE;
		}
	}
	V_CA_old = AC_Line_Measurement.V_CA_Line_buff[AC_Line_Measurement.Line_buff_index];
}

/**********************************************************************
// Function: ADC3_read()
// Description: get ADC result from ADC3 (U7).
// Revisions:
// ----------
**********************************************************************/
void ADC3_read() {
//	while(GpioDataRegs.GPADAT.bit.GPIO27==1){	// Read after busy
//		asm(" NOP");
//    }
	ADC3.ADC_ch0 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHA0+ (B0)
	ADC3.ADC_ch3 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHB0+ (B3)
	ADC3.ADC_ch1 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHA1+ (B1)
	ADC3.ADC_ch4 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHB1+ (B4)
	ADC3.ADC_ch2 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHA1- (B2)
	ADC3.ADC_ch5 = (float) AC_CH_FACTOR * (int)(*pADC_3<<4) + 2.5;				// Read CHB1- (B6)

	DC_BUS_1 = ADC3.ADC_ch2 * DC_BUS_1_SCALE_FACTOR;

}


/**********************************************************************
// Function: ADC4_read()
// Description: get ADC result from ADC4 (U8).
// Revisions:
// ----------
**********************************************************************/
void ADC4_read() {
float fTemp0, fTemp1, fTemp2;
int  I_gen_A, I_gen_B, I_gen_C;
//	while(GpioDataRegs.GPADAT.bit.GPIO27==1){	// Read after busy
//		asm(" NOP");
//    }
	ADC4.ADC_ch0 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHA0+ (B0)
	ADC4.ADC_ch3 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHB0+ (B3)
	ADC4.ADC_ch1 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHA1+ (B1)
	ADC4.ADC_ch4 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHB1+ (B4)
	ADC4.ADC_ch2 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHA1- (B2)
	ADC4.ADC_ch5 = (float) AC_CH_FACTOR * (int)(*pADC_4<<4);				// Read CHB1- (B6)

	// AC line current from P49, P55, P57
	AC_Line_Measurement.I_A	= ADC4.ADC_ch0 * ACLINE_CURRENT_SCALE_FACTOR - AC_Line_A_Offset;
	AC_Line_Measurement.I_B	= ADC4.ADC_ch1 * ACLINE_CURRENT_SCALE_FACTOR - AC_Line_B_Offset;
	AC_Line_Measurement.I_C	= ADC4.ADC_ch2 * ACLINE_CURRENT_SCALE_FACTOR - AC_Line_C_Offset;

	AC_Line_A_Zero = 0.01 * AC_Line_Measurement.I_A + 0.99 * AC_Line_A_Zero;
	AC_Line_B_Zero = 0.01 * AC_Line_Measurement.I_B + 0.99 * AC_Line_B_Zero;
	AC_Line_C_Zero = 0.01 * AC_Line_Measurement.I_C + 0.99 * AC_Line_C_Zero;

	AC_Line_Measurement.I_A	*= Calibration.I_a;
	AC_Line_Measurement.I_B	*= Calibration.I_b;
	AC_Line_Measurement.I_C	*= Calibration.I_c;

	// Add selective filter for all 3 channels
	// u(n)=K1*[i(n)-i(n-2)] + K2*u(n-1) + K3*u(n-2)
	// phase A current:
	fTemp0 = I_Line_SF_K1 * (AC_Line_Measurement.I_A - ACline_Ia_SFilter.input_n2);
	fTemp1 = I_Line_SF_K2 * ACline_Ia_SFilter.output_n1;
	fTemp2 = I_Line_SF_K3 * ACline_Ia_SFilter.output_n2;
	ACline_Ia_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Ia_SFilter.input_n2 = ACline_Ia_SFilter.input_n1;
	ACline_Ia_SFilter.input_n1 = AC_Line_Measurement.I_A;
	ACline_Ia_SFilter.output_n2 = ACline_Ia_SFilter.output_n1;
	ACline_Ia_SFilter.output_n1 = ACline_Ia_SFilter.output;

	// phase B current:
	fTemp0 = I_Line_SF_K1 * (AC_Line_Measurement.I_B - ACline_Ib_SFilter.input_n2);
	fTemp1 = I_Line_SF_K2 * ACline_Ib_SFilter.output_n1;
	fTemp2 = I_Line_SF_K3 * ACline_Ib_SFilter.output_n2;
	ACline_Ib_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Ib_SFilter.input_n2 = ACline_Ib_SFilter.input_n1;
	ACline_Ib_SFilter.input_n1 = AC_Line_Measurement.I_B;
	ACline_Ib_SFilter.output_n2 = ACline_Ib_SFilter.output_n1;
	ACline_Ib_SFilter.output_n1 = ACline_Ib_SFilter.output;

	// phase C current:
	fTemp0 = I_Line_SF_K1 * (AC_Line_Measurement.I_C - ACline_Ic_SFilter.input_n2);
	fTemp1 = I_Line_SF_K2 * ACline_Ic_SFilter.output_n1;
	fTemp2 = I_Line_SF_K3 * ACline_Ic_SFilter.output_n2;
	ACline_Ic_SFilter.output = fTemp0 + fTemp1 + fTemp2;

	ACline_Ic_SFilter.input_n2 = ACline_Ic_SFilter.input_n1;
	ACline_Ic_SFilter.input_n1 = AC_Line_Measurement.I_C;
	ACline_Ic_SFilter.output_n2 = ACline_Ic_SFilter.output_n1;
	ACline_Ic_SFilter.output_n1 = ACline_Ic_SFilter.output;
	
	//if (system.state == MICRO_GRID_MODE) {
	if (1) {
		// AC line current D value
		AC_Line_Measurement.I_Line_D = ACline_Ia_SFilter.output * sine_ref.sin_0_a 
									 + ACline_Ib_SFilter.output * sine_ref.sin_N120_b 
									 + ACline_Ic_SFilter.output * sine_ref.sin_P120_c;
		AC_Line_Measurement.I_Line_D *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// AC line current Q value
		AC_Line_Measurement.I_Line_Q = ACline_Ia_SFilter.output * sine_ref.cos_0_a 
									 + ACline_Ib_SFilter.output * sine_ref.cos_N120_b
									 + ACline_Ic_SFilter.output * sine_ref.cos_P120_c;
		AC_Line_Measurement.I_Line_Q *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// AC line current Zero value
		AC_Line_Measurement.I_Line_Zero = ACline_Ia_SFilter.output + ACline_Ib_SFilter.output + ACline_Ic_SFilter.output;
		AC_Line_Measurement.I_Line_Zero *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// instant AC line current
		AC_Line_Measurement.I_Line = sqrt(AC_Line_Measurement.I_Line_D * AC_Line_Measurement.I_Line_D 
									+ AC_Line_Measurement.I_Line_Q * AC_Line_Measurement.I_Line_Q);
	} else {
		// AC line current D value
		AC_Line_Measurement.I_Line_D = AC_Line_Measurement.I_A * sine_ref.sin_0_a 
									 + AC_Line_Measurement.I_B * sine_ref.sin_N120_b 
									 + AC_Line_Measurement.I_C * sine_ref.sin_P120_c;
		AC_Line_Measurement.I_Line_D *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// AC line current Q value
		AC_Line_Measurement.I_Line_Q = AC_Line_Measurement.I_A * sine_ref.cos_0_a 
									 + AC_Line_Measurement.I_B * sine_ref.cos_N120_b
									 + AC_Line_Measurement.I_C * sine_ref.cos_P120_c;
		AC_Line_Measurement.I_Line_Q *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// AC line current Zero value
		AC_Line_Measurement.I_Line_Zero = AC_Line_Measurement.I_A + AC_Line_Measurement.I_B + AC_Line_Measurement.I_C;
		AC_Line_Measurement.I_Line_Zero *= AC_CURRENT_DQ_CONVERT_FACTOR;

		// instant AC line current
		AC_Line_Measurement.I_Line = sqrt(AC_Line_Measurement.I_Line_D * AC_Line_Measurement.I_Line_D 
									+ AC_Line_Measurement.I_Line_Q * AC_Line_Measurement.I_Line_Q);
	}
	// Generator current from p54
	Generator_IA[AC_Line_Measurement.Line_buff_index] = ADC4.ADC_ch3 * GEN_CURRENT_SCALE_FACTOR;
	Generator_IB[AC_Line_Measurement.Line_buff_index] = ADC4.ADC_ch4 * GEN_CURRENT_SCALE_FACTOR;
	Generator_IC[AC_Line_Measurement.Line_buff_index] = ADC4.ADC_ch5 * GEN_CURRENT_SCALE_FACTOR;
	I_gen_A = (int)Generator_IA[AC_Line_Measurement.Line_buff_index];
	I_gen_B = (int)Generator_IB[AC_Line_Measurement.Line_buff_index];
	I_gen_C = (int)Generator_IC[AC_Line_Measurement.Line_buff_index];

	if (I_gen_A > Generator_Current_Limit || I_gen_B > Generator_Current_Limit || I_gen_C > Generator_Current_Limit
		|| I_gen_A < -Generator_Current_Limit || I_gen_B < -Generator_Current_Limit || I_gen_C < -Generator_Current_Limit) {
		Generator_Current_Limit_cnt++;
		if (Generator_Current_Limit_cnt > (InputStageFilter + 1)) {
			system.SW_faults |= I_GEN_PEAK_OVERCURRENT;
		}
	} else {
		Generator_Current_Limit_cnt = 0;
	}


	// Fill data buffer
	AC_Line_Measurement.Line_buff_index ++;
	VAC_out_Measurement.Line_buff_index ++;
	//if (Phase_Lock != TRUE) {
	if (TRUE) {
		if(AC_Line_Measurement.Line_buff_index >= OperationData.Period_Cnt) {
			AC_Line_Measurement.Line_buff_index = 0;
			VAC_out_Measurement.Line_buff_index = 0;
		}
	}
	AC_Line_Measurement.I_A_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_A * 10.0);
	AC_Line_Measurement.I_B_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_B * 10.0);
	AC_Line_Measurement.I_C_buff[AC_Line_Measurement.Line_buff_index] = (int)(AC_Line_Measurement.I_C * 10.0);

}

/**********************************************************************
// Function: DAC_output()
// Description: Access to DAC7725.		Engine_Speed_RPM	
// Revisions:
// ----------
**********************************************************************/
void DAC_output() {
	switch(DAC_OutputCtrl) {
		case 0:
			//pDAC_Ptr->Data_4 = (unsigned int)(2*AC_Line_Measurement.I_A + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(2*AC_Line_Measurement.I_B + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(2*AC_Line_Measurement.I_C + 2047); 
			pDAC_Ptr->Data_4 = (unsigned int)Engine_Speed_RPM;
			pDAC_Ptr->Data_3 = (unsigned int)(1800*sine_ref.sin_P120_c + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1800*sine_ref.sin_N120_b + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		case 1:
			pDAC_Ptr->Data_4 = (unsigned int)(5*AC_Line_Measurement.V_AB_RMS);
			pDAC_Ptr->Data_3 = (unsigned int)(5*AC_Line_Measurement.V_BC_RMS);
			pDAC_Ptr->Data_2 = (unsigned int)(5*AC_Line_Measurement.V_CA_RMS);
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_5th_harmonic + 2047);
			break;

		case 2:
			pDAC_Ptr->Data_4 = (unsigned int)(10*InverterOut_Ia_SFilter.output + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*InverterOut_Ib_SFilter.output + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*InverterOut_Ic_SFilter.output + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(10*VAC_out_Measurement.I_A + 2047);
			break;

		case 3:
			pDAC_Ptr->Data_4 = (unsigned int)(10*ACline_Ia_SFilter.output + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*ACline_Ib_SFilter.output + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*ACline_Ic_SFilter.output + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(10*AC_Line_Measurement.I_A + 2047);
			break;

		case 4:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_in + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.Ib_in + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.Ic_in + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_in + 2047);
			//pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_in + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_out + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.I_D_Fil + 2047); 
			//pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.I_Q_Fil + 2047);
			break;

		case 5:
			pDAC_Ptr->Data_4 = (unsigned int)(10*AC_Line_Measurement.I_A + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_cmd_ref + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*AC_Line_Measurement.I_B + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.Ib_cmd_ref + 2047);
			//pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.I_D_err + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.I_Q_err + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.I_D_com + 2047); 
			//pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.I_Q_com + 2047);
			break;

		case 6:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.I_D + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.I_Q + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.I_D_Fil + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.I_Q_Fil + 2047);
			//pDAC_Ptr->Data_4 = (unsigned int)(10*AC_Line_Measurement.I_B + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(10*GT_5th_harm_ctrl.Ib_cmd_ref + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(10*AC_Line_Measurement.I_C + 2047); 
			//pDAC_Ptr->Data_1 = (unsigned int)(10*GT_5th_harm_ctrl.Ic_cmd_ref + 2047);
			break;

		case 7:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_in + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1800*GT_5th_harm_ctrl.Ia_out + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.Ic_in + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(1800*GT_5th_harm_ctrl.Ic_out + 2047); 
			//pDAC_Ptr->Data_4 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(1800*sine_ref.sin_N120_b + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(1800*sine_ref.sin_P120_c + 2047); 
			//pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.cos_0_a + 2047);
			break;

		case 8:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_out + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1800*GT_5th_harm_ctrl.Ib_out + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*GT_5th_harm_ctrl.Ib_in + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(1800*GT_5th_harm_ctrl.Ib_out + 2047); 
			//pDAC_Ptr->Data_4 = (unsigned int)(1800*sine_ref.cos_0_a + 2047);
			//pDAC_Ptr->Data_3 = (unsigned int)(1800*sine_ref.cos_N120_b + 2047);
			//pDAC_Ptr->Data_2 = (unsigned int)(1800*sine_ref.cos_P120_c + 2047); 
			//pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		case 9:
			pDAC_Ptr->Data_4 = (unsigned int)(100*Inverter_SysControl.GT_ID_out + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(100*Inverter_SysControl.GT_IQ_out + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(100*Inverter_SysControl.GT_IDout_error + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(100*Inverter_SysControl.GT_IQout_error + 2047);
			break;

		case 10:
			pDAC_Ptr->Data_4 = (unsigned int)(1800*iGrid_Tie.Ta + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1800*iGrid_Tie.Tb + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1800*iGrid_Tie.Tc + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		case 11:
			pDAC_Ptr->Data_4 = (unsigned int)(1000*svgen1.Ta + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1000*svgen1.Tb + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1000*svgen1.Tc + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		case 12:
			pDAC_Ptr->Data_4 = (unsigned int)(1000*svgen1.tmp1 + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1000*svgen1.tmp2 + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1000*svgen1.tmp3 + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		case 13:
			pDAC_Ptr->Data_4 = (unsigned int)(100*AC_Line_Measurement.I_Line_D);
			pDAC_Ptr->Data_3 = (unsigned int)(100*AC_Line_Measurement.I_Line_Q);
			pDAC_Ptr->Data_2 = (unsigned int)(1000*Inverter_SysControl.GT_ID_out + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(1000*Inverter_SysControl.GT_IQ_out + 2047);
			break;

		case 14:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_5th_harm_ctrl.Ia_in + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1800*GT_2nd_harm_ctrl.Ia_out + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1800*GT_2nd_harm_ctrl.Ib_out + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(1800*GT_2nd_harm_ctrl.Ic_out + 2047); 
			break;

		case 15:
			pDAC_Ptr->Data_4 = (unsigned int)(10*GT_2nd_harm_ctrl.I_D + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(10*GT_2nd_harm_ctrl.I_Q + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(10*GT_2nd_harm_ctrl.I_D_Fil + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(10*GT_2nd_harm_ctrl.I_Q_Fil + 2047);
			break;

		case 16:
			pDAC_Ptr->Data_4 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(1800*sine_ref.cos_0_a + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(1800*sine_ref.sin_2nd_harmonic + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.cos_2nd_harmonic + 2047);
			break;

		case 17:
			pDAC_Ptr->Data_4 = (unsigned int)(2*AC_Line_Measurement.V_AB_Line + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(2*Anti_Islanding.Vab + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(2*AC_Line_Measurement.V_BC_Line + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(2*Anti_Islanding.Vbc + 2047);
			break;

		case 18:
			pDAC_Ptr->Data_4 = (unsigned int)(2*AC_Line_Measurement.V_A + 2047);
			pDAC_Ptr->Data_3 = (unsigned int)(2*Anti_Islanding.Va + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(2*AC_Line_Measurement.V_B + 2047); 
			pDAC_Ptr->Data_1 = (unsigned int)(2*Anti_Islanding.Vb + 2047);
			break;

		case 19:
			pDAC_Ptr->Data_4 = (unsigned int)(2*AC_Line_Measurement.V_B + 2047); 
			pDAC_Ptr->Data_3 = (unsigned int)(1800*sine_ref.sin_N120_b + 2047);
			pDAC_Ptr->Data_2 = (unsigned int)(2*AC_Line_Measurement.V_A + 2047);
			pDAC_Ptr->Data_1 = (unsigned int)(1800*sine_ref.sin_0_a + 2047);
			break;

		default:	// unknown status
			// do nothing
			DAC_OutputCtrl = 0;
			break;
	}


}
/*** end of file *****************************************************/
