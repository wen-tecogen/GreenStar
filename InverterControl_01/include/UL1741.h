/**********************************************************************************
// File: UL1741.h
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
// 12/12/13	|  J Wen 	| Original
**********************************************************************************/
#ifndef UL1741_H_
#define UL1741_H_

//---------------------------------------------------------------------------------
// UL1741 data structure
// note: voltage setting: V/bit		frequency setting: 0.1Hz/bit
typedef struct {
   Uint16 V_Upper_Limit_1;
   Uint16 V_Upper_Limit_1_Trip_Time;
   Uint16 V_Upper_Limit_2;
   Uint16 V_Upper_Limit_2_Trip_Time;
   Uint16 V_Lower_Limit_1;
   Uint16 V_Lower_Limit_1_Trip_Time;
   Uint16 V_Lower_Limit_2;
   Uint16 V_Lower_Limit_2_Trip_Time;
   Uint16 F_Upper_Limit_1;
   Uint16 F_Upper_Limit_1_Trip_Time;
   Uint16 F_Upper_Limit_2;
   Uint16 F_Upper_Limit_2_Trip_Time;
   Uint16 F_Lower_Limit_1;
   Uint16 F_Lower_Limit_1_Trip_Time;
   Uint16 F_Lower_Limit_2;
   Uint16 F_Lower_Limit_2_Trip_Time;
   Uint16 GT_ReconnectTime;
   Uint16 Anti_Islanding_Sense;
}UL1741_SETTING;

// Anormal voltage and frequency trip time counter
typedef struct {
   Uint16 V_Upper_Limit1_cnt;
   Uint16 V_Upper_Limit2_cnt;
   Uint16 V_Lower_Limit1_cnt;
   Uint16 V_Lower_Limit2_cnt;
   Uint16 F_Upper_Limit1_cnt;
   Uint16 F_Upper_Limit2_cnt;
   Uint16 F_Lower_Limit1_cnt;
   Uint16 F_Lower_Limit2_cnt;
}TRIP_TIME_CNT;
//---------------------------------------------------------------------------------

// UL1741 Trip define
#define dF_UPPER_LIMIT_1_TRIP					0x0001
#define dF_LOWER_LIMIT_1_TRIP					0x0002
#define dF_UPPER_LIMIT_2_TRIP					0x0004
#define dF_LOWER_LIMIT_2_TRIP					0x0008
#define dV_UPPER_LIMIT_1_TRIP					0x0010
#define dV_LOWER_LIMIT_1_TRIP					0x0020
#define dV_UPPER_LIMIT_2_TRIP					0x0040
#define dV_LOWER_LIMIT_2_TRIP					0x0080
#define dANTI_ISLANDING_TRIP					0x0100

#endif /* UL1741_H_ */

