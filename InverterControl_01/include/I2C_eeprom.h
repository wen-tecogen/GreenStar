/**********************************************************************************
// File: I2C_eeprom.h
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
// This is the header file for I2C eeprom.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 09/12/12	|  J Wen 	| Original
**********************************************************************************/

#ifndef I2C_EEPROM_H_
#define I2C_EEPROM_H_

// EEPROM constants define
#define I2C_SLAVE_ADDR        0x50
#define I2C_NUMBYTES          8

// EEPROM address define
#define EEPROM_ADDRESS_HIGH_GROUP_0        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_0        	0x00

#define EEPROM_ADDRESS_HIGH_GROUP_1        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_1        	0x08

#define EEPROM_ADDRESS_HIGH_GROUP_2        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_2        	0x10

#define EEPROM_ADDRESS_HIGH_GROUP_3        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_3        	0x18

#define EEPROM_ADDRESS_HIGH_GROUP_4        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_4        	0x20

#define EEPROM_ADDRESS_HIGH_GROUP_5        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_5        	0x28

#define EEPROM_ADDRESS_HIGH_GROUP_6        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_6        	0x30

#define EEPROM_ADDRESS_HIGH_GROUP_7        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_7        	0x38

#define EEPROM_ADDRESS_HIGH_GROUP_8        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_8        	0x40

#define EEPROM_ADDRESS_HIGH_GROUP_9        	0x00
#define EEPROM_ADDRESS_LOW_GROUP_9        	0x48

// EEPROM data structure
typedef struct {
   Uint16 date;
   Uint16 month;
   Uint16 year;
}EEPROM_GROUP_0;

typedef struct {
   Uint16 Operation_Frequency;
   Uint16 PWM_Frequency;
   Uint16 spare;
}EEPROM_GROUP_1;

typedef struct {
   Uint16 DCBUS_Setting;
   Uint16 DC_OV_Setting;
   Uint16 BRK_Setting;
}EEPROM_GROUP_2;

typedef struct {
   Uint16 AI_sensitivity;
   Uint16 AI_trip_time;
   Uint16 AI_Q_inject;
}EEPROM_GROUP_3;

typedef struct {
   Uint16 V_ab_Cal_x10k;
   Uint16 V_bc_Cal_x10k;
   Uint16 V_ca_Cal_x10k;
}EEPROM_GROUP_4;

typedef struct {
   Uint16 I_a_Cal_x10k;
   Uint16 I_b_Cal_x10k;
   Uint16 I_c_Cal_x10k;
}EEPROM_GROUP_5;

typedef struct {
   Uint16 Boost_PWM_Frequency;
   Uint16 spare_1;
   Uint16 spare_2;
}EEPROM_GROUP_6;

typedef struct {
	EEPROM_GROUP_0 Group_0;
	EEPROM_GROUP_1 Group_1;
	EEPROM_GROUP_2 Group_2;
	EEPROM_GROUP_3 Group_3;
	EEPROM_GROUP_4 Group_4;
	EEPROM_GROUP_5 Group_5;
	EEPROM_GROUP_6 Group_6;
}EEPROM_DATA;

#endif /* I2C_EEPROM_H_ */
