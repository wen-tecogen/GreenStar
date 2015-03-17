/**********************************************************************************
// File: eCAN_communication.h
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
// This is the header file for eCAN_communication.c.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/27/12	|  J Wen 	| Original
**********************************************************************************/

#ifndef ECAN_COMMUNICATION_H_
#define ECAN_COMMUNICATION_H_

typedef struct {
   Uint32 eCAN_ID;
   Uint16 Data_Length;
   Uint16 Data1;
   Uint16 Data2;
   Uint16 Data3;
   Uint16 Data4;
   Uint16 New_Message;
}ECAN_MESSAGE;

// CAN message IDs  -------------------------------------------------
// Note: CAN message ID bit 31 is Identifier extension bit 0x80000000
#define		eCANMsgID_CmdSysOperation				0x00001001
#define		eCANMsgID_CmdSysControlMode				0x00001002
#define		eCANMsgID_CmdSysParam_VDCBUS			0x00001003
#define		eCANMsgID_CmdSysParam_Vout				0x00001004
#define		eCANMsgID_CmdSysParam_Iout				0x00001007
#define		eCANMsgID_CmdSysParam_Phase				0x0000100A
#define		eCANMsgID_GridTieSpeedCtrl				0x00001010
#define		eCANMsgID_GridTieTestMode				0x000F1001
#define		eCANMsgID_GridTieTestData				0x000F1002
#define		eCANMsgID_Filter_Setting1				0x000F1011
#define		eCANMsgID_GTFifthHarmonicCtrl			0x000F1012

#define		eCANMsgID_SystemReset					0x00001301
#define		eCANMsgID_SystemDebugData1				0x00014004
#define		eCANMsgID_SystemDebugData2				0x00014005
#define		eCANMsgID_SystemDebugData3				0x00014006
#define		eCANMsgID_SystemDebugData4				0x00014007
#define		eCANMsgID_HarmonicCtrlData				0x00014008

#define		eCANMsgID_DigitalOutTest				0x00011001
#define		eCANMsgID_DataOutputCtrl				0x00011002
#define		eCANMsgID_BoostControlParameters		0x00011003
#define		eCANMsgID_VolageControlParameters		0x00011004
#define		eCANMsgID_PLLControlParameters			0x00011005
#define		eCANMsgID_GridTieControlParameters		0x00011006
#define		eCANMsgID_EngineSpeedControlParameters	0x00011007

// Echo -------------------------------------------------------------
#define		eCANMsgID_CmdSysOperation_Echo			0x00003001
#define		eCANMsgID_CmdSysControlMode_Echo		0x00003002
#define		eCANMsgID_CmdSysParam_VDCBUS_Echo		0x00003003
#define		eCANMsgID_CmdSysParam_Vout_Echo			0x00003004
#define		eCANMsgID_CmdSysParam_Iout_Echo			0x00003007
#define		eCANMsgID_CmdSysParam_Phase_Echo		0x0000300A
#define		eCANMsgID_GridTieSpeedCtrl_Echo			0x00003010
#define		eCANMsgID_GridTieTestMode_Echo			0x000F3001
#define		eCANMsgID_GridTieTestData_Echo			0x000F3002
#define		eCANMsgID_Filter_Setting1_Echo			0x000F3011
#define		eCANMsgID_GTFifthHarmonicCtrl_Echo		0x000F3012
#define		eCANMsgID_FlashTestData_Echo			0x000F30A1

#define		eCANMsgID_DACOutputChannel_Echo			0x00013002
#define		eCANMsgID_BoostControlParameters_Echo	0x00013003
#define		eCANMsgID_VolageControlParameters_Echo	0x00013004
#define		eCANMsgID_PLLControlParameters_Echo		0x00013005
#define		eCANMsgID_GridTieControlParameters_Echo	0x00013006
#define		eCANMsgID_EngineSpeedControlParameters_Echo	0x00013007

// Measurement data report
#define		eCANMsgID_StateStatusFault				0x00002001
#define		Software_Faults_Warnings				0x00002002
#define		DataMeasurementMsgID_Digital_IO			0x00004001
#define		DataMeasurementMsgID_DCBusData			0x00004004
#define		DataMeasurementMsgID_ACLineVrms			0x00004005
#define		DataMeasurementMsgID_ACLineIrms			0x00004006
#define		DataMeasurementMsgID_ACPowerOut			0x00004007
#define		DataMeasurementMsgID_I_GEN_rms			0x00004008
#define		DataMeasurementMsgID_ACLineData1		0x00004009
#define		DataMeasurementMsgID_ACLineData2		0x0000400A
#define		DataMeasurementMsgID_ACLineData3		0x0000400B
#define		DataMeasurementMsgID_InverterOutData1	0x0000400C
#define		DataMeasurementMsgID_InverterOutData2	0x0000400D
#define		DataMeasurementMsgID_InverterOutData3	0x0000400E
#define		DataMeasurementMsgID_InverterOutData4	0x0000400F
#define		DataMeasurementMsgID_InverterPower		0x00004010
#define		DataMeasurementMsgID_MicroGridData1		0x00004011
#define		DataMeasurementMsgID_MicroGridData2		0x00004012

// Software date code, version
#define 	eCANMsgID_SoftwareDateCode				0x00045001
#define 	eCANMsgID_FpgaDateCode					0x00045002

// UL 1741 setting
#define 	eCANMsgID_UL1741_Setting				0x00100000
#define 	eCANMsgID_OverVoltage_Setting			0x00100001
#define 	eCANMsgID_UnderVoltage_Setting			0x00100002
#define 	eCANMsgID_OverFrequency_Setting			0x00100003
#define 	eCANMsgID_UnderFrequency_Setting		0x00100004
#define 	eCANMsgID_ReConnTime_Setting			0x00100005
#define 	eCANMsgID_AntiIslanding_Setting			0x00100006

#define 	eCANMsgID_OverVoltage_Setting_Echo		0x00300001
#define 	eCANMsgID_UnderVoltage_Setting_Echo		0x00300002
#define 	eCANMsgID_OverFrequency_Setting_Echo	0x00300003
#define 	eCANMsgID_UnderFrequency_Setting_Echo	0x00300004
#define 	eCANMsgID_ReConnTime_Setting_Echo		0x00300005
#define 	eCANMsgID_AntiIslanding_Setting_Echo	0x00300006

// Micro-Grid setting
#define 	eCANMsgID_MicroGrid_Setting				0x00200000
#define 	eCANMsgID_FrequencyLimit_Setting		0x00200001
#define 	eCANMsgID_Power_Setting					0x00200002
#define 	eCANMsgID_P_vs_Freq_Setting				0x00200003
#define 	eCANMsgID_Q_vs_Vout_Setting				0x00200004

#define 	eCANMsgID_FrequencyLimit_Setting_Echo	0x00500001
#define 	eCANMsgID_Power_Setting_Echo		 	0x00500002
#define 	eCANMsgID_P_vs_Freq_Setting_Echo	 	0x00500003
#define 	eCANMsgID_Q_vs_Vout_Setting_Echo	 	0x00500004

// EEPROM read/write
#define 	eCANMsgID_ReadEEPROM					0x00070001
#define 	eCANMsgID_ReadEEPROMEcho				0x00073001
#define 	eCANMsgID_WriteEEPROM					0x00070002
#define 	eCANMsgID_WriteEEPROMEcho				0x00073002

#define 	eCANMsgID_EEPROM_Group_0				0x00071001
#define 	eCANMsgID_EEPROM_Group_1				0x00071002
#define 	eCANMsgID_EEPROM_Group_2				0x00071003
#define 	eCANMsgID_EEPROM_Group_3				0x00071004
#define 	eCANMsgID_EEPROM_Group_4				0x00071005
#define 	eCANMsgID_EEPROM_Group_5				0x00071006
#define 	eCANMsgID_EEPROM_Group_6				0x00071007
#define 	eCANMsgID_EEPROM_Group_7				0x00071008
#define 	eCANMsgID_EEPROM_Group_8				0x00071009
#define 	eCANMsgID_EEPROM_Group_9				0x00071010

// Flash program message
#define		FlashCANMsgID_Data						0x0E000000
#define		FlashCANMsgID_EraseCmd					0x0A000001
#define		FlashCANMsgID_ProgramCmd				0x0A000002
#define		FlashCANMsgID_VerifyCmd					0x0A000003
#define		FlashCANMsgID_FlashFinish				0x0A000004
#define		FlashCANMsgID_ProgramStatus				0x000A0001
#define		FlashCANMsgID_FlashPageComplete			0x000A0002

/************************************************************
#define	DataMeasurementMsgID_IoutVout_DQ		    0x00044001
#define	DataMeasurementMsgID_PQout				    0x00044002
#define	DataMeasurementMsgID_E0_f_cmd			    0x00044003
#define	DataMeasurementMsgID_DataReport_4		    0x00044004
#define	DataMeasurementMsgID_VlineF_Pcmd		    0x00044005
#define	DataMeasurementMsgID_VlineVrms			    0x00044006
#define	DataMeasurementMsgID_Input_Irms			    0x00044007
************************************************************/

//==================================================================================

#endif /* ECAN_COMMUNICATION_H_ */
