/**********************************************************************************
// File: SystemControl.c
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
// This module contains:
//       state machine, CAN bus message process, Data report
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/27/12	|  J Wen 	| Original
**********************************************************************************/

#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>

#include "DSP28x_Project.h"     // Device Header file
#include "SystemControl.h"      // Header file for system control
#include "EPWM_Control.h"       // Header file for ePWM
#include "eCAN_communication.h" // Header file for eCAN
#include "ECAP_VLine.h"     	// Header file for eCAP
#include "I2C_eeprom.h"			// EEPROM header file
#include "Fpga.h"
#include "Digital_IO.h"
#include "ADC.h"				// Header file for A to D converter
#include "Grid_Control.h"		// Header file for grid control
#include "UL1741.h"				// Header file for UL1741
#include "DataMeasurement.h"       // Header file for ePWM module

//#include "SPIFLASH28_SPI_Flashprog_Library.h"


/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle systemSema;
extern const Semaphore_Handle commSema;

//====== Global variables used in this module ======
Uint16			StateMachineCount;
Uint16			StateTransferCount;
Uint16			CommunicationCount;

SYSTEM_INFO 	system;
Uint16			CRC_checkSum;
EEPROM_DATA		EEPROM_Data;
Uint16			EERPOM_report;
int				EERPOM_AC_Frequency;

Uint16			FlashMemory_buffer[128];
Uint32			Buufer_Index;
Uint16			PRG_Page_status;
Uint16			ProgramPageActive;
Uint16			ProgramEraseActive;
Uint16			Harmonic_TuneCtrl;
Uint16			DebugData_OutputCtrl;
Uint16			UL_1741_MG_Init;
Uint16			Brake_Chopper_On;
Uint16			CAN_Bus_Reset;
float			report_Engine_Speed_RPM;

int 	intCAN_Data;
int		hart_beat;

int		iGT_ID_command;
Uint16	GT_SpeedCtrl_Cnt;
int	 	InputStageFilter;

float	V_ACLine_D, V_ACLine_Q, V_ACLine_Z, V_ACLine;  
float	I_ACLine_D, I_ACLine_Q, I_ACLine_Z, I_ACLine;  

FLASH_INFO	flash_Info;
//---------------------------------------------------

static Uint16	Digitalout_Image;

static Uint16	InverterTest_timer;
static Uint16	InverterDischarge_timer;

extern Uint32 CAN_Data1, CAN_Data2, CAN_DataID;
extern EPWM_INFO 		epwm1_info;
extern EPWM_INFO 		epwm2_info;
extern EPWM_INFO 		epwm3_info;
extern OPERATION_DATA 	OperationData;
extern ECAN_MESSAGE 	eCAN_TxMessage, eCAN_RxMessage;
extern AC_LINE_DATA		ACLine;
extern unsigned int FPGA_r_data[16];

extern PLL_DATA		PLL_Data;
extern struct I2CMSG I2cMsgOut1;
extern struct I2CMSG I2cMsgIn1;
extern OPERATION_DATA 	OperationData;
extern BOOST_CONTROL	Boost_Control;
extern INVERTER_CONTROL Inverter_SysControl;
extern FPGA_REGWR_IMAGE FPGA_RegWR_Image;
extern union DIGITAL_IO Digital_IO;
extern AC_MEASURE_DATA 	AC_Line_Measurement;		// AC line data
extern AC_MEASURE_DATA 	VAC_out_Measurement;		// Inverter Output data
extern GT_CURRENT_HARMONICS	GT_5th_harm_ctrl;
extern GT_CURRENT_HARMONICS	GT_2nd_harm_ctrl;
extern ANTI_ISLANDING	Anti_Islanding;

extern GRID_TIE	iGrid_Tie;
extern UL1741_SETTING	UL1741_Setting;
extern MICRO_GRID	MicroGrid;
extern Uint16 GT_CtrlLoop_TIME;
extern TRIP_TIME_CNT	VF_Trip;

extern CALIBRATION		Calibration;
extern SPEED_CONTROL speed_control;

extern float	DC_BUS_Voltage;
extern float	Boost_Current;
extern float	T_ambient_C;
extern float	I_Boost_report;
extern int		DC_bus_OV_setting;
extern int		Brake_ON_V_setting;
extern float	Frequency_fpga_ch[3];
extern float	Engine_Speed_RPM;

//extern int	 V_A_at_cap1isr;
//extern int	 V_B_at_cap1isr;
//extern int	 V_C_at_cap1isr;

extern Uint16	Flash_Erase_Status;
extern Uint16	PRG_status;
extern Uint16  	Buffer[];
extern Uint16	DAC_OutputCtrl;

extern float	Generator_IA_RMS;
extern float	Generator_IB_RMS;
extern float	Generator_IC_RMS;

extern float	Output_KW;
extern float	Output_KVA;

extern float	VAC_Line_QI, VAC_Line_QI_Old;
extern float	VAC_Line_QF, VAC_Line_QF_Old;
extern float	I_Line_SF_K1, I_Line_SF_K2, I_Line_SF_K3;

extern int		Boost_Currentx10;
extern int      Generator_Current_Limit;

// AC Line current offset and adjustment
int 	AC_Line_Current_sensing_zero;
extern float	AC_Line_A_Offset;
extern float	AC_Line_B_Offset;
extern float	AC_Line_C_Offset;
extern float	AC_Line_A_Zero;
extern float	AC_Line_B_Zero;
extern float	AC_Line_C_Zero;

//====== Function prototype ======
void Msg_Processor(void);
void Flash_Msg_Processor(void);
void SystemSetting_Report(void);
void Data_Report(void);
void EEPROM_Report(void);
void SWData_Report(void);
void Boost_On(void);
void Boost_Off(void);
void Boost2_On(void);
void Boost2_Off(void);
void Brake_On(void);
void Brake_Off(void);
void Inverter_On(void);
void Inverter_Off(void);
void BoostCommand(void);
void Sys_Var_Init(void);

extern void eCAN_DataTx(ECAN_MESSAGE *eCAN_msg);
extern void eCAN_TxSetup(void);
extern void eCAN_RxSetup(void);
extern void EPWM1_On(void);
extern void EPWM1_Off(void);
extern void EPWM_Boost_On(void);
extern void EPWM_Boost_Off(void);
extern void EPWM_Boost2_On(void);
extern void EPWM_Boost2_Off(void);
extern void EPWM_Brake_On(void);
extern void EPWM_Brake_Off(void);
extern void Load_eeprom(void);
extern Uint16 I2CA_WriteData(struct I2CMSG *msg);
extern Uint16 crc16(Uint16 *message, Uint16 length);
extern void FPGA_Write(void);
extern void FPGA_Read(void);
extern void spiA_xmit(Uint16 a);
extern void SPI_A_test(void);

extern void ADC_vars_init(void);
extern void ECAP_Vars_init(void);
extern void Data_Measure_Init(void);
	   
extern void	UL1741_Setting_Init(void);
extern void UL1741_Setting_CAN(void);
extern void	UL1741_Setting_Init(void);
extern void MicroGrid_init(void);
extern void MicroGrid_Setting_CAN(void);
extern void GT_Anti_Islanding_init(void);

extern void Digital_IO_Init(void);
extern void Digital_Input(void);
extern void DO_1_On(void);
extern void DO_2_On(void);
extern void DO_3_On(void);
extern void DO_4_On(void);
extern void DO_5_On(void);
extern void DO_6_On(void);
extern void DO_7_On(void);
extern void DO_1_Off(void);
extern void DO_2_Off(void);
extern void DO_3_Off(void);
extern void DO_4_Off(void);
extern void DO_5_Off(void);
extern void DO_6_Off(void);
extern void DO_7_Off(void);

//---- Flash programmer ----
extern Uint16 SPIFLASH_Init(void);
extern void PRG_program(void);    // Program a block
extern void PRG_erase(void);      // Erase sectors
extern void PRG_verify(void);     // Verify a block
extern void FlashMemoryInit(void);

//
//  ======== StateMachine ========
//  System control task
//
Void StateMachine(Void) {
float	fTemp;
Uint16	uiTemp;

	eCAN_TxSetup();
	eCAN_RxSetup();

	/****** All PWM channels off ******/
	Boost_Off();
	Inverter_Off();
	system.status = 0;
	system.state = SYS_INIT;
	system.TestMode = FALSE;
	system.setting = 0;
	system.InverterTest = TRUE;
	UL_1741_MG_Init = TRUE;
	AC_Line_Current_sensing_zero = TRUE;
	InputStageFilter = 10;
	Sys_Var_Init();
	Digital_IO_Init();
	//UL1741_Setting_Init();
	//SPI_ADC_channel = 0x0000;	// channel 0

	// System power on set to micro-grid mode (voltage source)
 	Inverter_SysControl.SystemOperationMode = MICRO_GRID_MODE;
	system.setting &= ~OP_MODE_MG_0_GT_1;

	// Flash programming init
	Flash_Erase_Status = 99;
	StateMachineCount = 0;
	flash_Info.crc_CheckSUM = 0xFFFF;
    //
    // Do this forever
    //

    while (TRUE) {
        /*
         * Pend on "systemSema" until the timer ISR says
         * its time to do something.
         */
        Semaphore_pend(systemSema, BIOS_WAIT_FOREVER);

        if (eCAN_RxMessage.New_Message) {
			if (system.FlashEnable) {
        		Flash_Msg_Processor();
			} else {
        		Msg_Processor();
			}
        	eCAN_RxMessage.New_Message = FALSE;
        } else {
        	//GpioDataRegs.GPBTOGGLE.bit.GPIO62 = 1; // GPIO62: DO_2 Active Low
        	if (hart_beat == 0) {
        		FPGA_RegWR_Image.Data_1 |= DSP_HART_BEAT;
        		hart_beat = 1;
        	} else {
        		FPGA_RegWR_Image.Data_1  &= ~DSP_HART_BEAT;
        		hart_beat = 0;
        	}
    		pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
        	// ========== CAN ==========
			if (!system.FlashEnable) {	 // if the processor is in programming state, stop send CAN message
        		eCAN_TxMessage.eCAN_ID = eCANMsgID_StateStatusFault;
        		eCAN_TxMessage.Data_Length = 8;
        		eCAN_TxMessage.Data1 = system.state;
        		eCAN_TxMessage.Data2 = system.status;
        		eCAN_TxMessage.Data3 = system.faults;
        		eCAN_TxMessage.Data4 = system.setting;
        		eCAN_DataTx(&eCAN_TxMessage);
			} else {
				// send flash memory message back to host
	        	eCAN_TxMessage.eCAN_ID = FlashCANMsgID_ProgramStatus;
	        	eCAN_TxMessage.Data_Length = 8;
	        	eCAN_TxMessage.Data1 = Flash_Erase_Status;
	        	eCAN_TxMessage.Data2 = PRG_status;
	        	eCAN_TxMessage.Data3 = PRG_Page_status;
	        	eCAN_TxMessage.Data4 = 1;
	        	eCAN_DataTx(&eCAN_TxMessage);
			}
        	// ==========================
			// Digital inputs:
			Digital_Input();
        	// FPGA read
        	FPGA_Read();

        	// ==========================
			// Fault signals from FPGA:	 
		    system.faults = pFPGA_RegRD->Data_1 & 0xCFFF;
			if (!Digital_IO.bit.DI2 && !system.TestMode) {
				system.faults |= HEATSINK_OVER_TEMP;
			}
			if (!Digital_IO.bit.DI4 && !system.TestMode) {
				system.faults |= BRAKE_OVER_TEMP;
			}
			if (iGrid_Tie.Trip != 0) {
				system.SW_faults |= UL_1741_TRIP;
			}
			if (system.SW_faults != 0 && !system.TestMode) {
				system.faults |= SW_FAULT_TRUE;
			}

        	// ====== For Test ==========
        	// Turn on/off 2nd Boost
    		if (system.command == SysCtrlCmd_2ndBoostOn && !system.EMS) {
        		FPGA_RegWR_Image.Data_2 |= PWM_POWER_CTRL;					// Enable PWM power
    			pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
				Boost2_On();
    			system.command = SysCtrlCmd_DoNothing;
    		}
    		if (system.command == SysCtrlCmd_2ndBoostOff) {
				Boost2_Off();
    			system.command = SysCtrlCmd_DoNothing;
    		}
        	// Turn on/off Brake   for test only if DC bus is below 50V
    		if (system.command == SysCtrlCmd_BrakeOn && !system.EMS && system.TestMode && ((int)DC_BUS_Voltage < 50)) {
				Brake_On();
    			system.command = SysCtrlCmd_DoNothing;
    		}
    		if (system.command == SysCtrlCmd_BrakeOff) {
				Brake_Off();
    			system.command = SysCtrlCmd_DoNothing;
    		}

        	// ==========================	
			// Host controller watch-dog.
        	// ==========================
        	if (system.state != FLASH_PROGRAMMING) {	
				system.HostActiveCnt ++ ;
				if (system.HostActiveCnt == 20 || system.HostActiveCnt >= 30) {
					//**** Reset CAN bus ****
					//if (CAN_Bus_Reset == FALSE){
						InitECanb();
						eCAN_TxSetup();
						eCAN_RxSetup();
						CAN_Bus_Reset = TRUE;
					//}
				}
				if (system.HostActiveCnt >= 120) {			// if lost communication from host controller for 60sec, shutdown.
					system.status &= ~HOST_COMMUNICATION;				// NO COMMUNICATION TO THE HOST CONTROLLER
					system.HostActiveCnt = 120;
			
					if (system.TestMode == FALSE) {
						DO_2_Off();
						system.SW_faults |= HOST_COMM_FAULT;
					}
				} else {
					DO_2_On();
					CAN_Bus_Reset = FALSE;
					system.status |= HOST_COMMUNICATION;				// HOST COMMUNICATION OK
				}
			}
        	// ==========================	


// ================= State Machine ======================
    		switch (system.state) {
    			/*** Power On State ***/
    			case	SYS_INIT:
    				StateMachineCount ++ ;
    				//FPGA_RegWR_Image.Data_1 |= FAULT_RESET;		// Reset fault register
            		//pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
    				//FPGA_RegWR_Image.Data_2 &= ~PWM_POWER_CTRL;	// Disable PWM power
    				//pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;

					//-------------------------------------------------------------
    				// Load EEPORM data
    				if (StateMachineCount == 1) {
        				FPGA_Write();			// Get ready for FPGA	   
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_0;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_0;
        				Load_eeprom();
        				EEPROM_Data.Group_0.date = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_0.month = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_0.year = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						for (uiTemp = 0; uiTemp <= 15; uiTemp ++) {
							FPGA_r_data[uiTemp] = 0;
						}
    				}
    				if (StateMachineCount == 2) {
    					// Load EEPROM setting: operation frequency, PWM frequency....
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_2;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_2;
        				Load_eeprom();
        				EEPROM_Data.Group_2.DCBUS_Setting = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_2.DC_OV_Setting = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_2.BRK_Setting = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						if (EEPROM_Data.Group_2.DCBUS_Setting <= 1000) {
							Boost_Control.DCBUS_cmd = (float)EEPROM_Data.Group_2.DCBUS_Setting;
							DC_bus_OV_setting = EEPROM_Data.Group_2.DC_OV_Setting;
							Brake_ON_V_setting = EEPROM_Data.Group_2.BRK_Setting;
						}
						// Anti-islanding setting
						DELAY_US(10000L);							// delay 10msec
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_3;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_3;
        				Load_eeprom();
        				EEPROM_Data.Group_3.AI_sensitivity = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_3.AI_trip_time = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_3.AI_Q_inject = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						if (EEPROM_Data.Group_3.AI_Q_inject <= 20) {
		 					Anti_Islanding.sensitivity = EEPROM_Data.Group_3.AI_sensitivity; 		
		 					Anti_Islanding.trip_time = EEPROM_Data.Group_3.AI_trip_time;	 		
		 					iGrid_Tie.Q_inject_Setting = 0.01 * (float)EEPROM_Data.Group_3.AI_Q_inject; 		
						}
						// V line calibration data
						DELAY_US(10000L);							// delay 10msec
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_4;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_4;
        				Load_eeprom();
        				EEPROM_Data.Group_4.V_ab_Cal_x10k = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_4.V_bc_Cal_x10k = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_4.V_ca_Cal_x10k = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						if (EEPROM_Data.Group_4.V_ab_Cal_x10k >= 9000 && EEPROM_Data.Group_4.V_ab_Cal_x10k <= 11000) {
							Calibration.V_ab = 0.0001 * (float)EEPROM_Data.Group_4.V_ab_Cal_x10k;
							Calibration.V_bc = 0.0001 * (float)EEPROM_Data.Group_4.V_bc_Cal_x10k;
							Calibration.V_ca = 0.0001 * (float)EEPROM_Data.Group_4.V_ca_Cal_x10k;
						} else {
							Calibration.V_ab = 1.0;
							Calibration.V_bc = 1.0;
							Calibration.V_ca = 1.0;
						}
						// I line calibration data
						DELAY_US(10000L);							// delay 10msec
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_5;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_5;
        				Load_eeprom();
        				EEPROM_Data.Group_5.I_a_Cal_x10k = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_5.I_b_Cal_x10k = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_5.I_c_Cal_x10k = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						if (EEPROM_Data.Group_5.I_a_Cal_x10k >= 9000 && EEPROM_Data.Group_5.I_a_Cal_x10k <= 11000) {
							Calibration.I_a = 0.0001 * (float)EEPROM_Data.Group_5.I_a_Cal_x10k;
							Calibration.I_b = 0.0001 * (float)EEPROM_Data.Group_5.I_b_Cal_x10k;
							Calibration.I_c = 0.0001 * (float)EEPROM_Data.Group_5.I_c_Cal_x10k;
						} else {
							Calibration.I_a = 1.0;
							Calibration.I_b = 1.0;
							Calibration.I_c = 1.0;
						}
					}
    				if (StateMachineCount == 3) {
    					EERPOM_report = TRUE;
    					// Load EEPROM setting: operation frequency, PWM frequency....
    					I2cMsgIn1.MemoryLowAddr = EEPROM_ADDRESS_LOW_GROUP_1;
    					I2cMsgIn1.MemoryHighAddr = EEPROM_ADDRESS_HIGH_GROUP_1;
        				Load_eeprom();
        				EEPROM_Data.Group_1.Operation_Frequency = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        				EEPROM_Data.Group_1.PWM_Frequency = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        				EEPROM_Data.Group_1.spare = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
						OperationData.VoltageCommand = (float)EEPROM_Data.Group_1.spare;
						Anti_Islanding.Vref = OperationData.VoltageCommand;

        				if (EEPROM_Data.Group_1.Operation_Frequency == 50){
        					system.setting |= FREQ_SET_60HZ_0_50HZ_1;
        				} else if (EEPROM_Data.Group_1.Operation_Frequency == 60){
        					system.setting &= ~FREQ_SET_60HZ_0_50HZ_1;
        				} else {
        					system.setting &= ~FREQ_SET_60HZ_0_50HZ_1;
        					EEPROM_Data.Group_1.Operation_Frequency = 60;
        					system.status &= ~EEPROM_OK;
        				}
						EERPOM_AC_Frequency = EEPROM_Data.Group_1.Operation_Frequency;

        				if(EEPROM_Data.Group_1.PWM_Frequency >= 15000) {
        					EEPROM_Data.Group_1.PWM_Frequency = 15000;
        				}
        				if(EEPROM_Data.Group_1.PWM_Frequency <= 5000) {
        					EEPROM_Data.Group_1.PWM_Frequency = 5000;
        				}
    					FPGA_RegWR_Image.Data_1 |= FAULT_RESET;		// Reset fault register
            			pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
    					FPGA_RegWR_Image.Data_2 &= ~PWM_POWER_CTRL;	// Disable PWM power
    					pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
    				}
    				//-------------------------------------------------------------
    				if (StateMachineCount == 4) {
    					FPGA_RegWR_Image.Data_1 &= ~FAULT_RESET;		// clear fault-reset register
                		pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
					}
    				if (StateMachineCount == 8) {
						EPWM_Boost_Off();
						EPWM_Boost2_Off();
						EPWM_Brake_Off();
						//DO_6_On();
						Sys_Var_Init();
						//--- load AI setting ----
						if (EEPROM_Data.Group_3.AI_Q_inject <= 20) {
		 					Anti_Islanding.sensitivity = EEPROM_Data.Group_3.AI_sensitivity; 		
		 					Anti_Islanding.trip_time = EEPROM_Data.Group_3.AI_trip_time;	 		
		 					iGrid_Tie.Q_inject_Setting = 0.01 * (float)EEPROM_Data.Group_3.AI_Q_inject; 		
						}
					}
    				if (StateMachineCount >= 10) {
    					// Initialize system setting
    					OperationData.PhaseCommand = PHASE_REF_OFFSET;
    					OperationData.Frequency = EEPROM_Data.Group_1.Operation_Frequency;
    					OperationData.PWM_Frequency = EEPROM_Data.Group_1.PWM_Frequency;
    					fTemp = OperationData.PWM_Frequency / OperationData.Frequency;
    					fTemp = fTemp / 3.0;
    					uiTemp = (unsigned int)fTemp;
    					OperationData.Period_Cnt = uiTemp * 3;
    					fTemp = (CPU_FREQUENCY / OperationData.PWM_Frequency)/2;
    					OperationData.PWM_Period_Reg = (unsigned int)fTemp + 1;
    					EPwm1Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
    					EPwm2Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
    					EPwm3Regs.TBPRD = OperationData.PWM_Period_Reg; 			// Period update
    					
						if (UL_1741_MG_Init) {
							UL1741_Setting_Init();
							MicroGrid_init();
							UL_1741_MG_Init = FALSE;
						}
    					// Transfer to next state.......
    					FPGA_RegWR_Image.Data_1 &= ~FAULT_RESET;		// clear fault-reset register
                		pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;

        				pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
            			//if (system.InverterTest == TRUE  && !system.EMS && !system.faults) {
            			if (system.InverterTest == TRUE  && !system.faults) {
        					FPGA_RegWR_Image.Data_2 |= PWM_POWER_CTRL;					// Enable PWM power
    						pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
            				
							DO_6_On();
            				StateMachineCount = 0;
            				InverterTest_timer = 0;
            				system.state = DISCHARGE;								// Only test power modules once
            			} else {
							InitECap1Gpio();
            				StateMachineCount = 0;
            				system.state = SYS_OFF;
							DO_6_Off();
							Sys_Var_Init();
            			}
    				}
    				break;

            	/*** OFF state ***/
            	case	SYS_OFF:
					if (AC_Line_Current_sensing_zero == TRUE) {
						AC_Line_A_Offset = AC_Line_A_Zero;
						AC_Line_B_Offset = AC_Line_B_Zero;
						AC_Line_C_Offset = AC_Line_C_Zero;
						AC_Line_Current_sensing_zero = FALSE;
					}
					//if (StateMachineCount <= 1) {
					//	Data_Measure_Init();
        			//}
        			StateMachineCount ++ ;
					if (StateMachineCount <= 10) {
    					FPGA_RegWR_Image.Data_2 &= ~PWM_POWER_CTRL;	// Disable PWM power
    					pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
					}
					if (StateMachineCount >= 100) StateMachineCount = 100;
        			// ==========================
					// Phase rotation
					// May need to setup EEPROM for final 	 !!! 9/30/13
					if (!PLL_Data.PLL_Enable && Inverter_SysControl.SystemOperationMode == MICRO_GRID_MODE) {
						PLL_Data.PhaseRotation_ABC = TRUE;
						system.setting |= LINE_PH_ROTATION_ABC;
					}
        			// ==========================
        			//  FPGA Test
        			FPGA_Write();		   
        			FPGA_Read();
        			// Turn on Boost, transfer to standby state
    				if (system.command == SysCtrlCmd_BoostOn && !system.EMS) {
        				FPGA_RegWR_Image.Data_2 |= PWM_POWER_CTRL;					// Enable PWM power
    					pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
						Boost_On();
    					system.command = SysCtrlCmd_DoNothing;
        				StateMachineCount = 0;
        				system.state = STANDBY_MODE;
    					break;
    				}
        			// Turn on filter cap contactor and inverter, transfer to discharge state
    				if (system.command == SysCtrlCmd_Discharge && !system.EMS && !system.faults) {
        				FPGA_RegWR_Image.Data_2 |= PWM_POWER_CTRL;					// Enable PWM power
    					pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
						Inverter_SysControl.SystemOperationMode = MICRO_GRID_MODE;
						system.setting &= ~OP_MODE_MG_0_GT_1;
						DO_6_On();									// filter contactor on
						Boost_Off();								// boost off
						DELAY_US(10000L);
						Inverter_On();								// inverter on
    					system.command = SysCtrlCmd_DoNothing;
        				StateMachineCount = 0;
						InverterDischarge_timer = 0;
        				system.state = DISCHARGE;					// goto discharge state
    					break;
    				}
    				// Reset system, transfer to system initialization
    				if (system.command == SysCtrlCmd_SystemReset) {
    					StateMachineCount = 0;
    					system.state = SYS_INIT;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
    					system.status &= ~INVERTER_ON;
    				}
    				// Program flash memory, transfer to FlashProgramming state
    				if (system.command == SysCtrlCmd_FlashProgram) {
    					StateMachineCount = 0;
						flash_Info.crc_CheckSUM = 0xFFFF;
    					system.state = FLASH_PROGRAMMING;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
    					system.status &= ~INVERTER_ON;

    					 // Disable CPU INT2 which is connected to EPWM1-3 INT:
    					IER &= ~M_INT2;
    					// Disable CPU INT3 which is connected to EPWM1-3 INT:
    					IER &= ~M_INT3;
    					// Disable CPU INT4 which is connected to ECAP1-4 INT:
    					IER &= ~M_INT4;

						for (uiTemp=0; uiTemp<128; uiTemp++) {
							FlashMemory_buffer[uiTemp] = 0;
						}
						// Prepare for Flash programming
						FlashMemoryInit();
						ProgramPageActive = FALSE;
						ProgramEraseActive = FALSE;
						PRG_Page_status = 0;
    				}
    				// If fault, transfer to system fault state
    				if (system.faults != 0) {
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
    				}
    				if (system.command == SysCtrlCmd_Fault_Reset || system.command == SysCtrlCmd_SystemReset) {
    					system.command = SysCtrlCmd_DoNothing;
    					system.faults = 0;
    					system.SW_faults = 0;
    					system.SW_warning = 0;
						iGrid_Tie.Trip = 0;
					}
            	   	break;

    			/*** Stand by state ***/
        		case	STANDBY_MODE:
    				StateMachineCount ++ ;
    				// Turn off Boost, transfer off state
    				if (system.command == SysCtrlCmd_BoostOff || system.command == SysCtrlCmd_MainOutputOff || system.EMS) {
						Boost_Off();
    					system.command = SysCtrlCmd_DoNothing;
        				system.state = SYS_OFF;
        				StateMachineCount = 0;
    				}
    				// Turn on PWM........to be continue
    				if (system.command == SysCtrlCmd_MainOutputOn) {
    					if (Inverter_SysControl.SystemOperationMode == MICRO_GRID_MODE) {
							StateTransferCount ++ ;
							if (StateTransferCount == 1) {
								DO_6_On(); // Close filter contactor
							}
							if (StateTransferCount == 2) {
								Inverter_On();
								//DO_7_On(); // Close output contactor
							}
							if (StateTransferCount >= 6) {
    							StateMachineCount = 0;
								PLL_Data.PLL_Enable = FALSE;				// Disable PLL
        						system.status &= ~PLL_ENABLE;
								//---- should delay enable ----				// For test now !!!
								MicroGrid.enable = TRUE;					// Enable micro-grid function
								system.status |= MICRO_GRID_ENABLE;
								DO_5_On(); // Turn on FAN
								DO_7_On(); // Close output contactor
								//Inverter_On();
    							system.state = MICRO_GRID_MODE;
    							system.command = SysCtrlCmd_DoNothing;
        						StateMachineCount = 0;
								StateTransferCount = 0;
							}
						} else if (Inverter_SysControl.SystemOperationMode == GRID_TIE_MODE) {
							StateTransferCount ++ ;
							if (StateTransferCount >= 1 && PLL_Data.PhaseRotation_ABC == TRUE) {
    							StateMachineCount = 0;
    							system.state = GRID_TIE_STANDBY;
    							system.command = SysCtrlCmd_DoNothing;
        						StateMachineCount = 0;
								StateTransferCount = 0;
							}
						}
    				}
    				// Reset system
    				if (system.command == SysCtrlCmd_SystemReset) {
    					StateMachineCount = 0;
    					system.state = SYS_INIT;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
        				StateMachineCount = 0;
    				}
    				// If fault, transfer to system fault state
    				if (system.faults != 0) {
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
    				}
    				break;

        		/*** MicroGrid mode state ***/
        	   	case	MICRO_GRID_MODE:
    				// Shutdown power modules, transfer to OFF state
    				if (system.command == SysCtrlCmd_MainOutputOff || system.EMS) {
    					StateMachineCount = 0;
    					system.state = SYS_OFF;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_5_Off();
						DO_6_Off();
						DO_7_Off();

    				}
    				// If fault, transfer to system fault state
    				if (system.faults != 0) {
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
    				}
        	   		break;

        	   	/*** Grid-Tie mode state ***/
        	   	case	GRID_TIE_MODE:
    				// Shutdown power modules, transfer to OFF state
    				if (system.command == SysCtrlCmd_MainOutputOff || system.EMS) {
    					StateMachineCount = 0;
    					system.state = SYS_OFF;
    					system.command = SysCtrlCmd_DoNothing;
						OperationData.GT_SpeedControl = FALSE;
						iGT_ID_command = 25; 
						OperationData.ID_out_cmd = 25.0;
						Boost_Off();
						Inverter_Off();
						DO_5_Off();
						DO_6_Off();
						DO_7_Off();
					
    				}

    				// If AC line is output of operation range, transfer to GT standby
    				if (PLL_Data.AC_Line_Measure_OK != TRUE) {
    					StateMachineCount = 0;
    					system.state = GRID_TIE_STANDBY;
    					system.command = SysCtrlCmd_DoNothing;
						OperationData.GT_SpeedControl = FALSE;
						iGT_ID_command = 25; 
						OperationData.ID_out_cmd = 25.0;
						//Boost_Off();	  // Keep boost on
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
					
    				}

					//-----------------------------------------
					// Q inject
					//-----------------------------------------
					switch (iGrid_Tie.Q_inject_switch) {
						case 0:
							iGrid_Tie.Q_inject = iGrid_Tie.Q_inject_Setting * OperationData.ID_out_cmd;
							iGrid_Tie.Q_inject_switch = 1;
							break;
					
						case 1:
							iGrid_Tie.Q_inject = 0.0;
							iGrid_Tie.Q_inject_switch = 2;
							break;
					
						case 2:
							iGrid_Tie.Q_inject = -iGrid_Tie.Q_inject_Setting * OperationData.ID_out_cmd;
							iGrid_Tie.Q_inject_switch = 0;
							break;

						default:	// unknown status
							iGrid_Tie.Q_inject = 0.0;
							iGrid_Tie.Q_inject_switch = 0;
							break;
					}
					//----------------------------------------------------------------------------
					// Speed control  Inverter_SysControl.GT_ID_cmd = OperationData.ID_out_cmd	   
					//----------------------------------------------------------------------------
					if (OperationData.GT_SpeedControl == TRUE) {
						GT_SpeedCtrl_Cnt ++ ;
						if (GT_SpeedCtrl_Cnt == 20) {
							speed_control.S_Control_Ki = 1.0;
							speed_control.S_Control_Kp = 15.0;
							speed_control.S_Control_Kd = 0.0;
    						system.status |= GT_SPEED_CTRL;
						}
						if (GT_SpeedCtrl_Cnt >= 30) {
							GT_SpeedCtrl_Cnt = 30;
						}
						//if ((int)OperationData.GT_SpeedCommand > OperationData.int_GT_SpeedCommand + 3) {
						//	OperationData.GT_SpeedCommand -= 3.0;	
						//} else if ((int)OperationData.GT_SpeedCommand < OperationData.int_GT_SpeedCommand - 3) {
						//	OperationData.GT_SpeedCommand += 1.0;	
						//} else {
						//	OperationData.GT_SpeedCommand = (float)OperationData.int_GT_SpeedCommand;	
						//}
						if ((int)OperationData.GT_SpeedCommand < 1621){
							speed_control.S_Control_Ki = 0.2;
							speed_control.S_Control_Kp = 10.0;
							speed_control.S_Control_Kd = 0.0;
						} else {
							speed_control.S_Control_Ki = 1.0;
							speed_control.S_Control_Kp = 15.0;
							speed_control.S_Control_Kd = 0.0;
						}
						//----- current command tracking -----
						//iGT_ID_command = (int)(Inverter_SysControl.GT_ID_cmd * 0.5);
						//if (iGT_ID_command > 40) {
						//	if ((int)OperationData.ID_out_cmd >	iGT_ID_command) {
						//		OperationData.ID_out_cmd -= 1.0;	
						//	} else {
						//		OperationData.ID_out_cmd += 1.0;	
						//	}
						//}

					} else {
    					system.status &= ~GT_SPEED_CTRL;
						OperationData.ID_out_cmd = (float)iGT_ID_command;	
					}
					//-----------------------------------------
    				
    				// If fault, transfer to system fault state
    				if (system.faults != 0) {
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						OperationData.GT_SpeedControl = FALSE;
						iGT_ID_command = 25; 
						OperationData.ID_out_cmd = 25.0;
						Boost_Off();
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
    				}
        	   		break;

        	   	/*** Grid-Tie standby mode state ***/
        	    case	GRID_TIE_STANDBY:
					// Transfer to grid-tie state
        			if (PLL_Data.PLL_State == dPHASE_LOCK && PLL_Data.AC_Line_Measure_OK == TRUE){
						StateTransferCount ++ ;
				   		//if (StateTransferCount == 1) {
				   		//	DO_6_On(); // Close filter contactor !!! for simulation test only
				   		//}
				   		if (StateTransferCount == 1) {
				   			DO_5_On(); // Turn on FAN
				   			DO_7_On(); // Close output contactor
				   		}
				   		if (StateTransferCount >= 6) {
    			   			StateMachineCount = 0;
				   			Inverter_On();
    			   			system.state = GRID_TIE_MODE;
    			   			system.command = SysCtrlCmd_DoNothing;
        		   			StateMachineCount = 0;
				   			StateTransferCount = 0;
				   		}
					}
    				
    				// Shutdown power modules, transfer to OFF state
    				if (system.command == SysCtrlCmd_MainOutputOff || system.EMS) {
    					StateMachineCount = 0;
    					system.state = SYS_OFF;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_5_Off();
						DO_6_Off();
						DO_7_Off();
					
    				}
     				// If fault, transfer to system fault state
    				if (system.faults != 0) {
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
    				}
        	    	break;

        	    /*** Fault state ***/
        	    case	FAULT_CONDITION:
					if (!Brake_Chopper_On) {
    					FPGA_RegWR_Image.Data_2 &= ~PWM_POWER_CTRL;	// Disable PWM power
    					pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
					}
        	    	// Fault reset to bring system back to normal:
    				// Reset system, transfer to system initialization
    				if (system.command == SysCtrlCmd_Fault_Reset || system.command == SysCtrlCmd_SystemReset) {
    					StateMachineCount = 10;
        				// Reset hardware fault:
        				FPGA_RegWR_Image.Data_1 |= FAULT_RESET;		// Reset fault register
                		pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;

    					system.state = SYS_INIT;
    					system.command = SysCtrlCmd_DoNothing;
    					system.faults = 0;
    					system.SW_faults = 0;
    					system.SW_warning = 0;
						system.HostActiveCnt = 0;
						iGrid_Tie.Trip = 0;
						Boost_Off();
						Inverter_Off();
						GT_Anti_Islanding_init();
						DO_1_Off();
						//--- load AI setting ----
						if (EEPROM_Data.Group_3.AI_Q_inject <= 20) {
		 					Anti_Islanding.sensitivity = EEPROM_Data.Group_3.AI_sensitivity; 		
		 					Anti_Islanding.trip_time = EEPROM_Data.Group_3.AI_trip_time;	 		
		 					iGrid_Tie.Q_inject_Setting = 0.01 * (float)EEPROM_Data.Group_3.AI_Q_inject; 		
						}
    				}
    				// Program flash memory, transfer to FlashProgramming state
    				if (system.command == SysCtrlCmd_FlashProgram) {
    					StateMachineCount = 0;
						flash_Info.crc_CheckSUM = 0xFFFF;
    					system.state = FLASH_PROGRAMMING;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
    					system.status &= ~INVERTER_ON;

    					 // Disable CPU INT2 which is connected to EPWM1-3 INT:
    					IER &= ~M_INT2;
    					// Disable CPU INT3 which is connected to EPWM1-3 INT:
    					IER &= ~M_INT3;
    					// Disable CPU INT4 which is connected to ECAP1-4 INT:
    					IER &= ~M_INT4;

						for (uiTemp=0; uiTemp<128; uiTemp++) {
							FlashMemory_buffer[uiTemp] = 0;
						}
						// Prepare for Flash programming
						FlashMemoryInit();
						ProgramPageActive = FALSE;
						ProgramEraseActive = FALSE;
						PRG_Page_status = 0;
    				}
        	    	break;

        	    /*** BUS discharge state ***/
        	    case	DISCHARGE:
        			// Transition to FAULT state:
        			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        			if (system.faults){
    					StateMachineCount = 0;
    					system.state = FAULT_CONDITION;
    					system.command = SysCtrlCmd_DoNothing;
						Boost_Off();
						Inverter_Off();
						DO_6_Off();
						DO_7_Off();
						break;
        			}

        			// Transition to OFF state if doing startup SKIIP reset
        			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        			if (system.InverterTest == TRUE) {
        				InverterTest_timer ++;
        			}

        			if (InverterTest_timer == 2) {
						Boost_On();
       				}

        			if (InverterTest_timer == 4) {
						Boost_Off();
						Inverter_On();
        			}

        			if (InverterTest_timer >= 6) {
						Inverter_Off();
						DO_6_Off();
						Sys_Var_Init();
        				system.state = SYS_OFF;
        				InverterTest_timer = 0;
        				system.InverterTest = FALSE;
        			}
        			// After bus discharge, transition to off state
        			//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        			if ((int)DC_BUS_Voltage <= 5 && system.InverterTest == FALSE) {
						InverterDischarge_timer ++ ;
						if (InverterDischarge_timer >= 10) {
							Inverter_Off();
							DO_6_Off();
        					system.state = SYS_OFF;
        					InverterTest_timer = 0;
        					system.InverterTest = FALSE;
							InverterDischarge_timer = 0;
						}
        			}
    				// Shutdown power modules, transfer to OFF state
    				//if (system.command == SysCtrlCmd_MainOutputOff || system.EMS) {
    				if (system.command == SysCtrlCmd_MainOutputOff) {
    					StateMachineCount = 0;
    					system.state = SYS_OFF;
    					system.command = SysCtrlCmd_DoNothing;
						DO_6_Off();
						DO_7_Off();
						Boost_Off();
						Inverter_Off();

    				}

        	       	break;


        	    /*** Flash programming state ***/
        	    case	FLASH_PROGRAMMING:
					StateMachineCount ++ ;
					if (StateMachineCount >= 100) {
						StateMachineCount = 100;
						if (ProgramPageActive) Flash_Erase_Status = 100;
					}
    				FPGA_RegWR_Image.Data_2 &= ~PWM_POWER_CTRL;	// Disable PWM power
    				pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
					//----- Only hardware reset can get out of this state
					if (StateMachineCount == 5){
						system.FlashEnable = TRUE;		// Stop send out regular CAN messages	 
					}
					//----- Erase flash memory -----
					if (ProgramEraseActive) {
			   			PRG_erase();
						ProgramEraseActive = FALSE;
						FlashMemoryInit();
						StateMachineCount = 0;
					}
					//----- Program one page of flash memory -----
					if (ProgramPageActive) {
						PRG_program();	   // Program a block
						DELAY_US(5000L);							// delay 5msec
    					if(PRG_status == 0)	{
							PRG_Page_status++;
							// send flash memory message back to host
	       			 		eCAN_TxMessage.eCAN_ID = FlashCANMsgID_FlashPageComplete;
	       			 		eCAN_TxMessage.Data_Length = 2;
	       			 		eCAN_TxMessage.Data1 = 1;
	       			 		eCAN_DataTx(&eCAN_TxMessage);
							ProgramPageActive = FALSE;
						}
					}
    				//----- Flash memory complete -----
    				if (system.command == SysCtrlCmd_FlashComplete) {
    					StateMachineCount = 0;
    					system.state = SYS_OFF;
    					system.command = SysCtrlCmd_DoNothing;
						system.FlashEnable = FALSE;
						ProgramPageActive = FALSE;	 
    				}

        	    	break;

        	    default:
        	   		break;

    		} //=== End of switch (system.state) ===

		} //=== end of if (eCAN_RxMessage.New_Message)
    } // end of while() loop
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Sys_Var_Init
//_________________________________________________________________________________
void Sys_Var_Init(){
	/****** Variables initialization ******/
	system.SW_warning = 0;
	system.FlashEnable = FALSE;
	system.HostActiveCnt = 0;
	system.faults = 0;
	system.SW_faults = 0;

	iGrid_Tie.Q_inject = 0.0;
	iGrid_Tie.Q_inject_Setting = 0.10;	 // 10% inject
	iGrid_Tie.Q_inject_switch = 0;

	V_ACLine_D = 0.0;
	V_ACLine_Q = 0.0;
	V_ACLine_Z = 0.0;
	V_ACLine   = 0.0;
	I_ACLine_D = 0.0;
	I_ACLine_Q = 0.0;
	I_ACLine_Z = 0.0;
	I_ACLine   = 0.0;
	
	StateTransferCount = 0;
	EERPOM_report = FALSE;
	hart_beat = 0;
	FPGA_RegWR_Image.Data_1 = 0;
	FPGA_RegWR_Image.Data_2 = 0;
	CommunicationCount = 0;
	Harmonic_TuneCtrl = 2;
	DebugData_OutputCtrl = 10;
	iGrid_Tie.Trip = 0;
	Brake_Chopper_On = FALSE;
	CAN_Bus_Reset = FALSE;

	OperationData.int_GT_SpeedCommand = 1950;
	OperationData.GT_SpeedControl = FALSE;

	MicroGrid.enable = FALSE;	
	system.status &= ~MICRO_GRID_ENABLE;
	// System power on set to micro-grid mode (voltage source)
 	//Inverter_SysControl.SystemOperationMode = MICRO_GRID_MODE;
	//system.setting &= ~OP_MODE_MG_0_GT_1;

	// Flash programming init
	Flash_Erase_Status = 99;
	report_Engine_Speed_RPM = 0.0;

	ADC_vars_init();
	ECAP_Vars_init();
	Data_Measure_Init();
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost_On
//_________________________________________________________________________________
void Boost_On(){
	EPWM_Boost_On();
	system.status |= BOOST_ON;
	FPGA_RegWR_Image.Data_1 |= BOOST_ON_LED;		// turn on led
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost_Off
//_________________________________________________________________________________
void Boost_Off(){
	EPWM_Boost_Off();
	system.status &= ~BOOST_ON;
	FPGA_RegWR_Image.Data_1 &= ~BOOST_ON_LED;		// turn off led
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost2_On
//_________________________________________________________________________________
void Boost2_On(){
	EPWM_Boost2_On();
	system.status |= BOOST2_ON;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost_Off
//_________________________________________________________________________________
void Boost2_Off(){
	EPWM_Boost2_Off();
	system.status &= ~BOOST2_ON;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost_On
//_________________________________________________________________________________
void Brake_On(){
    FPGA_RegWR_Image.Data_2 |= PWM_POWER_CTRL;					// Enable PWM power
    pFPGA_RegWR->Data_2 = FPGA_RegWR_Image.Data_2;
	EPWM_Brake_On();
	system.status |= BRAKE_ON;
	Brake_Chopper_On = TRUE;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Boost_Off
//_________________________________________________________________________________
void Brake_Off(){
	EPWM_Brake_Off();
	system.status &= ~BRAKE_ON;
	Brake_Chopper_On = FALSE;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	BoostCommand
//_________________________________________________________________________________
void BoostCommand(){
Uint16	Boost_DCBUS_Cmd, Boost_DCBUS_Cmd_Active;
	if (system.status & BOOST_ON) {
		Boost_DCBUS_Cmd = (unsigned int)Boost_Control.DCBUS_cmd;
		Boost_DCBUS_Cmd_Active = (unsigned int)Boost_Control.DCBUS_cmd_Active;
		if (Boost_DCBUS_Cmd > Boost_DCBUS_Cmd_Active + 20) {
			Boost_DCBUS_Cmd_Active += 20;
			Boost_Control.DCBUS_cmd_Active = (float)Boost_DCBUS_Cmd_Active;
		} else {
			Boost_Control.DCBUS_cmd_Active = (float)Boost_DCBUS_Cmd;
		}

		if (system.state == GRID_TIE_STANDBY || system.state == GRID_TIE_MODE || system.state == MICRO_GRID_MODE) {
			if ((int)DC_BUS_Voltage < ((int)Boost_Control.DCBUS_cmd_Active - 20)) {
				system.SW_faults |= DC_BUS_UNDERVOLTAGE;
			}
		}
	}
}


//_________________________________________________________________________________
//
// Type:	function
// Name:	Inverter_On
//_________________________________________________________________________________
void Inverter_On(){
   	if (Inverter_SysControl.SystemOperationMode == MICRO_GRID_MODE) {
		EPWM1_On();
	} // else if in Grid-Tie mode, turn on PWM in eCAP1 isr
	Inverter_SysControl.InverterOn = TRUE;
	system.status |= INVERTER_ON;
	FPGA_RegWR_Image.Data_1 |= INVERTER_ON_LED;		// turn on led
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Inverter_Off
//_________________________________________________________________________________
void Inverter_Off(){
	EPWM1_Off();
	Inverter_SysControl.InverterOn = FALSE;
	system.status &= ~INVERTER_ON;
	FPGA_RegWR_Image.Data_1 &= ~INVERTER_ON_LED;   	// turn off led
	pFPGA_RegWR->Data_1 = FPGA_RegWR_Image.Data_1;
	MicroGrid.enable = FALSE;						// Disable micro-grid function
	system.status &= ~MICRO_GRID_ENABLE;
}

//
//  ======== Msg_Processor ========
//  Message Processor
//
void Msg_Processor(void){
Uint16 	uintTemp;
int     iTemp;
int 	Kf;
float   a_1, a_2, a_3;

	if (eCAN_RxMessage.eCAN_ID && eCANMsgID_UL1741_Setting)	{
		if (system.state == STANDBY_MODE || system.state == SYS_OFF || system.state == FAULT_CONDITION) {
			UL1741_Setting_CAN();
		}
	}
	if (eCAN_RxMessage.eCAN_ID && eCANMsgID_MicroGrid_Setting)	{
	   	MicroGrid_Setting_CAN();
	}
	switch (eCAN_RxMessage.eCAN_ID)	{
		case eCANMsgID_CmdSysOperation:				// command: change system status!
			// system status has to be change
			StateTransferCount = 0;
			// -> analyze message contents
			switch (eCAN_RxMessage.Data1) {
				case SysCtrlCmd_MainOutputOn:
					system.command = SysCtrlCmd_MainOutputOn;
					break;

				case SysCtrlCmd_MainOutputOff:
					system.command = SysCtrlCmd_MainOutputOff;
					break;

				case SysCtrlCmd_Fault_Reset:
					system.command = SysCtrlCmd_Fault_Reset;
					break;

				case SysCtrlCmd_BoostOn:
					system.command = SysCtrlCmd_BoostOn;
					break;

				case SysCtrlCmd_BoostOff:
					system.command = SysCtrlCmd_BoostOff;
					break;

				case SysCtrlCmd_2ndBoostOn:
					system.command = SysCtrlCmd_2ndBoostOn;
					break;

				case SysCtrlCmd_2ndBoostOff:
					system.command = SysCtrlCmd_2ndBoostOff;
					break;

				case SysCtrlCmd_BrakeOn:
					system.command = SysCtrlCmd_BrakeOn;
					break;

				case SysCtrlCmd_BrakeOff:
					system.command = SysCtrlCmd_BrakeOff;
					break;

				case SysCtrlCmd_Discharge:
					system.command = SysCtrlCmd_Discharge;
					break;

				case SysCtrlCmd_FlashProgram:
					system.command = SysCtrlCmd_FlashProgram;
					break;

				case SysCtrlCmd_SPI_A_Test:
					break;

				default:	// unknown status
				// do nothing
				// insert failure management here !!!!!		 
				//	s_uiSystem_status |= RECEIVE_ERR_MSG;	 	
					system.command = SysCtrlCmd_DoNothing;
					break;

			} // switch message contents
			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysOperation_Echo;
        	eCAN_TxMessage.Data_Length = 2;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_CmdSysControlMode:
			if (eCAN_RxMessage.Data1 == HOST_ACTIVE) {
				system.HostActiveCnt = 0;
				break;
			}
			// Only allow mode change if module is in OFF state.
			if (system.state == STANDBY_MODE || system.state == SYS_OFF || system.state == FAULT_CONDITION) {
				switch (eCAN_RxMessage.Data1) {
					case CMD_MICRO_GRID_MODE:							 
						Inverter_SysControl.SystemOperationMode = MICRO_GRID_MODE;
						system.setting &= ~OP_MODE_MG_0_GT_1;
						InputStageFilter = 20;
						break;			   								 	
					case CMD_GRID_TIE_MODE:	   		
						Inverter_SysControl.SystemOperationMode = GRID_TIE_MODE;
						system.setting |= OP_MODE_MG_0_GT_1;
						InputStageFilter = 2;
						break;

					case SysCtrlCmd_TestMode:
						system.TestMode = TRUE;
						system.setting |= TEST_MODE;
					break;

					case SysCtrlCmd_TestModeOff:
						system.TestMode = FALSE;
						system.setting &= ~TEST_MODE;
					break;

					default:
						// Wrong data, do nothing!
					break;
				}
			}
			break;

		case eCANMsgID_GridTieTestMode:			// command: change grid-tie test mode
			iGrid_Tie.Feed_Forward = eCAN_RxMessage.Data1;	// 1: open loop, 0: close loop
			iGrid_Tie.SoftStart_cnt = eCAN_RxMessage.Data2;	// soft start control
			iGrid_Tie.HarmonicControl = eCAN_RxMessage.Data3;		// GT PWM mode: 1 = enable, 0 = disable
			iGrid_Tie.ThirdHarmonic = eCAN_RxMessage.Data4;		// GT PWM mode: 1 = SV PWM, 0 = sine PWM

			// send the same data back to host
	        eCAN_TxMessage.eCAN_ID = eCANMsgID_GridTieTestMode_Echo;
	        eCAN_TxMessage.Data_Length = 8;
	        eCAN_TxMessage.Data1 = iGrid_Tie.Feed_Forward;
	        eCAN_TxMessage.Data2 = iGrid_Tie.SoftStart_cnt;
	        eCAN_TxMessage.Data3 = iGrid_Tie.HarmonicControl;
	        eCAN_TxMessage.Data4 = iGrid_Tie.ThirdHarmonic;
	        eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_GridTieTestData:						// For grid tie initial condition
			iTemp = eCAN_RxMessage.Data1;					// D initial value
			iGrid_Tie.D_init = 0.01 * iTemp;				// soft start control
			iTemp = eCAN_RxMessage.Data2;					// Q initial value
			iGrid_Tie.Q_init = 0.01 * iTemp;				// soft start control
			GT_CtrlLoop_TIME = eCAN_RxMessage.Data3;		// GT control timing

			// send the same data back to host
	        eCAN_TxMessage.eCAN_ID = eCANMsgID_GridTieTestData_Echo;
	        eCAN_TxMessage.Data_Length = 6;
	        eCAN_TxMessage.Data1 = (int)(iGrid_Tie.D_init * 100.0);
	        eCAN_TxMessage.Data2 = (int)(iGrid_Tie.Q_init * 100.0);
	        eCAN_TxMessage.Data3 = GT_CtrlLoop_TIME;
	        eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_CmdSysParam_VDCBUS:			// command: DCBUS voltage
			uintTemp = eCAN_RxMessage.Data1;
			if (uintTemp > 900) uintTemp = 900;	// upper limit
			if (uintTemp < 10) uintTemp = 10;	// lower limit
			Boost_Control.DCBUS_cmd = (float)uintTemp;	//eCAN_RxMessage.Data1: 1bit-> 1V

			uintTemp = eCAN_RxMessage.Data2;
			if (uintTemp > 500) uintTemp = 500;	// upper limit
			if (uintTemp < 10) uintTemp = 10;	// lower limit
			Boost_Control.DCBUS_I_Limit_x10 = (float)uintTemp * 10.0;	//eCAN_RxMessage.Data1: 1bit-> 0.1A

			uintTemp = eCAN_RxMessage.Data3;
			if (uintTemp > 400) uintTemp = 400;	// upper limit
			if (uintTemp < 10) uintTemp = 10;	// lower limit
			Boost_Control.DCBUS_OverCurrent_x10 = uintTemp * 10;		//eCAN_RxMessage.Data1: 1bit-> 0.1A

			uintTemp = eCAN_RxMessage.Data4;
			if (uintTemp > 400) uintTemp = 400;	// upper limit
			if (uintTemp < 10) uintTemp = 10;	// lower limit
			Generator_Current_Limit = uintTemp;		//eCAN_RxMessage.Data1: 1bit-> 0.1A

			// send the same data back to host
	        eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_VDCBUS_Echo;
	        eCAN_TxMessage.Data_Length = 8;
	        eCAN_TxMessage.Data1 = (int)Boost_Control.DCBUS_cmd;
	        eCAN_TxMessage.Data2 = Boost_Control.DCBUS_I_Limit_x10/10;
	        eCAN_TxMessage.Data3 = Boost_Control.DCBUS_OverCurrent_x10/10;
	        eCAN_TxMessage.Data4 = Generator_Current_Limit;
	        eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_CmdSysParam_Vout:			// command: Output voltage
			uintTemp = eCAN_RxMessage.Data1;
			if (uintTemp > 480) uintTemp = 480;	// upper limit
			if (uintTemp < 50) uintTemp = 50;	// lower limit
			OperationData.VoltageCommand = (float)uintTemp;	//eCAN_RxMessage.Data1: 1bit-> 1V
			//Inverter_SysControl.Vout_cmd = OperationData.VoltageCommand;
			//fTemp = OperationData.VoltageCommand/SYSTEM_OUTPUT_VOLTAGE_MAX;
			//epwm1_info.EPwmGAIN = fTemp;
			//epwm2_info.EPwmGAIN = fTemp;
			//epwm3_info.EPwmGAIN = fTemp;
			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Vout_Echo;
        	eCAN_TxMessage.Data_Length = 2;
        	eCAN_TxMessage.Data1 = uintTemp;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_CmdSysParam_Iout:			// command: Output current
			iTemp = eCAN_RxMessage.Data1;
			if (iTemp > 150) iTemp = 150;		// upper limit
			if (iTemp < 0) iTemp = 0;			// lower limit
			//if (iTemp < -50) iTemp = -50;		// lower limit for open loop test
			if (OperationData.GT_SpeedControl == FALSE) {
				OperationData.ID_out_cmd = (float)iTemp;	//eCAN_RxMessage.Data1: 1bit-> 1A
			}
			iGT_ID_command = iTemp; 
			
			iTemp = eCAN_RxMessage.Data2;
			if (iTemp > 80) iTemp = 80;		// upper limit
			if (iTemp < -80) iTemp = -80;			// lower limit
			OperationData.IQ_out_cmd = (float)iTemp;	//eCAN_RxMessage.Data1: 1bit-> 1A

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Iout_Echo;
        	eCAN_TxMessage.Data_Length = 4;
        	eCAN_TxMessage.Data1 = iGT_ID_command;
        	eCAN_TxMessage.Data2 = (int)OperationData.IQ_out_cmd;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_CmdSysParam_Phase:			// command: phase offset
			intCAN_Data = (int)eCAN_RxMessage.Data1;
			if (intCAN_Data > 314) intCAN_Data = 314;	// upper limit
			if (intCAN_Data < -314) intCAN_Data = -314;	// upper limit
			//if(PLL_Data.PLL_Enable == FALSE) {
			//	OperationData.PhaseCommand = (float)intCAN_Data/100.0 + PHASE_REF_OFFSET;	//eCAN_RxMessage.Data1: 1bit-> 0.1V
			//} else {
			//	PLL_Data.Phase_Offset = intCAN_Data;
			//}
			PLL_Data.Q_command = (float)intCAN_Data;
			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Phase_Echo;
        	eCAN_TxMessage.Data_Length = 2;
        	eCAN_TxMessage.Data1 = (unsigned int)intCAN_Data;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_GridTieSpeedCtrl:			// command: engine speed, speed control active
			iTemp = eCAN_RxMessage.Data1;
			if (iTemp > 3500) iTemp = 3500;			// upper limit
			//if (iTemp < 1000) iTemp = 1000;			// lower limit
			OperationData.int_GT_SpeedCommand = iTemp;

			OperationData.GT_SpeedControl = eCAN_RxMessage.Data2;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_GridTieSpeedCtrl_Echo;
        	eCAN_TxMessage.Data_Length = 4;
        	eCAN_TxMessage.Data1 = OperationData.int_GT_SpeedCommand;
        	eCAN_TxMessage.Data2 = OperationData.GT_SpeedControl;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_SystemReset:					// Report EEPROM data
			if (system.state == STANDBY_MODE || system.state == SYS_OFF || system.state == FAULT_CONDITION) {
				system.command = SysCtrlCmd_SystemReset;
			}
        	break;

		case eCANMsgID_DigitalOutTest:					// Digital output test
			if (system.state == STANDBY_MODE || system.state == SYS_OFF || system.state == FAULT_CONDITION) {
				Digitalout_Image = eCAN_RxMessage.Data1;
				// Output active
				Digitalout_Image &= 0x00FF;
				if (Digitalout_Image == 0x0001) DO_1_On();
				if (Digitalout_Image == 0x0002) DO_2_On();
				if (Digitalout_Image == 0x0004) DO_3_On();
				if (Digitalout_Image == 0x0008) DO_4_On();
				if (Digitalout_Image == 0x0010) DO_5_On();
				if (Digitalout_Image == 0x0020) DO_6_On();
				if (Digitalout_Image == 0x0040) DO_7_On();
				// Output de-active
				Digitalout_Image = eCAN_RxMessage.Data1>>8;
				if (Digitalout_Image == 0x0001) DO_1_Off();
				if (Digitalout_Image == 0x0002)	DO_2_Off();
				if (Digitalout_Image == 0x0004)	DO_3_Off();
				if (Digitalout_Image == 0x0008)	DO_4_Off();
				if (Digitalout_Image == 0x0010) DO_5_Off();
				if (Digitalout_Image == 0x0020) DO_6_Off();
				if (Digitalout_Image == 0x0040) DO_7_Off();

			}
        	break;

		case eCANMsgID_DataOutputCtrl:				
			DAC_OutputCtrl = eCAN_RxMessage.Data1;			// DAC output control
			Harmonic_TuneCtrl = eCAN_RxMessage.Data2;		// Harmonic adjust control
			if (Harmonic_TuneCtrl !=2) Harmonic_TuneCtrl = 5;
			DebugData_OutputCtrl = eCAN_RxMessage.Data3;	// Debug data output (CAN) control
			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_DACOutputChannel_Echo;
        	eCAN_TxMessage.Data_Length = 6;
        	eCAN_TxMessage.Data1 = DAC_OutputCtrl;
        	eCAN_TxMessage.Data2 = Harmonic_TuneCtrl;
        	eCAN_TxMessage.Data3 = DebugData_OutputCtrl;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_BoostControlParameters:			// Boost controller
			uintTemp = eCAN_RxMessage.Data1;
			Boost_Control.V_Control_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data2;
			Boost_Control.V_Control_Ki = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data3;
			Boost_Control.I_Control_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data4;
			Boost_Control.I_Control_Ki = (float)uintTemp / 1000.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_BoostControlParameters_Echo;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_VolageControlParameters:			// Inverter controller
			uintTemp = eCAN_RxMessage.Data1;
			Inverter_SysControl.V_Control_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data2;
			Inverter_SysControl.V_Control_Ki = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data3;
			Inverter_SysControl.I_Control_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data4;
			Inverter_SysControl.I_Control_Ki = (float)uintTemp / 1000.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_VolageControlParameters_Echo;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_PLLControlParameters:			// PLL controller
			uintTemp = eCAN_RxMessage.Data1;
			PLL_Data.PLL_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data2;
			PLL_Data.PLL_Ki = (float)uintTemp / 1000.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_PLLControlParameters_Echo;
        	eCAN_TxMessage.Data_Length = 4;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_GridTieControlParameters:			// Grid-Tie controller
			uintTemp = eCAN_RxMessage.Data1;
			Inverter_SysControl.GT_I_Control_Kp = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data2;
			Inverter_SysControl.GT_I_Control_Ki = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data3;
			Inverter_SysControl.GT_I_Control_Kc = (float)uintTemp / 1000.0;

			uintTemp = eCAN_RxMessage.Data4;
			Inverter_SysControl.GT_I_Control_Kd = (float)uintTemp / 1000.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_GridTieControlParameters_Echo;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_EngineSpeedControlParameters:			// speed controller
			uintTemp = eCAN_RxMessage.Data1;
			speed_control.S_Control_Kp = (float)uintTemp / 100.0; 

			uintTemp = eCAN_RxMessage.Data2;
			speed_control.S_Control_Ki = (float)uintTemp / 100.0;

			eCAN_RxMessage.Data3 = 11;

			uintTemp = eCAN_RxMessage.Data4;
			speed_control.S_Control_Kd = (float)uintTemp / 100.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_EngineSpeedControlParameters_Echo;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_Filter_Setting1:			// filter constants
			Kf = (int)eCAN_RxMessage.Data1;
			a_1 = (2 * OperationData.PWM_Frequency * Kf);
			a_2 = (2*PI*OperationData.Frequency) * (2*PI*OperationData.Frequency);
			a_3 = OperationData.PWM_Frequency * OperationData.PWM_Frequency;

			I_Line_SF_K1 = a_1 / (a_2 + a_1 + 4 * a_3);
			I_Line_SF_K2 = (8 * a_3 - 2 * a_2) / (a_2 + a_1 + 4 * a_3);
			I_Line_SF_K3 = -(a_2 - a_1 + 4 * a_3) / (a_2 + a_1 + 4 * a_3);
			
			//I_Line_SF_K1 = (int)eCAN_RxMessage.Data1 / 10000.0;
			//I_Line_SF_K2 = (int)eCAN_RxMessage.Data2 / 10000.0;
			//I_Line_SF_K3 = (int)eCAN_RxMessage.Data3 / 10000.0;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_Filter_Setting1_Echo;
        	eCAN_TxMessage.Data_Length = 6;
        	eCAN_TxMessage.Data1 = (int)(10000.0 * I_Line_SF_K1);
        	eCAN_TxMessage.Data2 = (int)(10000.0 * I_Line_SF_K2);
        	eCAN_TxMessage.Data3 = (int)(10000.0 * I_Line_SF_K3);
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_GTFifthHarmonicCtrl:			// Grid-Tie controller
			if (Harmonic_TuneCtrl == 2) {
				uintTemp = eCAN_RxMessage.Data1;
				GT_2nd_harm_ctrl.Kp = (float)uintTemp / 10000.0;// Minimum int data = 10

				uintTemp = eCAN_RxMessage.Data2;
				GT_2nd_harm_ctrl.Ki = (float)uintTemp / 10000.0;// Minimum int data = 1

				uintTemp = eCAN_RxMessage.Data3;
				GT_2nd_harm_ctrl.Filter_K1 = (float)uintTemp / 10000.0;

				uintTemp = eCAN_RxMessage.Data4;
				GT_2nd_harm_ctrl.Filter_K2 = (float)uintTemp / 10000.0;
			} else {
				uintTemp = eCAN_RxMessage.Data1;
				GT_5th_harm_ctrl.Kp = (float)uintTemp / 10000.0;// Minimum int data = 10

				uintTemp = eCAN_RxMessage.Data2;
				GT_5th_harm_ctrl.Ki = (float)uintTemp / 10000.0;// Minimum int data = 1

				uintTemp = eCAN_RxMessage.Data3;
				GT_5th_harm_ctrl.Filter_K1 = (float)uintTemp / 10000.0;

				uintTemp = eCAN_RxMessage.Data4;
				GT_5th_harm_ctrl.Filter_K2 = (float)uintTemp / 10000.0;
			}
			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_GTFifthHarmonicCtrl_Echo;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
			break;

		case eCANMsgID_ReadEEPROM:					// Read EEPROM
			// EEPROM address
			uintTemp = eCAN_RxMessage.Data1 * 8;
			I2cMsgIn1.MemoryLowAddr = uintTemp & 0x00FF;
			I2cMsgIn1.MemoryHighAddr = uintTemp >> 8;

			I2cMsgIn1.NumOfBytes = 8;
			Load_eeprom();

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_ReadEEPROMEcho;
        	eCAN_TxMessage.Data_Length = 8;
        	uintTemp = I2cMsgIn1.MemoryLowAddr + (I2cMsgIn1.MemoryHighAddr << 8);
        	eCAN_TxMessage.Data1 = uintTemp;
        	uintTemp = I2cMsgIn1.MsgBuffer[0] + (I2cMsgIn1.MsgBuffer[1] << 8);
        	eCAN_TxMessage.Data2 = uintTemp;
        	uintTemp = I2cMsgIn1.MsgBuffer[2] + (I2cMsgIn1.MsgBuffer[3] << 8);
        	eCAN_TxMessage.Data3 = uintTemp;
        	uintTemp = I2cMsgIn1.MsgBuffer[4] + (I2cMsgIn1.MsgBuffer[5] << 8);
        	eCAN_TxMessage.Data4 = uintTemp;
        	eCAN_DataTx(&eCAN_TxMessage);
        	break;

		case eCANMsgID_WriteEEPROM:					// Write EEPROM
			// if (system.state != SYS_OFF)	break;	!!!!!
			// EEPROM address
			if (eCAN_RxMessage.Data1 == 0) 
				break;
			uintTemp = eCAN_RxMessage.Data1 * 8;
			I2cMsgOut1.MemoryLowAddr = uintTemp & 0x00FF;
			I2cMsgOut1.MemoryHighAddr = uintTemp >> 8;

			// EEPROM Data 1
			uintTemp = eCAN_RxMessage.Data2 & 0x00FF;
			I2cMsgOut1.MsgBuffer[0] = uintTemp;
			uintTemp = eCAN_RxMessage.Data2 >> 8;
			I2cMsgOut1.MsgBuffer[1] = uintTemp;

			// EEPROM Data 2
			uintTemp = eCAN_RxMessage.Data3 & 0x00FF;
			I2cMsgOut1.MsgBuffer[2] = uintTemp;
			uintTemp = eCAN_RxMessage.Data3 >> 8;
			I2cMsgOut1.MsgBuffer[3] = uintTemp;

			// EEPROM Data 3
			uintTemp = eCAN_RxMessage.Data4 & 0x00FF;
			I2cMsgOut1.MsgBuffer[4] = uintTemp;
			uintTemp = eCAN_RxMessage.Data4 >> 8;
			I2cMsgOut1.MsgBuffer[5] = uintTemp;

			// send the same data back to host
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_WriteEEPROMEcho;
        	eCAN_TxMessage.Data_Length = 8;
        	eCAN_TxMessage.Data1 = eCAN_RxMessage.Data1;
        	eCAN_TxMessage.Data2 = eCAN_RxMessage.Data2;
        	eCAN_TxMessage.Data3 = eCAN_RxMessage.Data3;
        	eCAN_TxMessage.Data4 = eCAN_RxMessage.Data4;
        	eCAN_DataTx(&eCAN_TxMessage);
        	// Write to EEPROM
        	I2cMsgOut1.NumOfBytes = 8;	// Note: 6 bytes data + 2 bytes check-sum
        	I2CA_WriteData(&I2cMsgOut1);
        	break;

		default:	// unknown status
			// do nothing
			break;
	}
}


//
//  ======== Flash Program Msg_Processor ========
//  Flash Program Message Processor
//
void Flash_Msg_Processor(void){
Uint32	lngTemp;
Uint16	uintTemp;
Uint16	i,j;

	lngTemp = eCAN_RxMessage.eCAN_ID & 0x0F000000;
	if (lngTemp == FlashCANMsgID_Data) {
	// Receive data
		Buufer_Index = eCAN_RxMessage.eCAN_ID & 0x000000FF;
		lngTemp =  4 * (Buufer_Index - 1);
		FlashMemory_buffer[lngTemp] = eCAN_RxMessage.Data1;
		Buffer[lngTemp]	= FlashMemory_buffer[lngTemp];
		FlashMemory_buffer[lngTemp+1] = eCAN_RxMessage.Data2;
		Buffer[lngTemp+1]	= FlashMemory_buffer[lngTemp+1];
		FlashMemory_buffer[lngTemp+2] = eCAN_RxMessage.Data3;
		Buffer[lngTemp+2]	= FlashMemory_buffer[lngTemp+2];
		FlashMemory_buffer[lngTemp+3] = eCAN_RxMessage.Data4;
		Buffer[lngTemp+3]	= FlashMemory_buffer[lngTemp+3];

		if (Buufer_Index == 0x00000020) {
		   	ProgramPageActive = TRUE;
		}
		//-------------------------------------------------
		//    Check Sum Calculation
		//-------------------------------------------------
		for (j = 0; j <=3; j++)	{
			flash_Info.crc_CheckSUM = flash_Info.crc_CheckSUM ^ (Buffer[lngTemp + j] & 0x00FF);
			for (i=0; i < 8; i++){
				if ((flash_Info.crc_CheckSUM & 0x0001) != 0)
					flash_Info.crc_CheckSUM = (flash_Info.crc_CheckSUM >> 1) ^ 0xA001;
				else  flash_Info.crc_CheckSUM = (flash_Info.crc_CheckSUM >> 1);
			}
			flash_Info.crc_CheckSUM = flash_Info.crc_CheckSUM ^ (Buffer[lngTemp + j]>>8 & 0x00FF);
			for (i=0; i < 8; i++){
				if ((flash_Info.crc_CheckSUM & 0x0001) != 0)
					flash_Info.crc_CheckSUM = (flash_Info.crc_CheckSUM >> 1) ^ 0xA001;
				else  flash_Info.crc_CheckSUM = (flash_Info.crc_CheckSUM >> 1);
			}
		}

	} else {
	// Receive command
		switch (eCAN_RxMessage.eCAN_ID)	{
			case	FlashCANMsgID_EraseCmd :				// command: erase EE flash memory chip
			   	Flash_Erase_Status = 66;
				ProgramEraseActive = TRUE;
    	    	break;

			case	FlashCANMsgID_FlashFinish:
				system.command = SysCtrlCmd_FlashComplete;
				uintTemp = eCAN_RxMessage.Data1*100;
				uintTemp += eCAN_RxMessage.Data2;
				flash_Info.Flash_Date = uintTemp;
				flash_Info.Flash_Year = eCAN_RxMessage.Data3;

				// Write to EEPROM Data Group 0
				uintTemp = 0;				   // Group 0
				I2cMsgOut1.MemoryLowAddr = uintTemp & 0x00FF;
				I2cMsgOut1.MemoryHighAddr = uintTemp >> 8;

				// EEPROM Data 1
				uintTemp = flash_Info.crc_CheckSUM & 0x00FF;
				I2cMsgOut1.MsgBuffer[0] = uintTemp;
				uintTemp = flash_Info.crc_CheckSUM >> 8;
				I2cMsgOut1.MsgBuffer[1] = uintTemp;

				// EEPROM Data 2
				uintTemp = flash_Info.Flash_Date & 0x00FF;
				I2cMsgOut1.MsgBuffer[2] = uintTemp;
				uintTemp = flash_Info.Flash_Date >> 8;
				I2cMsgOut1.MsgBuffer[3] = uintTemp;

				// EEPROM Data 3
				uintTemp = flash_Info.Flash_Year & 0x00FF;
				I2cMsgOut1.MsgBuffer[4] = uintTemp;
				uintTemp = flash_Info.Flash_Year >> 8;
				I2cMsgOut1.MsgBuffer[5] = uintTemp;

				// send the flash data back to host
        		eCAN_TxMessage.eCAN_ID = eCANMsgID_WriteEEPROMEcho;
        		eCAN_TxMessage.Data_Length = 8;
        		eCAN_TxMessage.Data1 = 0;
        		eCAN_TxMessage.Data2 = flash_Info.crc_CheckSUM;
        		eCAN_TxMessage.Data3 = flash_Info.Flash_Date;
        		eCAN_TxMessage.Data4 = flash_Info.Flash_Year;
        		eCAN_DataTx(&eCAN_TxMessage);
        		// Write to EEPROM
        		I2cMsgOut1.NumOfBytes = 8;	// Note: 6 bytes data + 2 bytes check-sum
        		I2CA_WriteData(&I2cMsgOut1);

				break;
			default:	// unknown status
				// do nothing
				break;
		}
	}
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	Data_Report							
//_________________________________________________________________________________
void Data_Report(){
static int itempCount=0;
int tempData;
	switch (itempCount) {
		case 0:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_Digital_IO;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = Digital_IO.all;
			eCAN_TxMessage.Data2 = (unsigned int)(10.0 * T_ambient_C);
			report_Engine_Speed_RPM = 1.0 * Engine_Speed_RPM + 0.0 * report_Engine_Speed_RPM;
			eCAN_TxMessage.Data3 = (unsigned int)report_Engine_Speed_RPM;
			eCAN_TxMessage.Data4 = CommunicationCount;
			eCAN_DataTx(&eCAN_TxMessage);
			CommunicationCount++;
			itempCount = 1;
			break;

		case 1:
			tempData = (int)(100.0 * ACLine.Frequency);
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_ACLineData1;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = OperationData.PWM_Period_Reg;
			eCAN_TxMessage.Data2 = (int) OperationData.PWM_Frequency;
			eCAN_TxMessage.Data3 = PLL_Data.PLL_State;
			eCAN_TxMessage.Data4 = tempData;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 2;
			break;

		case 2:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_ACLineData2;
			eCAN_TxMessage.Data_Length = 8;
			V_ACLine_D = 0.3 * AC_Line_Measurement.V_Line_D + 0.7 * V_ACLine_D;
			V_ACLine_Q = 0.3 * AC_Line_Measurement.V_Line_Q + 0.7 * V_ACLine_Q;
			V_ACLine_Z = 0.3 * AC_Line_Measurement.V_Line_Zero + 0.7 * V_ACLine_Z;
			V_ACLine   = 0.3 * AC_Line_Measurement.V_Line   + 0.7 * V_ACLine;

			eCAN_TxMessage.Data1 = (int)V_ACLine_D;
			eCAN_TxMessage.Data2 = (int)V_ACLine_Q;
			eCAN_TxMessage.Data3 = (int)V_ACLine_Z;
			eCAN_TxMessage.Data4 = (int)V_ACLine;  
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 3;
			break;

		case 3:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_ACLineData3;
			eCAN_TxMessage.Data_Length = 8;
			I_ACLine_D = 0.3 * AC_Line_Measurement.I_Line_D + 0.7 * I_ACLine_D;
			I_ACLine_Q = 0.3 * AC_Line_Measurement.I_Line_Q + 0.7 * I_ACLine_Q;
			I_ACLine_Z = 0.3 * AC_Line_Measurement.I_Line_Zero + 0.7 * I_ACLine_Z;
			I_ACLine   = 0.3 * AC_Line_Measurement.I_Line   + 0.7 * I_ACLine;

			eCAN_TxMessage.Data1 = (int)(10.0 * I_ACLine_D);
			eCAN_TxMessage.Data2 = (int)(10.0 * I_ACLine_Q);
			eCAN_TxMessage.Data3 = (int)(10.0 * I_ACLine_Z);
			eCAN_TxMessage.Data4 = (int)(10.0 * I_ACLine);  
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 4;
			break;

		case 4:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_DCBusData;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(10.0 * DC_BUS_Voltage);
			eCAN_TxMessage.Data2 = (int)(10.0 * I_Boost_report);
			eCAN_TxMessage.Data3 = (int)(Boost_Control.DCBUS_V_error);
			eCAN_TxMessage.Data4 = Boost_Control.EPwmDuty;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 5;
			break;

		case 5:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_ACLineVrms;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)AC_Line_Measurement.V_AB_RMS;
			eCAN_TxMessage.Data2 = (int)AC_Line_Measurement.V_BC_RMS;
			eCAN_TxMessage.Data3 = (int)AC_Line_Measurement.V_CA_RMS;
			eCAN_TxMessage.Data4 = (int)Anti_Islanding.V_error;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 6;
			break;

		case 6:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_ACLineIrms;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)(AC_Line_Measurement.I_A_RMS * 10.0);
			eCAN_TxMessage.Data2 = (int)(AC_Line_Measurement.I_B_RMS * 10.0);
			eCAN_TxMessage.Data3 = (int)(AC_Line_Measurement.I_C_RMS * 10.0);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 7;
			break;

		case 7:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_I_GEN_rms;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)(Generator_IA_RMS * 10.0);
			eCAN_TxMessage.Data2 = (int)(Generator_IB_RMS * 10.0);
			eCAN_TxMessage.Data3 = (int)(Generator_IC_RMS * 10.0);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 8;
			break;

		case 8:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_InverterOutData1;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)VAC_out_Measurement.V_Line_D;
			eCAN_TxMessage.Data2 = (int)VAC_out_Measurement.V_Line_Q;
			eCAN_TxMessage.Data3 = (int)VAC_out_Measurement.V_Line_Zero;
			eCAN_TxMessage.Data4 = (int)VAC_out_Measurement.V_Line;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 9;
			break;

		case 9:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_InverterOutData2;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(10.0 * VAC_out_Measurement.I_Line_D);
			eCAN_TxMessage.Data2 = (int)(10.0 * VAC_out_Measurement.I_Line_Q);
			eCAN_TxMessage.Data3 = (int)(10.0 * VAC_out_Measurement.I_Line_Zero);
			eCAN_TxMessage.Data4 = (int)(10.0 * VAC_out_Measurement.I_Line);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 10;
			break;

		case 10:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_InverterOutData3;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)VAC_out_Measurement.V_AB_RMS;
			eCAN_TxMessage.Data2 = (int)VAC_out_Measurement.V_BC_RMS;
			eCAN_TxMessage.Data3 = (int)VAC_out_Measurement.V_CA_RMS;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 12;		//Skip inverter current reading
			break;

		case 11:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_InverterOutData4;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = (int)(VAC_out_Measurement.I_A * 10.0);
			eCAN_TxMessage.Data2 = (int)(VAC_out_Measurement.I_B * 10.0);
			eCAN_TxMessage.Data3 = (int)(VAC_out_Measurement.I_C * 10.0);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 12;
			break;


		case 12:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_MicroGridData1;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(MicroGrid.P * 10.0);
			eCAN_TxMessage.Data2 = (int)(MicroGrid.Q * 10.0);
			eCAN_TxMessage.Data3 = (int)(MicroGrid.V_cmd * 10.0);
			eCAN_TxMessage.Data4 = (int)(MicroGrid.Frequency * 100.0);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 13;
			break;

		case 13:
			eCAN_TxMessage.eCAN_ID = DataMeasurementMsgID_InverterPower;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)Output_KW ;
			eCAN_TxMessage.Data2 = (int)Output_KVA;
			eCAN_TxMessage.Data3 = (int)(MicroGrid.KVA * 10.0);
			eCAN_TxMessage.Data4 = (int)(iGrid_Tie.Q_inject * 10.0);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 14;
			break;

		case 14:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_SystemDebugData1;
			eCAN_TxMessage.Data_Length = 8;
			switch(DebugData_OutputCtrl) {
				// Read data under the DebugData_OutputCtrl
				case 0:
					eCAN_TxMessage.Data1 = (int)(VAC_out_Measurement.I_A * 10.0);
					eCAN_TxMessage.Data2 = (int)(VAC_out_Measurement.I_B * 10.0);
					eCAN_TxMessage.Data3 = (int)(VAC_out_Measurement.I_C * 10.0);
					eCAN_TxMessage.Data4 = Boost_Currentx10;
					break;
				
				case 1:
					eCAN_TxMessage.Data1 = (int)(Boost_Control.DCBUS_I_error);
					eCAN_TxMessage.Data2 = (int)(Boost_Control.DCBUS_I_error_int);
					eCAN_TxMessage.Data3 = (int)(Boost_Control.DCBUS_I_Loopout);
					eCAN_TxMessage.Data4 = (int)(Boost_Control.DCBUS_I_cmd);
					break;

				case 2:
					eCAN_TxMessage.Data1 = (int)(Inverter_SysControl.GT_IDout_error * 1000);
					eCAN_TxMessage.Data2 = (int)(Inverter_SysControl.GT_ID_error_int * 1000);
					eCAN_TxMessage.Data3 = (int)(Inverter_SysControl.GT_ID_error_int_old * 1000);
					eCAN_TxMessage.Data4 = (int)(Inverter_SysControl.GT_ID_out * 1000);
					break;

				case 3:
					eCAN_TxMessage.Data1 = (int)Boost_Control.DCBUS_cmd;
					eCAN_TxMessage.Data2 = DC_bus_OV_setting; 
					eCAN_TxMessage.Data3 = PLL_Data.AC_Line_Measure_OK; //Brake_ON_V_setting;
					eCAN_TxMessage.Data4 = StateTransferCount;
					break;

				case 4:
					eCAN_TxMessage.Data1 = (int)(Inverter_SysControl.GT_IQout_error * 1000);
					eCAN_TxMessage.Data2 = (int)(Inverter_SysControl.GT_IQ_error_int *1000);
					eCAN_TxMessage.Data3 = (int)(Inverter_SysControl.GT_IQ_error_int_old * 1000);
					eCAN_TxMessage.Data4 = (int)(Inverter_SysControl.GT_IQ_out * 1000);
					break;

				case 5:
					eCAN_TxMessage.Data1 = iGrid_Tie.Feed_Forward;
					eCAN_TxMessage.Data2 = iGrid_Tie.SoftStart_cnt;
					eCAN_TxMessage.Data3 = iGrid_Tie.HarmonicControl;
					eCAN_TxMessage.Data4 = iGrid_Tie.ThirdHarmonic;
					break;

				case 6:
					eCAN_TxMessage.Data1 = VF_Trip.V_Lower_Limit1_cnt;
					eCAN_TxMessage.Data2 = VF_Trip.V_Lower_Limit2_cnt;
					eCAN_TxMessage.Data3 = VF_Trip.V_Upper_Limit1_cnt;
					eCAN_TxMessage.Data4 = VF_Trip.V_Upper_Limit2_cnt;
					break;

				case 7:
					eCAN_TxMessage.Data1 = VF_Trip.F_Lower_Limit1_cnt;
					eCAN_TxMessage.Data2 = VF_Trip.F_Lower_Limit2_cnt;
					eCAN_TxMessage.Data3 = VF_Trip.F_Upper_Limit1_cnt;
					eCAN_TxMessage.Data4 = VF_Trip.F_Upper_Limit2_cnt;
					break;

				case 8:
					eCAN_TxMessage.Data1 = (int)(100.0 * speed_control.speed_error);
					eCAN_TxMessage.Data2 = (int)(100.0 * speed_control.speed_error_int);
					eCAN_TxMessage.Data3 = (int)(100.0 * speed_control.speed_error_der);
					eCAN_TxMessage.Data4 = (int)(100.0 * speed_control.speed_Loop_Compensator);
					break;

				case 9:
					eCAN_TxMessage.Data1 = (int)(100.0 * speed_control.S_Control_Ki);
					eCAN_TxMessage.Data2 = (int)(100.0 * speed_control.S_Control_Kp);
					eCAN_TxMessage.Data3 = (int)(OperationData.GT_SpeedCommand);
					eCAN_TxMessage.Data4 = (int)OperationData.ID_out_cmd;
					break;

				case 10:
					eCAN_TxMessage.Data1 = (int)(AC_Line_A_Zero * 1000.0);
					eCAN_TxMessage.Data2 = (int)(AC_Line_B_Zero * 1000.0);
					eCAN_TxMessage.Data3 = (int)(AC_Line_C_Zero * 1000.0);
					eCAN_TxMessage.Data4 = 0;
					break;

				case 11:
					eCAN_TxMessage.Data1 = (int)(AC_Line_A_Offset * 1000.0);
					eCAN_TxMessage.Data2 = (int)(AC_Line_B_Offset * 1000.0);
					eCAN_TxMessage.Data3 = (int)(AC_Line_C_Offset * 1000.0);
					eCAN_TxMessage.Data4 = 0;
					break;

				default:
					eCAN_TxMessage.Data1 = (int)(100.0 * speed_control.S_Control_Ki);
					eCAN_TxMessage.Data2 = (int)(100.0 * speed_control.S_Control_Kp);
					eCAN_TxMessage.Data3 = (int)(OperationData.GT_SpeedCommand);
					eCAN_TxMessage.Data4 = GT_SpeedCtrl_Cnt;
					break;

			}
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 18;
			break;

		case 15:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_SystemDebugData2;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(Inverter_SysControl.GT_IDout_error * 1000);
			eCAN_TxMessage.Data2 = (int)(Inverter_SysControl.GT_ID_error_int * 1000);
			eCAN_TxMessage.Data3 = (int)(Inverter_SysControl.GT_ID_error_int_old * 1000);
			eCAN_TxMessage.Data4 = (int)(Inverter_SysControl.GT_ID_out * 1000);
			//eCAN_TxMessage.Data1 = (int)Boost_Control.DCBUS_cmd;
			//eCAN_TxMessage.Data2 = DC_bus_OV_setting; 
			//eCAN_TxMessage.Data3 = Brake_ON_V_setting;
			//eCAN_TxMessage.Data4 = 0;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 16;
			break;

		case 16:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_SystemDebugData3;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(Inverter_SysControl.GT_IQout_error * 1000);
			eCAN_TxMessage.Data2 = (int)(Inverter_SysControl.GT_IQ_error_int *1000);
			eCAN_TxMessage.Data3 = (int)(Inverter_SysControl.GT_IQ_error_int_old * 1000);
			eCAN_TxMessage.Data4 = (int)(Inverter_SysControl.GT_IQ_out * 1000);
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 17;
			break;

		case 17:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_SystemDebugData4;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = (int)(100.0 * Frequency_fpga_ch[0]);
			eCAN_TxMessage.Data2 = (int)(100.0 * Frequency_fpga_ch[1]);
			eCAN_TxMessage.Data3 = (int)(100.0 * Frequency_fpga_ch[2]);
			eCAN_TxMessage.Data4 = PLL_Data.PLL_State;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 18;
			break;

		case 18:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_HarmonicCtrlData;
			eCAN_TxMessage.Data_Length = 8;
			if (Harmonic_TuneCtrl == 2) {
				eCAN_TxMessage.Data1 = (int)(1000.0*GT_2nd_harm_ctrl.I_D_Fil);
				eCAN_TxMessage.Data2 = (int)(1000.0*GT_2nd_harm_ctrl.I_Q_Fil);
				eCAN_TxMessage.Data3 = (int)(1000.0*GT_2nd_harm_ctrl.I_D_com);
				eCAN_TxMessage.Data4 = (int)(1000.0*GT_2nd_harm_ctrl.I_Q_com);
			} else {
				eCAN_TxMessage.Data1 = (int)(1000.0*GT_5th_harm_ctrl.I_D_Fil);
				eCAN_TxMessage.Data2 = (int)(1000.0*GT_5th_harm_ctrl.I_Q_Fil);
				eCAN_TxMessage.Data3 = (int)(1000.0*GT_5th_harm_ctrl.I_D_com);
				eCAN_TxMessage.Data4 = (int)(1000.0*GT_5th_harm_ctrl.I_Q_com);
			}
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 19;
			break;

		case 19:
			eCAN_TxMessage.eCAN_ID = Software_Faults_Warnings;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = system.SW_faults;
			eCAN_TxMessage.Data2 = system.SW_warning;
			eCAN_TxMessage.Data3 = iGrid_Tie.Trip;
			eCAN_DataTx(&eCAN_TxMessage);
			itempCount = 20;
			break;

		case 20:
			if (system.state == STANDBY_MODE || system.state == SYS_OFF || system.state == FAULT_CONDITION) {
				SWData_Report();
			}
			itempCount = 21;
			break;

		case 21:
			if (system.state == SYS_OFF) {
				SystemSetting_Report();
			}
			itempCount = 0;
			break;

		default:	// unknown status
			// do nothing
			itempCount = 0;
			break;

	}

}

//_________________________________________________________________________________
//
// Type:	function
// Name:	System setting Report
//_________________________________________________________________________________
void SystemSetting_Report() {
static int iSettingReport_count = 0;

	switch (iSettingReport_count) {
		case 0:
	        eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_VDCBUS_Echo;
	        eCAN_TxMessage.Data_Length = 8;
	        eCAN_TxMessage.Data1 = (int)Boost_Control.DCBUS_cmd;
	        eCAN_TxMessage.Data2 = Boost_Control.DCBUS_I_Limit_x10 / 10;
	        eCAN_TxMessage.Data3 = Boost_Control.DCBUS_OverCurrent_x10 / 10;
	        eCAN_TxMessage.Data4 = Generator_Current_Limit;
			eCAN_DataTx(&eCAN_TxMessage);

			iSettingReport_count = 1;
			break;

		case 1:
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Vout_Echo;
        	eCAN_TxMessage.Data_Length = 2;
        	eCAN_TxMessage.Data1 = (int)OperationData.VoltageCommand;
			eCAN_DataTx(&eCAN_TxMessage);

			iSettingReport_count = 2;
			break;

		case 2:
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Iout_Echo;
        	eCAN_TxMessage.Data_Length = 4;
        	eCAN_TxMessage.Data1 = (int)OperationData.ID_out_cmd;
        	eCAN_TxMessage.Data2 = (int)OperationData.IQ_out_cmd;
        	eCAN_DataTx(&eCAN_TxMessage);
	
			iSettingReport_count = 3;
			break;
	
		case 3:
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_CmdSysParam_Phase_Echo;
        	eCAN_TxMessage.Data_Length = 2;
        	eCAN_TxMessage.Data1 = (int)PLL_Data.Q_command;
        	//eCAN_TxMessage.Data1 = PLL_Data.Phase_Offset;
        	eCAN_DataTx(&eCAN_TxMessage);
	
			iSettingReport_count = 4;
			break;
	
		case 4:
        	eCAN_TxMessage.eCAN_ID = eCANMsgID_GridTieSpeedCtrl_Echo;
        	eCAN_TxMessage.Data_Length = 4;
        	eCAN_TxMessage.Data1 = OperationData.int_GT_SpeedCommand;
        	eCAN_TxMessage.Data2 = OperationData.GT_SpeedControl;
        	eCAN_DataTx(&eCAN_TxMessage);
	
			iSettingReport_count = 16;
			break;
	
		case 16:
			iSettingReport_count = 0;
			break;

		default:	// unknown status
			// do nothing
			iSettingReport_count = 0;
			break;
	}
}
//_________________________________________________________________________________
//
// Type:	function
// Name:	EEPROM_Report
//_________________________________________________________________________________
void EEPROM_Report(){
static int eepromReportCount=0;

	switch (eepromReportCount) {
		case 0:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_0;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_0.date;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_0.month;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_0.year;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 1;
			break;

		case 1:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_1;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_1.Operation_Frequency;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_1.PWM_Frequency;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_1.spare;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 2;
			break;

		case 2:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_2;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_2.DCBUS_Setting;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_2.DC_OV_Setting;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_2.BRK_Setting;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 3;
			break;

		case 3:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_3;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_3.AI_sensitivity;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_3.AI_trip_time;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_3.AI_Q_inject;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 4;
			break;

		case 4:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_4;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_4.V_ab_Cal_x10k;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_4.V_bc_Cal_x10k;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_4.V_ca_Cal_x10k;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 5;
			break;

		case 5:
			eCAN_TxMessage.eCAN_ID = eCANMsgID_EEPROM_Group_5;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = EEPROM_Data.Group_5.I_a_Cal_x10k;
			eCAN_TxMessage.Data2 = EEPROM_Data.Group_5.I_b_Cal_x10k;
			eCAN_TxMessage.Data3 = EEPROM_Data.Group_5.I_c_Cal_x10k;
			eCAN_DataTx(&eCAN_TxMessage);
			eepromReportCount = 0;
			EERPOM_report = FALSE;
			break;

		default:	// unknown status
			// do nothing
			eepromReportCount = 0;
			break;
	}

}


//_________________________________________________________________________________
//
// Type:	function
// Name:	Software date code Report
//_________________________________________________________________________________
void SWData_Report() {
			eCAN_TxMessage.eCAN_ID = eCANMsgID_SoftwareDateCode;
			eCAN_TxMessage.Data_Length = 8;
			eCAN_TxMessage.Data1 = 3;		 // Month
			eCAN_TxMessage.Data2 = 17;		 // Date	3/17/2015 Initial check-in to gitHub
			eCAN_TxMessage.Data3 = 2015;	 // Year
			eCAN_TxMessage.Data4 = 101;
			eCAN_DataTx(&eCAN_TxMessage);
			DELAY_US(5000L);							// delay 5msec

			eCAN_TxMessage.eCAN_ID = eCANMsgID_FpgaDateCode;
			eCAN_TxMessage.Data_Length = 6;
			eCAN_TxMessage.Data1 = FPGA_r_data[1];	 // Month
			eCAN_TxMessage.Data2 = FPGA_r_data[2];	 // Date
			eCAN_TxMessage.Data3 = FPGA_r_data[3];	 // Year
			//eCAN_TxMessage.Data4 = tempData;
			eCAN_DataTx(&eCAN_TxMessage);
}

/************* End of File ***************/


