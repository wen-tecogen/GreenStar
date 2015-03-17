/**********************************************************************************
// File: main.c
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
// This module contains main().
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/8/12	|  J Wen 	| Original
**********************************************************************************/

#include <xdc/std.h>
#include <xdc/runtime/Log.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Swi.h>

#include "DSP28x_Project.h"     // Device Header file and Examples Include File
#include "EPWM_control.h"       // Device Header file and Examples Include File


/* Semaphore handle defined in task.cfg */
extern const Semaphore_Handle mySem;
extern const Semaphore_Handle systemSema;
extern const Semaphore_Handle lineSema;

/* Counter incremented by timer interrupt */
volatile UInt16 tickCount = 0;
extern Uint16	StateMachineCount;
extern Uint16	EERPOM_report;


//====== Function prototype ======
extern void GPIO_Setup(void);
extern void ECAP_Setup(void);
extern void EPWM_Setup(void);
extern void EPWM1_On(void);
extern void EPWM1_Off(void);
extern void InitECan(void);
extern void I2CA_Init(void);
extern void EEPROM_Report(void);
extern void InitXintf(void);
extern void InitSpi(void);
extern void spiDAC_ref(void);
extern void ADC1_init(void);
extern void ADC2_init(void);
extern void ADC3_init(void);
extern void ADC4_init(void);
extern Uint16 SPIFLASH_Init(void);

//
//  ======== main ========
//
Void main() {
    InitSysCtrl();
	//DELAY_US(10000L);
    GPIO_Setup();
    InitXintf();
    InitECan();
    ECAP_Setup();
    EPWM_Setup();
    I2CA_Init();
	SPIFLASH_Init();
    InitSpi();
    ADC1_init();
    ADC2_init();
    ADC3_init();
    ADC4_init();

	GpioDataRegs.GPBCLEAR.bit.GPIO63 = 1;
	GpioDataRegs.GPBSET.bit.GPIO62 = 1;
	GpioDataRegs.GPBSET.bit.GPIO61 = 1;
	GpioDataRegs.GPBSET.bit.GPIO60 = 1;
	DELAY_US(500000L);
	GpioDataRegs.GPBSET.bit.GPIO63 = 1;
    //======= Variables initialization =======
    StateMachineCount = 0;
    spiDAC_ref();

    //========================================
    // Start BIOS.
    // Begins task scheduling.
    //========================================
    BIOS_start();    /* does not return */
}

//
// ================ myTickFxn ================
//  Timer ISR function that posts a Swi to perform
//  the non-real-time service functions.
//  (myTickFxn: timer tick = 10msec/myTimer count 3000000)
//  myTickFxn: timer tick = 5msec/myTimer count 1500000
//
Void myTickFxn(UArg arg) {
	/* every 5 msec post lineSema */
	Semaphore_post(lineSema);
    /* every 100 timer interrupts post the semaphore */
    if (tickCount == 0) {
        Semaphore_post(systemSema);
    }

    /* 100msec post a semaphore to send a group of data to CAN bus */
    if (tickCount == 10 || tickCount == 30 || tickCount == 50 || tickCount == 70 || tickCount == 90) {
        Semaphore_post(mySem);
	
    }
    
    /* 50msec post a semaphore to send a group of data to CAN bus */
    //if (tickCount == 5 || tickCount == 15 || tickCount == 25 || tickCount == 35 || tickCount == 45 ||
    //    tickCount == 55 || tickCount == 65 || tickCount == 75 || tickCount == 85 || tickCount == 95) {
    //    Semaphore_post(mySem);
    //}

    /* increment the counter */
    tickCount += 1;
    if (tickCount == 100) {
    	tickCount = 0;
    }
}

//
// ================ myTaskFxn ================
//  Task function that pends on a semaphore until 10 ticks have
//  expired.
//
//Void myTaskFxn(Void) {
//    /*
//     * Do this forever
//     */
//    while (TRUE) {
//        /* 
//         * Pend on "mySem" until the timer ISR says
//         * its time to do something.
//         */ 
//        Semaphore_pend(mySem, BIOS_WAIT_FOREVER);
//
//    	//GpioDataRegs.GPBTOGGLE.bit.GPIO63 = 1;		// GPIO63 : DO1
//        //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
//
//        if (EERPOM_report == TRUE) {
//        	EEPROM_Report();
//        } else {
//        	Data_Report();
//        }
//    }
//}


/************* End of File ***************/

