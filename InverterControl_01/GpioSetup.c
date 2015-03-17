/**********************************************************************************
// File: GpioSetup.c
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
// This module contains GPIO setup.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/10/12	|  J Wen 	| Original
**********************************************************************************/
// DESCRIPTION:
//
//
//    Configures the 2834x GPIO into two different configurations
//    This code is verbose to illustrate how the GPIO could be setup.
//    In a real application, lines of code can be combined for improved
//    code size and efficiency.
//
//    This example only sets-up the GPIO.. nothing is actually done with
//    the pins after setup.
//
//    In general:
//
//       All pull-up resistors are enabled.  For ePWMs this may not be desired.
//       Input qual for communication ports (eCAN, SPI, SCI, I2C) is asynchronous
//       Input qual for Trip pins (TZ) is asynchronous
//       Input qual for eCAP and eQEP signals is synch to SYSCLKOUT
//       Input qual for some I/O's and interrupts may have a sampling window
//
//
//===================================================================================

#include "DSP28x_Project.h"     // Device Header-file and Examples Include File
//#include "DSP2834x_Device.h"     // DSP2834x Header-file Include File


// Prototype statements for functions found within this file.
void GPIO_Setup(void);

void GPIO_Setup(void) {
   // Basic Pin-out.
   // This basic pin-out includes:
   // PWM1-3, ECAP1, ECAP2, TZ1-TZ4, SPI-A, EQEP1, SCI-A, I2C
   // and a number of I/O pins

   // These can be combined into single statements for improved
   // code efficiency.

   // Enable PWM1-3 on GPIO0-GPIO5
   EALLOW;
   GpioCtrlRegs.GPAPUD.bit.GPIO0 = 0;    // Enable pullup on GPIO0
   GpioCtrlRegs.GPAPUD.bit.GPIO1 = 0;    // Enable pullup on GPIO1
   GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pullup on GPIO2
   GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pullup on GPIO3
   GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pullup on GPIO4
   GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pullup on GPIO5
   GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // GPIO0 = PWM1A
   GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // GPIO1 = PWM1B
   GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // GPIO2 = PWM2A
   GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // GPIO3 = PWM2B
   GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // GPIO4 = PWM3A
   GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // GPIO5 = PWM3B

   // Enable an GPIO output on GPIO6, set it high
   GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pullup on GPIO6
//   GpioDataRegs.GPASET.bit.GPIO6 = 1;    // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // GPIO6 = GPIO6
//   GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;    // GPIO6 = output

   // Enable eCAP1 on GPIO7
   GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pullup on GPIO7
//   GpioCtrlRegs.GPAQSEL1.bit.GPIO7 = 0;  // Synch to SYSCLOUT
   GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // GPIO7 = ECAP2

   // Enable GPIO outputs on GPIO8 - GPIO11, set it high
   GpioCtrlRegs.GPAPUD.bit.GPIO8 = 0;    // Enable pullup on GPIO8
   GpioDataRegs.GPASET.bit.GPIO8 = 1;    // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;   // GPIO8 = GPIO8
   GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;    // GPIO8 = output

   GpioCtrlRegs.GPAPUD.bit.GPIO9 = 0;    // Enable pullup on GPIO9
   GpioDataRegs.GPASET.bit.GPIO9 = 1;    // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 0;   // GPIO9 = GPIO9
   GpioCtrlRegs.GPADIR.bit.GPIO9 = 1;    // GPIO9 = output

   GpioCtrlRegs.GPAPUD.bit.GPIO10 = 0;   // Enable pullup on GPIO10
   //GpioDataRegs.GPASET.bit.GPIO10 = 1; // Load output latch
   GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;  // GPIO10 = GPIO10
   GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;   // GPIO10 = output

   GpioCtrlRegs.GPAPUD.bit.GPIO11 = 0;   // Enable pullup on GPIO11
   GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 0;  // GPIO11 = GPIO11
   GpioCtrlRegs.GPADIR.bit.GPIO11 = 1;   // GPIO11 = output

   // Enable Trip Zone inputs on GPIO12 - GPIO15
   GpioCtrlRegs.GPAPUD.bit.GPIO12 = 0;   // Enable pullup on GPIO12
   GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO13
   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;   // Enable pullup on GPIO14
   GpioCtrlRegs.GPAPUD.bit.GPIO15 = 0;   // Enable pullup on GPIO15
   GpioCtrlRegs.GPAQSEL1.bit.GPIO12 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL1.bit.GPIO13 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL1.bit.GPIO14 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL1.bit.GPIO15 = 3; // asynch input
   GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 1;  // GPIO12 = TZ1
   GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 1;  // GPIO13 = TZ2
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 1;  // GPIO14 = TZ3
   GpioCtrlRegs.GPAMUX1.bit.GPIO15 = 1;  // GPIO15 = TZ4

   // Enable SPI-A on GPIO16 - GPIO19
   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pullup on GPIO16
   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pullup on GPIO17
   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pullup on GPIO18
   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pullup on GPIO19
   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // asynch input
   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // asynch input
   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1;  // GPIO16 = SPICLKA
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1;  // GPIO17 = SPIS0MIA
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1;  // GPIO18 = SPICLKA
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1;  // GPIO19 = SPISTEA

   // Enable EQEP1 on GPIO20 - GPIO23
   GpioCtrlRegs.GPAPUD.bit.GPIO20 = 0;   // Enable pullup on GPIO20
   GpioCtrlRegs.GPAPUD.bit.GPIO21 = 0;   // Enable pullup on GPIO21
   //GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO22
   //GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO23
   GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Synch to SYSCLKOUT
   GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 3; // Asynch input for CANRX
   //GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0; // Synch to SYSCLKOUT
   //GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0; // Synch to SYSCLKOUT
   GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 3;  // GPIO20 = CANTX
   GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 3;  // GPIO21 = CANRX
   //GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;  // GPIO22 = EQEP1S
   //GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;  // GPIO23 = EQEP1I

   // Enable eCAP1 on GPIO24
   GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO24
   GpioCtrlRegs.GPAQSEL2.bit.GPIO24 = 0; // Synch to SYSCLKOUT
   GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // GPIO24 = GPIO

   // Set input qualification period for GPIO25 & GPIO26
   GpioCtrlRegs.GPACTRL.bit.QUALPRD3=1;  // Qual period = SYSCLKOUT/2
   GpioCtrlRegs.GPAQSEL2.bit.GPIO25=2;   // 6 samples
   GpioCtrlRegs.GPAQSEL2.bit.GPIO26=2;   // 6 samples

   // Make GPIO25 the input source for Xint1
   GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // GPIO25 = GPIO25
   GpioCtrlRegs.GPADIR.bit.GPIO25 = 0;   // GPIO25 = input
   GpioIntRegs.GPIOXINT1SEL.all = 25;    // Xint1 connected to GPIO25

   // Make GPIO26 the input source for XINT2
   GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;  // GPIO26 = GPIO26
   GpioCtrlRegs.GPADIR.bit.GPIO26 = 0;   // GPIO26 = input
   GpioIntRegs.GPIOXINT2SEL.all = 26;    // XINT2 connected to GPIO26

   // Make GPIO27 wake up from HALT/STANDBY Low Power Modes
   GpioCtrlRegs.GPAMUX2.bit.GPIO27 = 0;  // GPIO27 = GPIO27
   GpioCtrlRegs.GPADIR.bit.GPIO27 = 0;   // GPIO27 = input
   GpioIntRegs.GPIOLPMSEL.bit.GPIO27=1;  // GPIO27 will wake the device
   SysCtrlRegs.LPMCR0.bit.QUALSTDBY=2;   // Qualify GPIO27 by 2 OSCCLK
                                         // cycles before waking the device
                                         // from STANDBY

   // Enable SCI-A on GPIO28 - GPIO29
//   GpioCtrlRegs.GPAPUD.bit.GPIO28 = 0;   // Enable pullup on GPIO28
//   GpioCtrlRegs.GPAQSEL2.bit.GPIO28 = 3; // Asynch input
//   GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;  // GPIO28 = SCIRXDA
//   GpioCtrlRegs.GPAPUD.bit.GPIO29 = 0;   // Enable pullup on GPIO29
//  GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;  // GPIO29 = SCITXDA

   // Enable I2C-A on GPIO32 - GPIO33
   GpioCtrlRegs.GPBPUD.bit.GPIO32 = 0;   // Enable pullup on GPIO32
   GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 1;  // GPIO32 = SDAA
   GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
   GpioCtrlRegs.GPBPUD.bit.GPIO33 = 0;   // Enable pullup on GPIO33
   GpioCtrlRegs.GPBQSEL1.bit.GPIO33 = 3; // Asynch input
   GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 1;  // GPIO33 = SCLA

   // Make GPIO34/35 an output
//   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  // GPIO34 = GPIO34
//   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;   // GPIO34 = output
//   GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;

   // Set GPIO29 - GPIO31 as output pins fot testing
   GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 0;  // GPIO30
   GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;   // GPIO30 = output
   GpioDataRegs.GPACLEAR.bit.GPIO29 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;  // GPIO30
   GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;   // GPIO30 = output
   GpioDataRegs.GPACLEAR.bit.GPIO30 = 0;
   GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;  // GPIO30
   GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;   // GPIO30 = output
   GpioDataRegs.GPACLEAR.bit.GPIO31 = 0;


   GpioCtrlRegs.GPBMUX1.bit.GPIO35 = 0;  // GPIO35 = GPIO35
   GpioCtrlRegs.GPBDIR.bit.GPIO35 = 1;   // GPIO35 = output
   GpioDataRegs.GPBCLEAR.bit.GPIO35 = 1;

   // Make GPIO60/61/62/63 an output
   GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;  // GPIO60 = GPIO60	--> DO_4
   GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;   // GPIO60 = output
   GpioDataRegs.GPBSET.bit.GPIO60 = 1;

   GpioCtrlRegs.GPBMUX2.bit.GPIO61 = 0;  // GPIO61 = GPIO61 --> DO_3
   GpioCtrlRegs.GPBDIR.bit.GPIO61 = 1;   // GPIO61 = output
   GpioDataRegs.GPBSET.bit.GPIO61 = 1;

   GpioCtrlRegs.GPBMUX2.bit.GPIO62 = 0;  // GPIO62 = GPIO62 --> DO_2 Active low
   GpioCtrlRegs.GPBDIR.bit.GPIO62 = 1;   // GPIO62 = output
   GpioDataRegs.GPBSET.bit.GPIO62 = 1;

   GpioCtrlRegs.GPBMUX2.bit.GPIO63 = 0;  // GPIO63 = GPIO63 --> DO_1 Active low
   GpioCtrlRegs.GPBDIR.bit.GPIO63 = 1;   // GPIO63 = output
   GpioDataRegs.GPBSET.bit.GPIO63 = 1;

   // Make GPIO22/23/34/14//52/53/58/59 an input
   GpioCtrlRegs.GPAPUD.bit.GPIO22 = 0;   // Enable pullup on GPIO22
   GpioCtrlRegs.GPAPUD.bit.GPIO23 = 0;   // Enable pullup on GPIO23
   GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 0;  // GPIO22 
   GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 0;  // GPIO23
   GpioCtrlRegs.GPADIR.bit.GPIO22 = 0;	 // GPIO22 = input --> DI_1_EMS
   GpioCtrlRegs.GPADIR.bit.GPIO23 = 0;	 // GPIO23 = input --> DI_2

   GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;  // GPIO34 = GPIO34
   GpioCtrlRegs.GPBDIR.bit.GPIO34 = 0;   // GPIO34 = input --> DI_3
   GpioCtrlRegs.GPBPUD.bit.GPIO34 = 0;	 // Enable pullup on GPIO34

//   GpioCtrlRegs.GPCMUX2.bit.GPIO87 = 0;  // GPIO87 = GPIO87
//   GpioCtrlRegs.GPCDIR.bit.GPIO87 = 0;   // GPIO87 = input --> DI_3
//   GpioCtrlRegs.GPCPUD.bit.GPIO87 = 0;	 // Enable pullup on GPIO87

   GpioCtrlRegs.GPAPUD.bit.GPIO14 = 0;	 // Enable pullup on GPIO14
   GpioCtrlRegs.GPAMUX1.bit.GPIO14 = 0;  // GPIO14 = GPIO14
   GpioCtrlRegs.GPADIR.bit.GPIO14 = 0;   // GPIO14 = input --> DI_4

   GpioCtrlRegs.GPBPUD.bit.GPIO52 = 0;	 // Enable pullup on GPIO52
   GpioCtrlRegs.GPBMUX2.bit.GPIO52 = 0;  // GPIO52 = GPIO52
   GpioCtrlRegs.GPBDIR.bit.GPIO52 = 0;   // GPIO52 = input --> DI_5

   GpioCtrlRegs.GPBPUD.bit.GPIO53 = 0;	 // Enable pullup on GPIO53
   GpioCtrlRegs.GPBMUX2.bit.GPIO53 = 0;  // GPIO53 = GPIO53
   GpioCtrlRegs.GPBDIR.bit.GPIO53 = 0;   // GPIO53 = input --> DI_6

   GpioCtrlRegs.GPBPUD.bit.GPIO58 = 0;	 // Enable pullup on GPIO58
   GpioCtrlRegs.GPBMUX2.bit.GPIO58 = 0;  // GPIO58 = GPIO58
   GpioCtrlRegs.GPBDIR.bit.GPIO58 = 0;   // GPIO58 = input --> DI_7

   GpioCtrlRegs.GPBPUD.bit.GPIO59 = 0;	 // Enable pullup on GPIO59
   GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;  // GPIO59 = GPIO59
   GpioCtrlRegs.GPBDIR.bit.GPIO59 = 0;   // GPIO59 = input --> DI_8

   EDIS;
}

//===========================================================================
// End of file
//===========================================================================

