/**********************************************************************************
// File: spi.c
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
// This module contains spi function.
//
// Change History:
//---------------------------------------------------------------------------------
// Date		| Author	| Description
//---------------------------------------------------------------------------------
// 08/22/12	|  J Wen 	| Original
**********************************************************************************/

#include "DSP2834x_Device.h"     // DSP2834x Headerfile Include File
#include "DSP2834x_Examples.h"   // DSP2834x Examples Include File

#include "spi.h"				 // define spi port related parts
#include "adc.h"				 // define spi port related parts

void spiA_xmit(Uint16 a);
void spiA_fifo_init(void);
void spiA_init(void);
void spiA8bit_init(void);

void spiD_xmit(Uint16 a);
void spiD_fifo_init(void);
void spiD_init(void);
void spiDAC_ref(void);

#define	DCBUS_TRIP_VOLTAGE 	900.0/DC_BUS_1_SCALE_FACTOR

//---------------------------------------------------------------------------
// InitSPI:
//---------------------------------------------------------------------------
// This function initializes the SPI(s) to a known state.
//
void InitSpi(void) {
//Uint16 uiTemp;
	//*** Setup SPI D port ***
	InitSpiGpio();
//	spiA_fifo_init();
//	spiA_init();
	spiD_fifo_init();
	spiD_init();

//    while(SpiaRegs.SPIFFRX.bit.RXFFST !=0) {
//    	uiTemp = SpiaRegs.SPIRXBUF;
//    }
	//******* SPI ADC init *******
//	uiTemp = 0x1800;
//	spiA_xmit(uiTemp);
//    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }
//	uiTemp = SpiaRegs.SPIRXBUF;
//	uiTemp = 0x2800;
//	spiA_xmit(uiTemp);
//    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }
//	uiTemp = SpiaRegs.SPIRXBUF;
//	uiTemp = 0x3800;
//	spiA_xmit(uiTemp);

//    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }
//   	uiTemp = SpiaRegs.SPIRXBUF;

}

//---------------------------------------------------------------------------
// Example: InitSpiGpio:
//---------------------------------------------------------------------------
// This function initializes GPIO pins to function as SPI pins
//
// Each GPIO pin can be configured as a GPIO pin or up to 3 different
// peripheral functional pins. By default all pins come up as GPIO
// inputs after reset.
//
// Caution:
// For each SPI peripheral
// Only one GPIO pin should be enabled for SPISOMO operation.
// Only one GPIO pin should be enabled for SPISOMI operation.
// Only one GPIO pin should be enabled for SPICLKA operation.
// Only one GPIO pin should be enabled for SPISTEA operation.
// Comment out other unwanted lines.

void InitSpiGpio() {

   //InitSpiaGpio();
   InitSpidGpio();
}

void InitSpiaGpio() {

   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;   // Enable pull-up on GPIO16 (SPISIMOA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;   // Enable pull-up on GPIO17 (SPISOMIA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;   // Enable pull-up on GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;   // Enable pull-up on GPIO19 (SPISTEA)


    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 0;   // Enable pull-up on GPIO54 (SPISIMOA)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;   // Enable pull-up on GPIO55 (SPISOMIA)
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 0;   // Enable pull-up on GPIO56 (SPICLKA)
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 0;   // Enable pull-up on GPIO57 (SPISTEA)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3; // Asynch input GPIO16 (SPISIMOA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3; // Asynch input GPIO17 (SPISOMIA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3; // Asynch input GPIO18 (SPICLKA)
//    GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3; // Asynch input GPIO19 (SPISTEA)

    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 3; // Asynch input GPIO16 (SPISIMOA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 3; // Asynch input GPIO17 (SPISOMIA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 3; // Asynch input GPIO18 (SPICLKA)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 3; // Asynch input GPIO19 (SPISTEA)


/* Configure SPI-A pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

//    GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // Configure GPIO16 as SPISIMOA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // Configure GPIO17 as SPISOMIA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // Configure GPIO18 as SPICLKA
//    GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 1; // Configure GPIO19 as SPISTEA

    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1; // Configure GPIO54 as SPISIMOA
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1; // Configure GPIO55 as SPISOMIA
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1; // Configure GPIO56 as SPICLKA
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1; // Configure GPIO57 as SPISTEA

    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0;   // Configure GPIO57 as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO57 = 1;	   // GPIO51 = GPIO57 --> SPI DAC CS
    GpioDataRegs.GPBSET.bit.GPIO57 = 1;	   // Set GPIO51 high

    EDIS;
}

void InitSpidGpio()
{
   EALLOW;
/* Enable internal pull-up for the selected pins */
// Pull-ups can be enabled or disabled disabled by the user.
// This will enable the pullups for the specified pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBPUD.bit.GPIO48 = 0;    // Enable pull-up on GPIO48 (SPISIMOD)
    GpioCtrlRegs.GPBPUD.bit.GPIO49 = 0;    // Enable pull-up on GPIO49 (SPISOMID)
    GpioCtrlRegs.GPBPUD.bit.GPIO50 = 0;    // Enable pull-up on GPIO50 (SPICLKD)
    //GpioCtrlRegs.GPBPUD.bit.GPIO51 = 0;    // Enable pull-up on GPIO51 (SPISTED)

/* Set qualification for selected pins to asynch only */
// This will select asynch (no qualification) for the selected pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBQSEL2.bit.GPIO48 = 3;   // Asynch input GPIO48 (SPISIMOD)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO49 = 3;   // Asynch input GPIO49 (SPISOMID)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO50 = 3;   // Asynch input GPIO50 (SPICLKD)
    //GpioCtrlRegs.GPBQSEL2.bit.GPIO51 = 3;   // Asynch input GPIO51 (SPISTED)

/* Configure SPI-D pins using GPIO regs*/
// This specifies which of the possible GPIO pins will be SPI functional pins.
// Comment out other unwanted lines.

    GpioCtrlRegs.GPBMUX2.bit.GPIO48 = 3;   // Configure GPIO48 as SPISIMOD
    GpioCtrlRegs.GPBMUX2.bit.GPIO49 = 3;   // Configure GPIO49 as SPISOMID
    GpioCtrlRegs.GPBMUX2.bit.GPIO50 = 3;   // Configure GPIO50 as SPICLKD

    GpioCtrlRegs.GPBMUX2.bit.GPIO51 = 0;   // Configure GPIO51 as GPIO
    GpioCtrlRegs.GPBDIR.bit.GPIO51 = 1;	   // GPIO51 = GPIO51 --> SPI DAC CS
    GpioDataRegs.GPBSET.bit.GPIO51 = 1;	   // Set GPIO51 high

    EDIS;
}

/*
 *  ======== Initialize SPI A port ========
 */
void spiA_init()
{
	SpiaRegs.SPICCR.all =0x000F;	             // Reset on, rising edge, 16-bit char bits
	SpiaRegs.SPICTL.all =0x000E;    		     // Enable master mode, normal phase,
                                                 // enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x000F;					 // SPI clock = 75MHz/15 = 5MHz (LSPCLK=75MHz)
    SpiaRegs.SPICCR.all =0x008F;		         // Relinquish SPI from Reset
//    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
}

/*
 *  ======== Initialize SPI A port ========
 */
void spiA8bit_init()
{
	SpiaRegs.SPICCR.all =0x0007;	             // Reset on, rising edge, 16-bit char bits
	SpiaRegs.SPICTL.all =0x000E;    		     // Enable master mode, normal phase,
                                                 // enable talk, and SPI int disabled.
	SpiaRegs.SPIBRR =0x000F;					 // SPI clock = 75MHz/15 = 5MHz (LSPCLK=75MHz)
    SpiaRegs.SPICCR.all =0x0087;		         // Relinquish SPI from Reset
//    SpiaRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
}

/*
 *  ======== Initialize SPI D port ========
 */
void spiD_init()
{
	SpidRegs.SPICCR.all =0x000F;	             // Reset on, rising edge, 16-bit char bits
	SpidRegs.SPICTL.all =0x0006;    		     // Enable master mode, normal phase,
                                                 // enable talk, and SPI int disabled.
	SpidRegs.SPIBRR =0x007F;					 // SPI clock = 75MHz/128 = 586KHz (LSPCLK=75MHz)
    SpidRegs.SPICCR.all =0x009F;		         // Relinquish SPI from Reset
    SpidRegs.SPIPRI.bit.FREE = 1;                // Set so breakpoints don't disturb xmission
}

/*
 *  ======== Initialize SPI A port fifo ========
 */
void spiA_fifo_init()
{
// Initialize SPI FIFO registers
    SpiaRegs.SPIFFTX.all=0x8000;		//0xE040;
    SpiaRegs.SPIFFRX.all=0x204f;
    SpiaRegs.SPIFFCT.all=0x0;
}

/*
 *  ======== Initialize SPI D port fifo ========
 */
void spiD_fifo_init()
{
// Initialize SPI FIFO registers
    SpidRegs.SPIFFTX.all=0xE040;
    SpidRegs.SPIFFRX.all=0x204f;
    SpidRegs.SPIFFCT.all=0x0;
}

/*
 *  ======== SPI A transmit ========
 */
void spiA_xmit(Uint16 a) {
	GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;	// Set GPIO57 low to select SPI DAC
	//DELAY_US(10L);						// delay
    SpiaRegs.SPITXBUF=a;					// write to DAC
    DELAY_US(3L);							// delay
    while(SpiaRegs.SPISTS.bit.INT_FLAG == 0) { }
    GpioDataRegs.GPBSET.bit.GPIO57 = 1;	   	// Set GPIO57 high
	DELAY_US(10L);							// delay
}

/*
 *  ======== SPI D transmit ========
 */
void spiD_xmit(Uint16 a) {
	GpioDataRegs.GPBCLEAR.bit.GPIO51 = 1;	// Set GPIO51 low to select SPI DAC
	DELAY_US(300L);							// delay
    SpidRegs.SPITXBUF=a;					// write to DAC
    DELAY_US(300L);							// delay
    GpioDataRegs.GPBSET.bit.GPIO51 = 1;	   	// Set GPIO51 high
	DELAY_US(500L);							// delay
}


void spiDAC_ref() {
Uint16	DAC_data;
	// Set channel A voltage as 2.5V for ref
	DAC_data = (Uint16)(4096*2.5/SPI_DAC_5V_VREF);
	DAC_data += SPI_DAC_CH_A;
	spiD_xmit(DAC_data);
	// Set channel B voltage as 2.5+1.5V for ref
	DAC_data = (Uint16)(4096*(2.5+1.5)/SPI_DAC_5V_VREF);
	DAC_data += SPI_DAC_CH_B;
	spiD_xmit(DAC_data);
	// Set channel C voltage as 2.5-1.5V for ref
	DAC_data = (Uint16)(4096*(2.5-1.5)/SPI_DAC_5V_VREF);
	DAC_data += SPI_DAC_CH_C;
	spiD_xmit(DAC_data);
	// Set channel D voltage as DC bus hardware trip point
	DAC_data = (Uint16)(4096*DCBUS_TRIP_VOLTAGE/SPI_DAC_5V_VREF);
	DAC_data += SPI_DAC_CH_D;
	spiD_xmit(DAC_data);

}


//===========================================================================
// End of file.
//===========================================================================
