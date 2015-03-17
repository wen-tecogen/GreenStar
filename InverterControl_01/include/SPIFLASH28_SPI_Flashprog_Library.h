// TI File $Revision:: 3    $
//###########################################################################
//
// FILE:  SPIFLASH28_Flashprog_Library.h
//
// TITLE: '28x SPI FLASH utility library
//
// DESCRIPTION:
//
//       This file should be included in any project that uses the 28x SPI
//       FLASH utility library.
//
//       This program uses the DSP2834x header files that are included in the
//       C/C++ Header Files and Peripheral Examples in C (SPRC876) which can
//       be downloaded from the TI website.  This program currently uses V1.00
//       of SPRC876 and the headerfiles used are included with this project.
//
//       Support is included for the Atmel AT25F1024A Flash
//
//###########################################################################
//
//  Ver | dd mmm yyyy  | Who  | Description of changes
// =====|==============|======|===============================================
// 1.00 | 02 Jun 2009  | TI   | Initial Release
// 1.01 | 22 Nov 2013  | JW   | Import for my project
//###########################################################################

#ifndef SPIFLASH28_FLASHPROG_LIBRARY_H
#define SPIFLASH28_FLASHPROG_LIBRARY_H

// Include the C/C++ header file main include file.  This is part of the
// C/C++ Header Files and Peripheral Examples in C (SPRC097) which can
// be downloaded from the TI website.
#include "DSP2834x_Device.h"

#define AT25DF041A   1         // Uses the AT25DF041A 4M bits Flash


// 2834x SPI Configuration Settings
// These are set in the SPIFLASH_init() function
//
// SYSCLKOUT = 300Mhz
// LSPCLK = 300Mhz/4 = 75Mhz
// BR = 75Mhz/(0x0015+1)= 3.4 MHz
#define LSPCLK_DIV  0x0002    // Low speed clk = SYSCLKOUT/2
#define SPI_BAUD    0x0015    // SPI Baud Rate


// DO NOT MODIFY THIS LINE.
#define DELAY_US(A)  DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

/*---------------------------------------------------------------------------
 Serial FLASH Commands and FLASH Parameters
---------------------------------------------------------------------------*/
#if AT25DF041A

// Serial Flash commands

#define   FLASH_SET_WRITE_ENABLE_LATCH      0x0006
#define   FLASH_RESET_WRITE_ENABLE_LATCH    0x0004
#define   FLASH_READ_STATUS_REG             0x0005
#define   FLASH_WRITE_STATUS_REG            0x0001
#define   FLASH_READ_MEMORY                 0x0003
#define   FLASH_WRITE_MEMORY                0x0002
#define   FLASH_CHIP_ERASE                  0x0060	//for AT25DF041A

#define   FLASH_SET_CS_HIGH  GpioDataRegs.GPASET.bit.GPIO19=1          //SPISTEA CSn Low
#define   FLASH_SET_CS_LOW   GpioDataRegs.GPACLEAR.bit.GPIO19=1        //SPISTEA CSn Low

// Serial Flash CS timing parameters (in us)
#define   FLASH_CS_HOLD          0.250L
#define   FLASH_CS_HIGH          0.250L
#define   FLASH_CS_SETUP         0.250L

// Serial Flash parameters
//#define   FLASH_TOTAL_BYTES      131072
#define   FLASH_TOTAL_BYTES      524288					// 4M bits
#define   FLASH_PAGE_SIZE_BYTES  256                      // Size in bytes per page
#define   FLASH_PAGE_SIZE_WORDS  FLASH_PAGE_SIZE_BYTES/2    // Size in words per page
#define   FLASH_TOTAL_PAGES      FLASH_TOTAL_BYTES/FLASH_PAGE_SIZE_BYTES
#define   FLASH_LAST_ADDR        0x7FFFF				// 4M bits 524K
                                                                                                               //  = 128bytes * 1word/2bytes
#endif    // -- AT25DF041A

/*---------------------------------------------------------------------------
 '28 Serial FLASH API Status Messages

 The following status values are returned from the SPI FLASH API to the
 calling program.  These can be used to determine if the FLASH (FLASH Programmer)
 API function passed or failed.
---------------------------------------------------------------------------*/
 // Operation passed, no errors were flagged
#define FLASH_STATUS_SUCCESS               0

// The CSM is preventing the function from performing its operation
#define FLASH_STATUS_FAIL_INIT            10

// ---- Erase Specific errors ----
#define FLASH_STATUS_FAIL_ERASE           20

// ---- Program Specific errors ----
#define FLASH_STATUS_FAIL_PROGRAM         30

// ---- Verify Specific errors ----
#define FLASH_STATUS_FAIL_VERIFY          40

// Busy is set by each API function before it determines
// a pass or fail condition for that operation.
// The calling function will will not receive this
// status condition back from the API

#define FLASH_STATUS_BUSY                999

/*---------------------------------------------------------------------------
   Serial FLASH Status Register
---------------------------------------------------------------------------*/

#define FLASH_RDY    0x0001  // Bit 0
#define FLASH_WEN    0x0002  // Bit 1
#define FLASH_BP0    0x0004  // Bit 2
#define FLASH_BP1    0x0008  // Bit 3
#define FLASH_WPEN   0x0080  // Bit 8

/*---------------------------------------------------------------------------
   '28 SPI FLASH Programmer Utility Function Prototypes
---------------------------------------------------------------------------*/

extern Uint16 SPIFLASH_WritePage(Uint32 FlashAddr, Uint16 *BufAddr, Uint16 Length);
extern Uint16 SPIFLASH_Verify(Uint32 FlashAddr, Uint16 *BufAddr, Uint16 Length);
extern Uint16 SPIFLASH_Erase(void);
extern Uint16 SPIFLASH_Read16bitWord(Uint32 FlashAddr);
extern Uint16 SPIFLASH_ReadStatReg(void);
extern void SPIFLASH_WriteStatReg(Uint16 StatReg);
extern Uint16 SPIFLASH_Init(void);


extern Uint16 SPIFLASH_WriteEnable(void);
extern void   SPIFLASH_ChipSelectHigh(void);
extern void   SPIFLASH_ChipSelectLow(void);
extern void   SPIFLASH_SendCommand(Uint16 Command);
extern Uint16 SPIFLASH_Send16bitWord(Uint16 Value);
extern void   SPIFLASH_SendFlashAddr(Uint32 FlashAddr);
extern void   DSP28x_usDelay(Uint32 Count);

/*---------------------------------------------------------------------------
* Public function definitions used by SDFlash.  These functions are used
* by SDFlash to interface to the F281x flash API.
*---------------------------------------------------------------------------*/

void PRG_init(void);       // Initalize system for programming
void PRG_program(void);    // Program a block
void PRG_erase(void);      // Erase sectors
void PRG_verify(void);     // Verify a block
void PRG_lock(void);       // Lock the Code Security Module
void PRG_unlock(void);     // Unlock the Code Security Module
void PRG_exit(void);

/*---------------------------------------------------------------------------
* Arguments used by the flash programmer.  These values are generally
* filled in by the flash programmer and read by the flash algos to
* erase/program/verify the flash.
*---------------------------------------------------------------------------*/
#pragma DATA_SECTION(Buffer,"Buffer");
Uint16  Buffer[FLASH_PAGE_SIZE_WORDS];

volatile Uint16  PRG_options; //   Used to pass the sector mask to the
                              //   algo's.  Used for erase/clear.
         Uint16 *PRG_bufaddr; //   Address of buffer for flash/program data
volatile Uint16  PRG_bufsize; //   Size of program data buffer
volatile Uint16  PRG_devsize; //   Size of programmable device
         Uint32  PRG_paddr;   //   First programming address
volatile Uint16  PRG_page;    //   Programming page
volatile Uint16  PRG_length;  //   Length of programming data
volatile Uint16  PRG_status;  //   Status of programming functions

volatile Uint16  PRG_options;  //   options - algo dependant
volatile Uint16  PRG_options2;
volatile Uint16  PRG_options3;
volatile Uint16  PRG_options4;

/*---- SDFlash User Options will be populated with this value by default ------*/
#define OPTION_BLANK 0x5F1F

// Note: CSM key values are currently not populated by the SDFlash GUI.  This
// will be a future enhancement to the utility.  For now they are populated
// by the included SDFlash28x_CsmKeys.asm file.

extern Uint16 PRG_key0;        //   CSM Key values
extern Uint16 PRG_key1;
extern Uint16 PRG_key2;
extern Uint16 PRG_key3;
extern Uint16 PRG_key4;
extern Uint16 PRG_key5;
extern Uint16 PRG_key6;
extern Uint16 PRG_key7;

#endif // ---- End of SDFLASH_FLASHPROG_WRAPPER_H


// --------- END OF FILE ----------------------------------

