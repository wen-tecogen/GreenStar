// TI File $Revision:: 3    $
//###########################################################################
//
// FILE:  SPIFLASH28_SPI_Flash_Library.c
//
// TITLE: SPI FLASH Programmer Utility Library
//
// NOTE:  These functions enable programming of a serial FLASH connected to
//        SPI-A of a C2834x device.
//
//        As written, the code assumes an Atmel AT25F1024A serial
//        FLASH is being used.
//
//###########################################################################
//
//  Ver | dd mmm yyyy | Who  | Description of changes
// =====|=============|======|===============================================
//  1.00| 02 Jun 2009 | TI   | Initial Release
//  1.01| 22 Nov 2013 | JW   | Import for my project
//###########################################################################

// Include file for the FLASH programming utility functions.
#include "SPIFLASH28_SPI_Flashprog_Library.h"

// Include the C/C++ header file main include file.  This is part of the
// C/C++ Header Files and Peripheral Examples in C (SPRC097) which can
// be downloaded from the TI website.
#include "DSP2834x_Device.h"

void FlashMemoryInit(void);

Uint16	Flash_Erase_Status;

//****************************************************************
// Function: SPIFLASH_Init()
//
// This function initalizes the SPI for communication with the
// serial FLASH device.
//
// Parameters:
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------
Uint16 SPIFLASH_Init(void)
{
   EALLOW;
   //SysCtrlRegs.PCLKCR0.bit.SPIAENCLK=1;  // Enable the SPI clock
   //SysCtrlRegs.LOSPCP.all = LSPCLK_DIV;

   GpioCtrlRegs.GPAPUD.bit.GPIO16 = 0;  // Enable pull-up
   GpioCtrlRegs.GPAPUD.bit.GPIO17 = 0;  // Enable pull-up
   GpioCtrlRegs.GPAPUD.bit.GPIO18 = 0;  // Enable pull-up
   GpioCtrlRegs.GPAPUD.bit.GPIO19 = 0;  // Enable pull-up

   GpioCtrlRegs.GPAQSEL2.bit.GPIO16 = 3;// Asynch
   GpioCtrlRegs.GPAQSEL2.bit.GPIO17 = 3;// Asynch
   GpioCtrlRegs.GPAQSEL2.bit.GPIO18 = 3;// Asynch
   GpioCtrlRegs.GPAQSEL2.bit.GPIO19 = 3;// Asynch

   GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 1; // SPISIMOA
   GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 1; // SPISOMIA
   GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 1; // SPICLKA
   GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0; // CSn
   GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;  // CSn

   EDIS;


   SpiaRegs.SPIFFTX.all = 0x8000;
   SpiaRegs.SPICCR.all = 0x0007;		 // 0x0007
   SpiaRegs.SPICTL.all = 0x000E;
   SpiaRegs.SPIBRR = 0x007F;			 // SPI clock = 75MHz/128 = 586KHz (LSPCLK=75MHz)
   SpiaRegs.SPICCR.all = 0x0087;		 // 0x0087
   FLASH_SET_CS_HIGH;                    //Set CSn high

   return FLASH_STATUS_SUCCESS;
}


//****************************************************************
// Function: SPIFLASH_WritePage(Uint16 FlashAddr, Uint16 *BufAddr, Uint16 Length)
//
// This function writes a page of data to the FLASH
//
// Parameters:
//
//    FlashAddr           First address of the page to be
//                         written to in the FLASH.
//    *BufAddr             Pointer to the buffer of data to be
//                         programmed.
//    Length               Number of values in the buffer. If this
//                         is less then a full page, then missing
//                         data will be read from the FLASH to fill
//                         in a full page.
//
// Return Value:
//
//    STATUS_SUCCESS        Write completed - use the verify function
//                          to check the data for correctness.
//
//    STATUS_FAIL_PROGRAM   Was unable to enable programming.
//
// Notes: ATMEL 25F1024A supports page writes
//
//-------------------------------------------------------------------
Uint16 SPIFLASH_WritePage(Uint32 FlashAddr, Uint16 *BufAddr, Uint16 Length)
{
    Uint16 WordCount;
    Uint16 FLASH_StatusReg;
    Uint16 Status;
    Uint32 temp;

    if(FlashAddr > FLASH_LAST_ADDR-FLASH_PAGE_SIZE_BYTES)
    {
       return FLASH_STATUS_FAIL_PROGRAM;
    }

    else if(Length > FLASH_PAGE_SIZE_WORDS)
    {
       return FLASH_STATUS_FAIL_PROGRAM;
    }

    else if(Length < FLASH_PAGE_SIZE_WORDS)
    {
       temp = FlashAddr;
       // Fill remainder of page with data from the FLASH
       FlashAddr = FlashAddr + (Length*2);
       for(WordCount = Length; WordCount <= FLASH_PAGE_SIZE_WORDS; WordCount++)
       {
           BufAddr[WordCount] = SPIFLASH_Read16bitWord(FlashAddr);
           FlashAddr+=2;
       }
       FlashAddr = temp;

    }

    // Clear protection bits in the status reg
    SPIFLASH_WriteStatReg(0x0000);

    Status = SPIFLASH_WriteEnable();
    if(Status != FLASH_STATUS_SUCCESS) return FLASH_STATUS_FAIL_PROGRAM;

    SPIFLASH_ChipSelectLow();
    SPIFLASH_SendCommand(FLASH_WRITE_MEMORY);
    SPIFLASH_SendFlashAddr(FlashAddr);

    for(WordCount = 0; WordCount < FLASH_PAGE_SIZE_WORDS; WordCount++)
    {
        SPIFLASH_Send16bitWord(*BufAddr);
        BufAddr++;
    }

    SPIFLASH_ChipSelectHigh();

    // Wait for write to complete
    do
    {
        FLASH_StatusReg = SPIFLASH_ReadStatReg();
        temp = FLASH_StatusReg & FLASH_RDY;
    } while( (FLASH_StatusReg & FLASH_RDY) != 0 );

    return FLASH_STATUS_SUCCESS;

}


//****************************************************************
// Function: SPIFLASH_Verify(Uint16 FlashAddr, Uint16 *BufAddr, Uint16 Length)
//
// This function verifys a data in the FLASH against a golden buffer.
//
// Parameters:
//    FlashAddr     First address of the page to be
//                   written to in the FLASH.
//    *BufAddr       Pointer to the buffer of golden data to be
//                   compared.
//     Length        Number of values to compare.
//
// Return Value:
//
//     STATUS_SUCCESS           Write completed - use the verify function
//                              to check the data for correctness.
//
//     FLASH_STATUS_FAIL_VERIFY   At least one word did not match.
//
// Notes: This function compares 16-bit values.
//-------------------------------------------------------------------
Uint16 SPIFLASH_Verify(Uint32 FlashAddr, Uint16 *BufAddr, Uint16 Length)
{
    Uint16 WordCount;
    Uint16 CheckVal;

    for(WordCount = 0; WordCount < Length; WordCount++)
    {
        CheckVal = SPIFLASH_Read16bitWord(FlashAddr);
        if( CheckVal != *BufAddr) return FLASH_STATUS_FAIL_VERIFY;
        FlashAddr +=2;
        BufAddr++;
    }

    return FLASH_STATUS_SUCCESS;

}

//****************************************************************
// Function: SPIFLASH_Read16bitWord(Uint16 FlashAddr)
//
// This function reads a 16-bit value out of the
// serial FLASH device.
//
// Parameters:
//
//     FlashAddr  FLASH Address.
//
// Return Value:
//
//          Uinsigned 16-bit word from the FLASH.  The value is read
//          from the FLASH in order LSB:MSB and returned to the calling
//          function as MSB:LSB.
//
// Notes:
//-------------------------------------------------------------------
Uint16 SPIFLASH_Read16bitWord(Uint32 FlashAddr)
{
	Uint16 WordVal = 0;
    SPIFLASH_ChipSelectLow();
	SPIFLASH_SendCommand(FLASH_READ_MEMORY);
 	SPIFLASH_SendFlashAddr(FlashAddr);
 	WordVal = SPIFLASH_Send16bitWord(0x0000);
    SPIFLASH_ChipSelectHigh();
    return WordVal;
}

//****************************************************************
// Function: SPIFLASH_ReadStatReg()
//
// This function reads the FLASH's status register
//
// Parameters:
//
// Return Value:
//
//          0:8-bitStatusReg
//
// Notes:
//-------------------------------------------------------------------

Uint16 SPIFLASH_ReadStatReg()
{
    Uint16 StatReg;

    SPIFLASH_ChipSelectLow();
    SPIFLASH_SendCommand(FLASH_READ_STATUS_REG);

    // Force the value of the status register to be shifted out
    DSP28x_usDelay(0.25);
	StatReg = 0x00FF;
	SpiaRegs.SPITXBUF = 0x0000;
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
	StatReg = SpiaRegs.SPIRXBUF;

    SPIFLASH_ChipSelectHigh();

    return StatReg;
}

//****************************************************************
// Function: SPIFLASH_WriteStatReg(Uint16 StatReg)
//
// This function writes to the FLASH's status register
//
// Parameters:
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------

void SPIFLASH_WriteStatReg(Uint16 StatReg)
{
    Uint16 FLASH_StatusReg;

    SPIFLASH_WriteEnable();

    SPIFLASH_ChipSelectLow();
    SPIFLASH_SendCommand(FLASH_WRITE_STATUS_REG);

	SpiaRegs.SPITXBUF = StatReg << 8;            //Send the Status Register value
    //DSP28x_usDelay(FLASH_CS_SETUP);
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
	SpiaRegs.SPIRXBUF;      	                 //Clear RX buffer

    SPIFLASH_ChipSelectHigh();

    // Wait for write to complete
    do
    {
        FLASH_StatusReg = SPIFLASH_ReadStatReg();
    } while( (FLASH_StatusReg & FLASH_RDY) != 0 );


    return;
}



//****************************************************************
// Function: void SPIFLASH_ChipSelectHigh(void)
//
// This function sets the chip select signal (GPIO19) high
//
// Parameters:
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------

void SPIFLASH_ChipSelectHigh(void)
{
    DSP28x_usDelay(FLASH_CS_HOLD);
    FLASH_SET_CS_HIGH;                             //Set CSn HIGH
    DSP28x_usDelay(FLASH_CS_HIGH);
}


//****************************************************************
// Function: void SPIFLASH_ChipSelectLow(void)
//
// This function sets the chip select signal (GPIO19) low
//
// Parameters:
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------
void SPIFLASH_ChipSelectLow(void)
{
    FLASH_SET_CS_LOW;                             //Set CSn LOW
    DSP28x_usDelay(FLASH_CS_SETUP);
}


//****************************************************************
// Function: void SPIFLASH_SendCommand(Uint16 Command)
//
// This function sends a command to the FLASH.
//
// Parameters:
//
//              Command        The 8-bit command is passed in the LSByte
//                             of the parameter 'Command'.
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------
void SPIFLASH_SendCommand(Uint16 Command)
{
	SpiaRegs.SPITXBUF = Command << 8;            //Send an FLASH Command
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
    //DSP28x_usDelay(FLASH_CS_SETUP);
	SpiaRegs.SPIRXBUF;	                         //Clear RX buffer
}

//****************************************************************
// Function: void SPIFLASH_Send16bitWord(Uint16 Word)
//
// This function sets the chip select signal (GPIO19) low
//
// Parameters:
//
//              Word             For a write command, Word is the 16-bit value
//                               to be stored in the FLASH.
//                               The Word is passed to the function as MSB:LSB and
//                               then stored in the FLASH as LSB:MSB as expected by
//                               the boot ROM.
//
//                               For a read command, Word can be garbage data
//                               and this function is used to shift out the value
//                               being read from the FLASH.
//
// Return Value:
//
//              There are two possible return values:
//
//              1) garbage - if the function is used to send a value to be written
//                 ie it follows a write command, then the return value is nonsense
//                 and is not used.
//
//              2) 16-bit value read - if the function is used to read a value, ie
//                 the parameter command is dummy data and the function follows a
//                 read command, then the return value will be the 16-bit value read
//                 from the FLASH.
//
// Notes:
//-------------------------------------------------------------------
Uint16 SPIFLASH_Send16bitWord(Uint16 Word)
{
    Uint16 temp;

    temp = 0x0000;

	SpiaRegs.SPITXBUF = Word << 8;                //FLASH LSB 16 bit value
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ;  //Wait for byte to be shifted out
	temp = SpiaRegs.SPIRXBUF;	                  //Clear RX buffer

	SpiaRegs.SPITXBUF = Word;                     //FLASH MSB 16 bit value
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ;  //Wait for byte to be shifted out
	temp |= SpiaRegs.SPIRXBUF << 8;               //Clear RX buffer

    return temp;
}


//****************************************************************
// Function: void SPIFLASH_SendFlashAddr(Uint16 FlashAddr)
//
// This function sends a 24-bit FLASH Address
//
// Parameters:
//
//          FlashAddr               Address of the FLASH.
//
// Return Value:
//
// Notes:
//-------------------------------------------------------------------

void SPIFLASH_SendFlashAddr(Uint32 FlashAddr)
{
	SpiaRegs.SPITXBUF = (Uint32)(FlashAddr>>8); //Flash - high 8-bits of 24-bit address (shifted into MSB of TXBUF)
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
	SpiaRegs.SPIRXBUF;	                         //Clear RX buffer

	SpiaRegs.SPITXBUF = FlashAddr;              //Flash - middle 8-bits of 24-bit address (in MSB of TXBUF)
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
	SpiaRegs.SPIRXBUF;	                         //Clear RX buffer

	SpiaRegs.SPITXBUF = FlashAddr <<8;          //Flash - bottom 8-bits of 24-bit address (in MSB of TXBUF)
	while ( SpiaRegs.SPISTS.bit.INT_FLAG != 1) ; //Wait for byte to be shifted out
	SpiaRegs.SPIRXBUF;	                         //Clear RX buffer
}

//****************************************************************
// Function: SPIFLASH_WriteEnable(void)
//
// This function sets the write enable bit in the FLASH status
// register.
//
// Parameters:
//
// Return Value:
//
//             STATUS_SUCCESS        Write enabled

//             STATUS_FAIL_PROGRAM   Was unable to enable programming.
//
// Notes:
//-------------------------------------------------------------------
Uint16 SPIFLASH_WriteEnable(void)
{
    Uint16 FLASH_StatusReg;
    SPIFLASH_ChipSelectLow();
	SPIFLASH_SendCommand(FLASH_SET_WRITE_ENABLE_LATCH);
    SPIFLASH_ChipSelectHigh();

    FLASH_StatusReg = SPIFLASH_ReadStatReg();
    if(FLASH_StatusReg & FLASH_WEN != 1) return FLASH_STATUS_FAIL_PROGRAM;
    else return FLASH_STATUS_SUCCESS;
}

//****************************************************************
// Function: SPIFLASH_Erase(void)
//
// This function erases the entire SPI FLASH chip.
//
// Parameters:
//
// Return Value:
//
//             STATUS_SUCCESS        Chip erased

//             STATUS_FAIL_PROGRAM   Was unable to erase chip.
//
// Notes:
//-------------------------------------------------------------------

Uint16 SPIFLASH_Erase(void) {
    Uint16 Status;
	Uint16 FLASH_StatusReg;

    // Clear protection bits in the status reg
    SPIFLASH_WriteStatReg(0x0000);

    Status = SPIFLASH_WriteEnable();
    if(Status != FLASH_STATUS_SUCCESS) return FLASH_STATUS_FAIL_ERASE;

    SPIFLASH_ChipSelectLow();
    SPIFLASH_SendCommand(FLASH_CHIP_ERASE);
    SPIFLASH_ChipSelectHigh();

    // Wait for write to complete
    do
    {
        FLASH_StatusReg = SPIFLASH_ReadStatReg();

    } while( (FLASH_StatusReg & FLASH_RDY) != 0 );

    return FLASH_STATUS_SUCCESS;
}


//****************************************************************
// SDFlash Interface Function: PRG_erase()
//
// Erase the specified sectors of flash memory.
//
// Parameters:
//
// Return Value:
//
// Notes:
//
// Which sectors will be erased is determined by the PRG_options2
// argument.
//-------------------------------------------------------------------
void PRG_erase()
{
    Uint16 i;
    Uint16 Status;

    PRG_status = FLASH_STATUS_BUSY;
    PRG_length = FLASH_PAGE_SIZE_WORDS;

    Status = FLASH_STATUS_BUSY;
    // Fill the programming buffer with all 0xFFFF
    for(i = 0; i < FLASH_PAGE_SIZE_WORDS; i++)
    {
       PRG_bufaddr[i] = 0xFFFF;
    }

    // Erase entire chip
    Status = SPIFLASH_Erase();
	Flash_Erase_Status = Status;
    // Verify that each page is erased
    PRG_paddr = 0x0000;

    if (Status== FLASH_STATUS_SUCCESS)
    {
	    for (i=0; i < FLASH_TOTAL_PAGES-1; i++)
	    {
	        Status = SPIFLASH_Verify(PRG_paddr, PRG_bufaddr, (Uint16)PRG_length);
	        if (Status != FLASH_STATUS_SUCCESS) break;
	        PRG_paddr += FLASH_PAGE_SIZE_BYTES;
	    }
	}


    PRG_status = Status;
    return;

}


//****************************************************************
// SDFlash Interface Function: PRG_program()
//
// Program a buffer of data into the FLASH.
//
// Parameters:
//
// Return Value:
//
// Notes:
//      This function is called directly by SDFlash. After one
//      page is programmed, PRG_status remains "BUSY" until all
//      pages in buffer are programmed.
//-------------------------------------------------------------------
void PRG_program(void)
{
    Uint16 Status;
    PRG_status = FLASH_STATUS_BUSY;
	PRG_length = FLASH_PAGE_SIZE_WORDS;

    Status = SPIFLASH_WritePage(PRG_paddr, PRG_bufaddr, PRG_length);

    if(Status == FLASH_STATUS_SUCCESS)
    {
        Status = SPIFLASH_Verify(PRG_paddr, PRG_bufaddr, (Uint16)PRG_length);
    }

    PRG_status = Status;
	PRG_paddr += 0x00000100;
    return;

}

//****************************************************************
// SDFlash Interface Function: PRG_verify()
//
// Verify a "buffer" of flash.
//
// Parameters:
//
// Return Value:
//
// Notes:
//     This function is called directly by SDFlash.
//-------------------------------------------------------------------
void PRG_verify()
{

    //PRG_unlock();
    PRG_status = FLASH_STATUS_BUSY;
    PRG_status = SPIFLASH_Verify(PRG_paddr, PRG_bufaddr, PRG_length);

    return;
}

//_________________________________________________________________________________
//
// Type:	function
// Name:	FlashMemoryInit
//_________________________________________________________________________________
void FlashMemoryInit(){

    // This is a pointer to the flash programming buffer.
    // SDFlash reads this pointer, and then writes image data to the
    // buffer pointed to by PRG_bufaddr.
    PRG_bufaddr  = (Uint16 *)&Buffer[0];

    // This is the size of the data buffer pointed to by PRG_bufaddr.
    // SDFlash reads this value.
    PRG_bufsize  = FLASH_PAGE_SIZE_BYTES;

    // Initialized by SDFlash. Number of target words to program.
    // For '28x 1 word is 16 bits of data.
    //PRG_length = 0;
	PRG_length = FLASH_PAGE_SIZE_WORDS;

    // Initialized by SDFlash. This value will contain the current flash
    // array address to be programmed. This value is updated for each
    // block to be programmed.
    PRG_paddr = 0;

    // Initialized by SDFlash.  The SDFlash utility will intialize
    // PRG_status to a fail value.  When an algorithm executes it
    // should set PRG_status to 0 for pass and non 0 for fail.
    // When the algorithm completes, SDFlash will read the value
    // of SDFlash and continue on pass or exit on fail.
    PRG_status   = 0;

    // The following two values are not used They are here
    // for historical reasons.
    PRG_devsize  = 0;
    PRG_page     = 0;

}



//****************************************************************
// SDFlash Interface Function: PRG_unlock()
//
// Unlock the Code Security Module.
//
// Parameters:
//
// Return Value:
//
// Notes:
//     This function is NOT called directly by SDFlash.
//-------------------------------------------------------------------
//void PRG_unlock()
//{
//    volatile Uint16 temp;
//
//    PRG_status = FLASH_STATUS_BUSY;
//
//    // Load the key registers with the current password
//    EALLOW;
//
//    CsmRegs.KEY0 = PRG_key0;
//    CsmRegs.KEY1 = PRG_key1;
//    CsmRegs.KEY2 = PRG_key2;
//    CsmRegs.KEY3 = PRG_key3;
//    CsmRegs.KEY4 = PRG_key4;
//    CsmRegs.KEY5 = PRG_key5;
//    CsmRegs.KEY6 = PRG_key6;
//    CsmRegs.KEY7 = PRG_key7;
//    EDIS;
//
//    // Perform a dummy read of the password locations
//    // if they match the key values, the CSM will unlock
//    temp = CsmPwl.PSWD0;
//    temp = CsmPwl.PSWD1;
//    temp = CsmPwl.PSWD2;
//    temp = CsmPwl.PSWD3;
//    temp = CsmPwl.PSWD4;
//    temp = CsmPwl.PSWD5;
//    temp = CsmPwl.PSWD6;
//    temp = CsmPwl.PSWD7;
//
//    // If the CSM unlocked then return for now.
//    // When SDFlash calls the functions directly this
//    // return statement can be deleted
//    // If the CSM did not unlock then set the fail
//    // flag and exit.
//    if ( CsmRegs.CSMSCR.bit.SECURE == 0) return;
//    else {
//         PRG_status = 1;
//         PRG_exit();
//    }
//}
