/*******************************************************************************
* Copyright 2011 Cypress Corporation.  All rights reserved.
*
* Unless you and Cypress execute a separate written software license agreement
* governing use of this software, this software is licensed to you under the
* terms of the GNU General Public License version 2, available at
* http://www.gnu.org/copyleft/gpl.html (the "GPL").
*
* Notwithstanding the above, under no circumstances may you combine this
* software in any way with any other Cypress software provided under a license
* other than the GPL, without Cypress' express prior written consent.
*******************************************************************************/
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/gpio.h>
#include <linux/synaptics_i2c_rmi.h>
#include <linux/device.h>
#include <mach/reg_syscfg.h>
#include <linux/uaccess.h> 
#include <linux/slab.h>
#include <linux/syscalls.h>

#include "tst200_download_luisa.h"
#include "./TST200_FIRMWARE_LUISA/Luisa_H02S03.h"
#include "./TST200_FIRMWARE_LUISA/Luisa_H03S0B.h"
#include "./TST200_FIRMWARE_LUISA/Luisa_H04S09.h"
#include "./TST200_FIRMWARE_LUISA/Luisa_H05S04.h"

#define TSP_SDA 27
#define TSP_SCL 26


/***********************
*
* Touchpad Tuning APIs
*
************************/

extern void touch_ctrl_regulator(int on_off);

void TchDrv_DownloadVddSetHigh(void)
{
	touch_ctrl_regulator(1); /* always on */
}

void TchDrv_DownloadVddSetLow(void)
{
	touch_ctrl_regulator(0); /* always off */
}

#if 0
void TchDrv_DownloadIntSetHigh(void)
{
	HAL_GPIO_SetState_st_t HAL_GPIO_Set_st;
	HAL_GPIO_Set_st.gpio_pin = TOUCH_INT_GPIO;
	HAL_GPIO_Ctrl(ACTION_GPIO_SetHigh_64pin, &HAL_GPIO_Set_st, NULL);
}

void TchDrv_DownloadIntSetLow(void)
{
	HAL_GPIO_SetState_st_t HAL_GPIO_Set_st;
	HAL_GPIO_Set_st.gpio_pin = TOUCH_INT_GPIO;
	HAL_GPIO_Ctrl(ACTION_GPIO_SetLow_64pin, &HAL_GPIO_Set_st, NULL);
}

void TchDrv_DownloadIntSetOutput(void)
{
	HAL_GPIO_ConfigOutput_st_t 	HAL_GPIO_ConfigOutput_st;
	HAL_GPIO_ConfigOutput_st.mask = 		GET_GPIO_PIN_MASK(TOUCH_INT_GPIO);/* Interrupt is input */
	HAL_GPIO_Ctrl(ACTION_GPIO_ConfigOutput, &HAL_GPIO_ConfigOutput_st, NULL);

}

void TchDrv_DownloadIntSetInput(void)
{
	HAL_GPIO_ConfigInput_st_t 	HAL_GPIO_ConfigInput_st;
	HAL_GPIO_ConfigInput_st.input_mask = 		GET_GPIO_PIN_MASK(TOUCH_INT_GPIO);/* Interrupt is input */
	HAL_GPIO_Ctrl(ACTION_GPIO_ConfigInput, &HAL_GPIO_ConfigInput_st, NULL);
}



IRQMask_t sv_tch_savedirqmask;

void TchDrv_DownloadDisableIRQ(void)
{
	extern void IRQ_DisableAll( IRQMask_t *mask );
	IRQ_DisableAll(&sv_tch_savedirqmask);
}

void TchDrv_DownloadEnableIRQ(void)
{
	extern void IRQ_Restore( IRQMask_t saved_mask_val );
	IRQ_Restore(sv_tch_savedirqmask);
}

void TchDrv_DownloadDisableWD(void)
{
	extern void WDT_Disable( void );
	WDT_Disable();
}

void TchDrv_DownloadEnableWD(void)
{
	extern void WDT_Enable(void);
	WDT_Enable();
}


void uart_printf(const char *szFormat, ...)
{
}
#endif

#if 0
#define TCH_I2C_SCL_GPIO GPIO_I2C_SCL
#define TCH_I2C_SDA_GPIO GPIO_I2C_SDA


#define I2C_SW_EMU_SCL_GPIO     TCH_I2C_SCL_GPIO
#define I2C_SW_EMU_SDA_GPIO     TCH_I2C_SDA_GPIO
  
////The following macro definitions may need to be changed case by case/////////
#define I2C_SW_EMU_SCL_GPIO_CTRL_REG 		(*(volatile UInt32 *)GPOR0_REG)
#define I2C_SW_EMU_SCL_GPIO_CTRL_MASK 		(0x1<<I2C_SW_EMU_SCL_GPIO)

#define I2C_SW_EMU_SCL_GPIO_TYPE_REG 		(*(volatile UInt32 *)IOTR0_REG)
#define I2C_SW_EMU_SCL_GPIO_READ_REG		(*(volatile UInt32 *)GPIPS0_REG)

#define I2C_SW_EMU_SCL_GPIO_OUTPUT_MASK 	(0x2<<(I2C_SW_EMU_SCL_GPIO*2))
#define I2C_SW_EMU_SCL_GPIO_INPUT_MASK 		(0x0<<(I2C_SW_EMU_SCL_GPIO*2))
#define I2C_SW_EMU_SCL_GPIO_NOTUSED_MASK 	(0x3<<(I2C_SW_EMU_SCL_GPIO*2))
#define I2C_SW_EMU_SCL_GPIO_READ_MASK		(0x1<<I2C_SW_EMU_SCL_GPIO)

#define I2C_SET_SCL_GPIO() 		I2C_SW_EMU_SCL_GPIO_TYPE_REG &= ~I2C_SW_EMU_SCL_GPIO_NOTUSED_MASK	//gpio input
#define I2C_CLR_SCL_GPIO() 		I2C_SW_EMU_SCL_GPIO_TYPE_REG |= I2C_SW_EMU_SCL_GPIO_OUTPUT_MASK		//gpio output
#define I2C_SET_SCL_GPIO_LOW() 	I2C_SW_EMU_SCL_GPIO_CTRL_REG &= ~I2C_SW_EMU_SCL_GPIO_CTRL_MASK		//gpio output low
#define I2C_SET_SCL_GPIO_HIGH() I2C_SW_EMU_SCL_GPIO_CTRL_REG |= I2C_SW_EMU_SCL_GPIO_CTRL_MASK		//gpio output high

#define I2C_SW_EMU_SDA_GPIO_CTRL_REG 		(*(volatile UInt32 *)GPOR0_REG)
#define I2C_SW_EMU_SDA_GPIO_TYPE_REG 		(*(volatile UInt32 *)IOTR0_REG)
#define I2C_SW_EMU_SDA_GPIO_READ_REG		(*(volatile UInt32 *)GPIPS0_REG)

#define I2C_SW_EMU_SDA_GPIO_CTRL_MASK 		(0x1<<I2C_SW_EMU_SDA_GPIO)
#define I2C_SW_EMU_SDA_GPIO_OUTPUT_MASK 	(0x2<<(I2C_SW_EMU_SDA_GPIO*2))
#define I2C_SW_EMU_SDA_GPIO_INPUT_MASK 		(0x0<<(I2C_SW_EMU_SDA_GPIO*2))
#define I2C_SW_EMU_SDA_GPIO_NOTUSED_MASK 	(0x3<<(I2C_SW_EMU_SDA_GPIO*2))
#define I2C_SW_EMU_SDA_GPIO_READ_MASK		(0x1<<I2C_SW_EMU_SDA_GPIO)

#define I2C_SET_SDA_GPIO() 		I2C_SW_EMU_SDA_GPIO_TYPE_REG &= ~I2C_SW_EMU_SDA_GPIO_NOTUSED_MASK	//gpio input
#define I2C_CLR_SDA_GPIO() 		I2C_SW_EMU_SDA_GPIO_TYPE_REG |= I2C_SW_EMU_SDA_GPIO_OUTPUT_MASK		//gpio output
#define I2C_SET_SDA_GPIO_LOW() 	I2C_SW_EMU_SDA_GPIO_CTRL_REG &= ~I2C_SW_EMU_SDA_GPIO_CTRL_MASK		//gpio output low
#define I2C_SET_SDA_GPIO_HIGH() I2C_SW_EMU_SDA_GPIO_CTRL_REG |= I2C_SW_EMU_SDA_GPIO_CTRL_MASK		//gpio output high

#define I2C_READ_SDA_GPIO() 		(I2C_SW_EMU_SDA_GPIO_READ_REG&I2C_SW_EMU_SDA_GPIO_READ_MASK)	//input is being driven high/low
#define I2C_READ_SCL_GPIO() 	   (I2C_SW_EMU_SCL_GPIO_READ_REG&I2C_SW_EMU_SCL_GPIO_READ_MASK)		//input is being driven high/low
#endif

/*++ DELAY TEST ++*/
// provides delays in us
#define ONE_MICROSSEC_CNT 100 // 312MHz
static void delay_1us(void)
{
	volatile unsigned int i;
 
	for(i=0; i<ONE_MICROSSEC_CNT; i++)
	{
	}
}

static void delay_us(unsigned int us)
{
	volatile unsigned int i;
 
	for(i=0; i<us; i++)
	{
		delay_1us();
	}
}
/*-- DELAY TEST --*/




//-----------------------------------------
//
//   Converting  ASCII and VALUE
//
//-----------------------------------------

static unsigned char mcsdl_hex_htoi( unsigned char *pAscii )
{

   unsigned char    ucTemp = 0;

   switch (*pAscii) {

      case '0' : ucTemp = 0x00;   break;
      case '1' : ucTemp = 0x01;   break;
      case '2' : ucTemp = 0x02;   break;
      case '3' : ucTemp = 0x03;   break;
      case '4' : ucTemp = 0x04;   break;
      case '5' : ucTemp = 0x05;   break;
      case '6' : ucTemp = 0x06;   break;
      case '7' : ucTemp = 0x07;   break;
      case '8' : ucTemp = 0x08;   break;
      case '9' : ucTemp = 0x09;   break;

      case 'a' :
      case 'A' : ucTemp = 0x0A;   break;
      case 'b' :
      case 'B' : ucTemp = 0x0B;   break;
      case 'c' :
      case 'C' : ucTemp = 0x0C;   break;
      case 'd' :
      case 'D' : ucTemp = 0x0D;   break;
      case 'e' :
      case 'E' : ucTemp = 0x0E;   break;
      case 'f' :
      case 'F' : ucTemp = 0x0F;   break;

      default  :                break;
   }

   return ucTemp;
}


//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_driver_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


unsigned int	wBinaryAddress;
unsigned char	bTargetDataPtr;
unsigned char	abTargetDataOUT[TARGET_DATABUFF_LEN];
unsigned char	abTargetDataOUT_secure[TARGET_DATABUFF_LEN] ={0x00,};
//unsigned char	abSecurityData[TARGET_DATABUFF_LEN];
///unsigned char    firmData[514][64];
unsigned char firmData[512][64];


// for test - kjhw
extern unsigned char BinaryData[128] =
{
        0x7d, 0x00, 0x68, 0x30, 0x30, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7d, 0x05, 0x27, 0x7e, 0x7d, 0x05, 0xcf, 0x7e, // 20
        0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7d, 0x10, 0xe7, 0x7e, 0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, // 40
        0x7e, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, // 60
        0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x7e, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, // 80
        0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x30, 0x4f, 0x5a, 0xf9, 0x06, // 100
        0xf9, 0x03, 0x00, 0x7f, 0x40, 0x43, 0xe6, 0x02, 0x70, 0xcf, 0x71, 0x10, 0x62, 0xe3, 0x00, 0x70, 0xcf, 0x50, 0x80, 0x4e, // 120
        0x5d, 0xd5, 0x08, 0x62, 0xd5, 0x00, 0x55, 0xfa  // 128
};



// ****************************** PORT BIT MASKS ******************************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
#define SDATA_PIN   0x80        // P1.7
#define SCLK_PIN    0x40        // P1.6
#define XRES_PIN    0x40        // P2.6
#define TARGET_VDD  0x08        // P2.3


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// Delay()
// This delay uses a simple "nop" loop. With the CPU running at 24MHz, each
// pass of the loop is about 1 usec plus an overhead of about 3 usec.
//      total delay = (n + 3) * 1 usec
// To adjust delays and to adapt delays when porting this application, see the
// ISSP_Delays.h file.
// ****************************************************************************
void Delay(unsigned char n)  // by KIMC
{
    while(n)
    {
        //asm("nop");
        n -= 1;
    }
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// LoadProgramData()
// The final application should load program data from HEX file generated by
// PSoC Designer into a 64 byte host ram buffer.
//    1. Read data from next line in hex file into ram buffer. One record
//      (line) is 64 bytes of data.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread data from file or abort
//    4. Exit this Function and Program block or verify the block.
// This demo program will, instead, load predetermined data into each block.
// The demo does it this way because there is no comm link to get data.
// ****************************************************************************
void LoadProgramData(unsigned char bBankNum, unsigned char bBlockNum)
{
    // >>> The following call is for demo use only. <<<
    // Function InitTargetTestData fills buffer for demo
    for (bTargetDataPtr = 0; bTargetDataPtr < TARGET_DATABUFF_LEN; bTargetDataPtr++)
    {
        // Binarydata is f/w binary.
        abTargetDataOUT[bTargetDataPtr] = BinaryData[wBinaryAddress];
        wBinaryAddress++;

        if (wBinaryAddress == 128)
        {
            wBinaryAddress = 0;
        }
    }

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns VOID.
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fLoadSecurityData()
// Load security data from hex file into 64 byte host ram buffer. In a fully
// functional program (not a demo) this routine should do the following:
//    1. Read data from security record in hex file into ram buffer.
//    2. Check host ram buffer + record data (Address, # of bytes) against hex
//       record checksum at end of record line
//    3. If error reread security data from file or abort
//    4. Exit this Function and Program block
// In this demo routine, all of the security data is set to unprotected (0x00)
// and it returns.
// This function always returns PASS. The flag return is reserving
// functionality for non-demo versions.
// ****************************************************************************
signed char fLoadSecurityData(unsigned char bBankNum)
{
    // >>> The following call is for demo use only. <<<
    // Function LoadArrayWithSecurityData fills buffer for demo

    // abSecurityData is security data which is included in the f/w file.
    for (bTargetDataPtr = 0; bTargetDataPtr < SECURITY_BYTES_PER_BANK; bTargetDataPtr++)
    {
        //abTargetDataOUT[bTargetDataPtr] = abSecurityData[bTargetDataPtr];
        abTargetDataOUT_secure[bTargetDataPtr] = 0xff;
    }

    // Note:
    // Error checking should be added for the final version as noted above.
    // For demo use this function just returns PASS.
    return(PASS);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// fSDATACheck()
// Check SDATA pin for high or low logic level and return value to calling
// routine.
// Returns:
//     0 if the pin was low.
//     1 if the pin was high.
// ****************************************************************************
unsigned char fSDATACheck(void)
{
    if ( gpio_get_value ( TSP_SDA ) )
        return(1);
    else
        return(0);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKHigh()
// Set the SCLK pin High
// ****************************************************************************
void SCLKHigh(void)
{
	gpio_direction_output(TSP_SCL, 1);//gpio output high
	delay_us(1);
	//udelay(1);
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SCLKLow()
// Make Clock pin Low
// ****************************************************************************
void SCLKLow(void)
{
	gpio_direction_output(TSP_SCL, 0);//gpio output low
	delay_us(1);
	//udelay(1);
}

#ifndef RESET_MODE  // Only needed for power cycle mode
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKHiZ()
// Set SCLK pin to HighZ drive mode.
// ****************************************************************************
void SetSCLKHiZ(void)
{
	gpio_direction_input(TSP_SCL);//gpio input
}
#endif

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSCLKStrong()
// Set SCLK to an output (Strong drive mode)
// ****************************************************************************
void SetSCLKStrong(void)
{
	gpio_direction_output(TSP_SCL ,0);//gpio output	(low)
}


// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHigh()
// Make SDATA pin High
// ****************************************************************************
void SetSDATAHigh(void)
{
#if 0
    PRT1DR |= SDATA_PIN;
#endif
	gpio_direction_output(TSP_SDA ,1);//gpio output high
	delay_us(2);
	//udelay(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATALow()
// Make SDATA pin Low
// ****************************************************************************
void SetSDATALow(void)
{
	gpio_direction_output(TSP_SDA, 0);//gpio output low
	delay_us(2);
	//udelay(2);
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAHiZ()
// Set SDATA pin to an input (HighZ drive mode).
// ****************************************************************************
void SetSDATAHiZ(void)
{
	gpio_direction_input(TSP_SDA);//gpio input	
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetSDATAStrong()
// Set SDATA for transmission (Strong drive mode) -- as opposed to being set to
// High Z for receiving data.
// ****************************************************************************
void SetSDATAStrong(void)
{
	gpio_direction_output(TSP_SDA, 1);//gpio output	(high)
}

#ifdef RESET_MODE
// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetXRESStrong()
// Set external reset (XRES) to an output (Strong drive mode).
// ****************************************************************************
void SetXRESStrong(void)
{
    PRT2DM0 |=  XRES_PIN;
    PRT2DM1 &= ~XRES_PIN;
    PRT2DM2 &= ~XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// AssertXRES()
// Set XRES pin High
// ****************************************************************************
void AssertXRES(void)
{
    PRT2DR |= XRES_PIN;
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// DeassertXRES()
// Set XRES pin low.
// ****************************************************************************
void DeassertXRES(void)
{
    PRT2DR &= ~XRES_PIN;
}
#else

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// SetTargetVDDStrong()
// Set VDD pin (PWR) to an output (Strong drive mode).
// ****************************************************************************
void SetTargetVDDStrong(void)
{
	TchDrv_DownloadVddSetLow();	
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// ApplyTargetVDD()
// Provide power to the target PSoC's Vdd pin through a GPIO.
// ****************************************************************************
void ApplyTargetVDD(void)
{
	TchDrv_DownloadVddSetHigh();
}

// ********************* LOW-LEVEL ISSP SUBROUTINE SECTION ********************
// ****************************************************************************
// ****                        PROCESSOR SPECIFIC                          ****
// ****************************************************************************
// ****                      USER ATTENTION REQUIRED                       ****
// ****************************************************************************
// RemoveTargetVDD()
// Remove power from the target PSoC's Vdd pin.
// ****************************************************************************
void RemoveTargetVDD(void)
{
	TchDrv_DownloadVddSetLow();
}
#endif





//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Issp_routines.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

unsigned char  bTargetDataIN;
//unsigned char  abTargetDataOUT_secure[TARGET_DATABUFF_LEN] ={0x00,};
//unsigned char  abTargetDataOUT[TARGET_DATABUFF_LEN];
//unsigned char  abSecurityData[TARGET_DATABUFF_LEN];

extern unsigned int   wBinaryAddress;

unsigned char  bTargetAddress;
unsigned char  bTargetDataPtr = 0;
unsigned char  bTargetID[10];
unsigned char  bTargetStatus[10];			//PTJ: created to support READ-STATUS in fReadStatus()

unsigned char  fIsError;

/* ((((((((((((((((((((( LOW-LEVEL ISSP SUBROUTINE SECTION ))))))))))))))))))))
   (( The subroutines in this section use functions from the C file          ))
   (( ISSP_Drive_Routines.c. The functions in that file interface to the     ))
   (( processor specific hardware. So, these functions should work as is, if ))
   (( the routines in ISSP_Drive_Routines.c are correctly converted.         ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

// ============================================================================
// RunClock()
// Description:
// Run Clock without sending/receiving bits. Use this when transitioning from
// write to read and read to write "num_cycles" is number of SCLK cycles, not
// number of counter cycles.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// ============================================================================
void RunClock(unsigned int iNumCycles)
{
    int i;

    for(i=0; i < iNumCycles; i++) {
        SCLKLow();
        SCLKHigh();
    }
    // function exits with CLK high.
}

// ============================================================================
// bReceiveBit()
// Clocks the SCLK pin (high-low-high) and reads the status of the SDATA pin
// after the rising edge.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if SDATA was low
//     1 if SDATA was high
// ============================================================================
unsigned char bReceiveBit(void)
{
    SCLKLow();
    SCLKHigh();
    if (fSDATACheck()) {
        return(1);
    }
    else {
        return(0);
    }
}

// ============================================================================
// bReceiveByte()
// Calls ReceiveBit 8 times to receive one byte.
// Returns:
//     The 8-bit values recieved.
// ============================================================================
unsigned char bReceiveByte(void)
{
    unsigned char b;
    unsigned char bCurrByte = 0x00;

    for (b=0; b<8; b++) {
        bCurrByte = (bCurrByte<<1) + bReceiveBit();
    }
    return(bCurrByte);
}


// ============================================================================
// SendByte()
// This routine sends up to one byte of a vector, one bit at a time.
//    bCurrByte   the byte that contains the bits to be sent.
//    bSize       the number of bits to be sent. Valid values are 1 to 8.
//
// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// There is no returned value.
// ============================================================================
void SendByte(unsigned char bCurrByte, unsigned char bSize)
{
    unsigned char b = 0;

    for(b=0; b<bSize; b++) {
        if (bCurrByte & 0x80) {
            // Send a '1'
            SetSDATAHigh();
            SCLKHigh();
            SCLKLow();
        }
        else {
            // Send a '0'
            SetSDATALow();
            SCLKHigh();
            SCLKLow();
        }
        bCurrByte = bCurrByte << 1;
    }
}

// ============================================================================
// SendVector()
// This routine sends the vector specifed. All vectors constant strings found
// in ISSP_Vectors.h.  The data line is returned to HiZ after the vector is
// sent.
//    bVect      a pointer to the vector to be sent.
//    nNumBits   the number of bits to be sent.
//    bCurrByte  scratch var to keep the byte to be sent.
//
// There is no returned value.
// ============================================================================
void SendVector(const unsigned char* bVect, unsigned int iNumBits)
{
    SetSDATAStrong();

    while(iNumBits > 0)
    {
        if (iNumBits >= 8) {
            SendByte(*(bVect), 8);
            iNumBits -= 8;
            bVect++;
        }
        else {
            SendByte(*(bVect), iNumBits);
            iNumBits = 0;
        }
    }
    SetSDATAHiZ();
}


// ============================================================================
// fDetectHiLoTransition()
// Waits for transition from SDATA = 1 to SDATA = 0.  Has a 100 msec timeout.
// TRANSITION_TIMEOUT is a loop counter for a 100msec timeout when waiting for
// a high-to-low transition. This is used in the polling loop of
// fDetectHiLoTransition(). The timing of the while(1) loops can be calculated
// and the number of loops is counted, using iTimer, to determine when 100
// msec has passed.
//
//// SCLK cannot run faster than the specified maximum frequency of 8MHz. Some
// processors may need to have delays added after setting SCLK low and setting
// SCLK high in order to not exceed this specification. The maximum frequency
// of SCLK should be measured as part of validation of the final program
//
// Returns:
//     0 if successful
//    -1 if timed out.
// ============================================================================
signed char fDetectHiLoTransition(void)
{
    // nTimer breaks out of the while loops if the wait in the two loops totals
    // more than 100 msec.  Making this static makes the loop run a faster.
    // This is really a processor/compiler dependency and it not needed.
    static unsigned int iTimer;

///	printk("[TSP] %s, %d\n", __func__, __LINE__);

    // NOTE:
    // These loops look unconventional, but it is necessary to check SDATA_PIN
    // as shown because the transition can be missed otherwise, due to the
    // length of the SDATA Low-High-Low after certain commands.

    // Generate clocks for the target to pull SDATA High
     iTimer = TRANSITION_TIMEOUT;//200ms
     while(1) {
        SCLKLow();
        if (fSDATACheck())       // exit once SDATA goes HI
            break;
        SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
			printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
            return (ERROR);
        }
    }
    // Generate Clocks and wait for Target to pull SDATA Low again
    iTimer = TRANSITION_TIMEOUT;              // reset the timeout counter
    while(1) {
        SCLKLow();
        if (!fSDATACheck()) {   // exit once SDATA returns LOW
            break;
        }
        //SCLKHigh();
        // If the wait is too long then timeout
        if (iTimer-- == 0) {
			printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
            return (ERROR);
        }
    }
    return (PASS);
}


/* ((((((((((((((((((((( HIGH-LEVEL ISSP ROUTINE SECTION ))))))))))))))))))))))
   (( These functions are mostly made of calls to the low level routines     ))
   (( above.  This should isolate the processor-specific changes so that     ))
   (( these routines do not need to be modified.                             ))
   (((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))))*/

#ifdef RESET_MODE
// ============================================================================
// fXRESInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fXRESInitializeTargetForISSP(void)
{
    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();
    SetXRESStrong();

    // Cycle reset and put the device in programming mode when it exits reset
    AssertXRES();
    Delay(XRES_CLK_DELAY);

    DeassertXRES();

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    //PTJ: Send id_setup_1 instead of init1_v
    //PTJ: both send CA Test Key and do a Calibrate1 SROM function
    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
        #ifdef TX_ON
            TX8SW_CPutString("\r\n fDetectHiLoTransition Error");
        #endif
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}


#else  //else = the part is power cycle programmed

// ============================================================================
// fPowerCycleInitializeTargetForISSP()
// Implements the intialization vectors for the device.
// The first time fDetectHiLoTransition is called the Clk pin is highZ because
// the clock is not needed during acquire.
// Returns:
//     0 if successful
//     INIT_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fPowerCycleInitializeTargetForISSP(void)
{
    unsigned char n;

///	printk("[TSP] %s, %d\n", __func__, __LINE__);

    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();

    // Turn on power to the target device before other signals
    SetTargetVDDStrong();
	mdelay(200);//200ms
    ApplyTargetVDD();

// wait 1msec for the power to stabilize
//    for (n=0; n<10; n++) {
//        Delay(DELAY100us);
//    }
	//mdelay(1);//1ms

    // Set SCLK to high Z so there is no clock and wait for a high to low
    // transition on SDAT. SCLK is not needed this time.
    //SetSCLKHiZ();

	SetSCLKStrong();

    if (fIsError = fDetectHiLoTransition()) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(INIT_ERROR);
    }

    // Configure the pins for initialization
    SetSDATAHiZ();
    SetSCLKStrong();
    SCLKLow();					//PTJ: DO NOT SET A BREAKPOINT HERE AND EXPECT SILICON ID TO PASS!

    // !!! NOTE:
    //  The timing spec that requires that the first Init-Vector happen within
    //  1 msec after the reset/power up. For this reason, it is not advisable
    //  to separate the above RESET_MODE or POWER_CYCLE_MODE code from the
    //  Init-Vector instructions below. Doing so could introduce excess delay
    //  and cause the target device to exit ISSP Mode.

    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(INIT_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    return(PASS);
}
#endif


// ============================================================================
// fVerifySiliconID()
// Returns:
//     0 if successful
//     Si_ID_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySiliconID(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    SendVector(id_setup_1, num_bits_id_setup_1);
    if (fIsError = fDetectHiLoTransition()) {
        #ifdef TX_ON
            TX8SW_CPutString("\r\n fDetectHiLoTransition Error");
        #endif
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(SiID_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    SendVector(id_setup_2, num_bits_id_setup_2);
    if (fIsError = fDetectHiLoTransition()) {
        #ifdef TX_ON
            TX8SW_CPutString("\r\n fDetectHiLoTransition Error");
        #endif
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(SiID_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    SendVector(tsync_enable, num_bits_tsync_enable);

    //Send Read ID vector and get Target ID
    SendVector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetID[0] = bReceiveByte();
    RunClock(1);
    SendVector(read_id_v+2, 12);    // 1+11 bits starting from the 3rd byte

    RunClock(2);                    // Read-LSB Command
    bTargetID[1] = bReceiveByte();

    RunClock(1);
    SendVector(read_id_v+4, 1);     // 1 bit starting from the 5th byte

    //read Revision ID from Accumulator A and Accumulator X
    SendVector(read_id_v+5, 11);	//11 bits starting from the 6th byte
    RunClock(2);
    bTargetID[2] = bReceiveByte();	//Read from Acc.X
    RunClock(1);
    SendVector(read_id_v+7, 12);    //1+11 bits starting from the 8th byte

    RunClock(2);
    bTargetID[3] = bReceiveByte();	//Read from Acc.A

    RunClock(1);
    SendVector(read_id_v+4, 1);     //1bit starting from the 5th byte,

    #ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);
    #endif /* TSYNC */


    #ifdef TX_ON
        // Print READ-ID
        TX8SW_CPutString("\r\n Silicon-ID : ");
        TX8SW_PutChar(' ');
        TX8SW_PutSHexByte(bTargetID[0]);
        TX8SW_PutChar(' ');
        TX8SW_PutSHexByte(bTargetID[1]);
        TX8SW_PutChar(' ');
        TX8SW_PutSHexByte(bTargetID[2]);
        TX8SW_PutChar(' ');
        TX8SW_PutSHexByte(bTargetID[3]);
        TX8SW_PutChar(' ');
    #endif

    if ( (bTargetID[0] != target_id_v[0]) || (bTargetID[1] != target_id_v[1]) )
    {
    	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(SiID_ERROR);
    }
    else
    {
        return(PASS);
    }
}


// PTJ: =======================================================================
// fReadStatus()
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadStatus(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

#ifdef TSYNC
        SendVector(tsync_enable, num_bits_tsync_enable);
#endif /* TSYNC */

    //Send Read ID vector and get Target ID
    SendVector(read_id_v, 11);      // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    // Two SCLK cycles between write & read
    bTargetStatus[0] = bReceiveByte();
    RunClock(1);
    SendVector(read_id_v+2, 12);    // 12 bits starting from the 3rd character

    RunClock(2);                    // Read-LSB Command
    bTargetStatus[1] = bReceiveByte();

    RunClock(1);
    SendVector(read_id_v+4, 1);     // 1 bit starting from the 5th character

#ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);
#endif /* TSYNC */

    if (bTargetStatus[0] == target_status00_v) {
        return(PASS);			//PTJ: Status = 00 means Success, the SROM function did what it was supposed to
    }
    if (bTargetStatus[0] == target_status01_v) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(STATUS_ERROR);	//PTJ: Status = 01 means that function is not allowed because of block level protection, for test with verify_setup (VERIFY-SETUP)
    }
    if (bTargetStatus[0] == target_status03_v) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(STATUS_ERROR);	//PTJ: Status = 03 is fatal error, SROM halted
    }
    if (bTargetStatus[0] == target_status04_v) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(STATUS_ERROR);	//PTJ: Status = 04 means there was a checksum faliure with either the smart write code checksum, or the smart write paramters checksum, for test with PROGRAM-AND-VERIFY
    }
    if (bTargetStatus[0] == target_status06_v) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(STATUS_ERROR);	//PTJ: Status = 06 means that Calibrate1 failed, for test with id_setup_1 (ID-SETUP-1)
    }
    else {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(STATUS_ERROR);
    }
}
// PTJ: =======================================================================
// fReadCalRegisters()
// PTJ:  use this to read some cal registers that should be loaded by Calibrate1 in id_setup_1
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadCalRegisters(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

#ifdef TSYNC
        SendVector(tsync_enable, num_bits_tsync_enable);
#endif /* TSYNC */

    SendVector(Switch_Bank1, 22);

    SendVector(read_IMOtrim, 11);      	// Read-MSB Vector is the first 11-Bits
    RunClock(2);                    	// Two SCLK cycles between write & read
    bTargetStatus[0] = bReceiveByte();
    RunClock(1);
    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(read_reg_end, 1);

    SendVector(read_SPCtrim, 11);      	// Read-MSB Vector is the first 11-Bits
    RunClock(2);                    	// Two SCLK cycles between write & read
    bTargetStatus[1] = bReceiveByte();
    RunClock(1);
    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(read_reg_end, 1);

    SendVector(read_VBGfinetrim, 11);   // Read-MSB Vector is the first 11-Bits
    RunClock(2);                    	// Two SCLK cycles between write & read
    bTargetStatus[2] = bReceiveByte();
    RunClock(1);
    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(read_reg_end, 1);

	SendVector(Switch_Bank0, 22);

#ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);
#endif /* TSYNC */

    if (bTargetStatus[0] == target_status00_v) {
        return(PASS);			//PTJ: Status = 00 means Success, the SROM function did what it was supposed to
    }
    return PASS;
}

// PTJ: =======================================================================
// fReadWriteSetup()
// PTJ: The READ-WRITE-SETUP vector will enable TSYNC and switches the device
//		to SRAM bank1 for PROGRAM-AND-VERIFY, SECURE and VERIFY-SETUP.
// Returns:
//     0 if successful
//     _____ if timed out on handshake to the device.
// ============================================================================
signed char fReadWriteSetup(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    SendVector(read_write_setup, num_bits_read_write_setup);
    return(PASS);					//PTJ: is there anything else that should be done?
}


// ============================================================================
// fEraseTarget()
// Perform a bulk erase of the target device.
// Returns:
//     0 if successful
//     ERASE_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fEraseTarget(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    SendVector(erase, num_bits_erase);
    if (fIsError = fDetectHiLoTransition()) {
        #ifdef TX_ON
            TX8SW_CPutString("\r\n fDetectHiLoTransition");
        #endif
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(ERASE_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}


extern unsigned int iBlockCounter;
// ============================================================================
// LoadTarget()
// Transfers data from array in Host to RAM buffer in the target.
// Returns the checksum of the data.
// ============================================================================
unsigned int iLoadTarget(void)
{
unsigned char bTemp;
unsigned int  iChecksumData = 0;

///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    // Set SDATA to Strong Drive here because SendByte() does not
    SetSDATAStrong();

    // Transfer the temporary RAM array into the target.
    // In this section, a 128-Byte array was specified by #define, so the entire
    // 128-Bytes are written in this loop.
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    while(bTargetDataPtr < TARGET_DATABUFF_LEN) {
        bTemp = abTargetDataOUT[bTargetDataPtr];
        iChecksumData += bTemp;

        SendByte(write_byte_start,4);    //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTargetAddress, 7);	 //PTJ: we need to be able to write 128 bytes from address 0x80 to 0xFF
        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);

        // !!!NOTE:
        // SendByte() uses MSbits, so inc by '2' to put the 0..128 address into
        // the seven MSBit locations.
        //
        // This can be confusing, but check the logic:
        //   The address is only 7-Bits long. The SendByte() subroutine will
        // send however-many bits, BUT...always reads them bits from left-to-
        // right. So in order to pass a value of 0..128 as the address using
        // SendByte(), we have to left justify the address by 1-Bit.
        //   This can be done easily by incrementing the address each time by
        // '2' rather than by '1'.

        bTargetAddress += 2;			//PTJ: inc by 2 in order to support a 128 byte address space, MSB~1 for address
        bTargetDataPtr++;
    }

    return(iChecksumData);
}



// ============================================================================
// fProgramTargetBlock()
// Program one block with data that has been loaded into a RAM buffer in the
// target device.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fProgramTargetBlock(unsigned char bBankNumber, unsigned char bBlockNumber)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    #ifdef TSYNC
        SendVector(tsync_enable, num_bits_tsync_enable);
    #endif /* TSYNC */

    #ifdef SET_BLOCK_NUM
        SendVector(set_block_num, num_bits_set_block_num);
    #endif

    // Set the drive here because SendByte() does not.
    SetSDATAStrong();
    SendByte(bBlockNumber,8);

    #ifdef SET_BLOCK_NUM
        SendByte(set_block_num_end, 3);
    #endif

    #ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);
    #endif /* TSYNC */

    // Send the program-block vector.
    #ifdef PROGRAM_AND_VERIFY
        SendVector(program_and_verify, num_bits_program_and_verify);		//PTJ: PROGRAM-AND-VERIFY
    #endif
    // wait for acknowledge from target.
    if (fIsError = fDetectHiLoTransition()) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(BLOCK_ERROR);
    }
    // Send the Wait-For-Poll-End vector
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);

}


// ============================================================================
// fAddTargetBankChecksum()
// Reads and adds the target bank checksum to the referenced accumulator.
// Returns:
//     0 if successful
//     VERIFY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fAccTargetBankChecksum(unsigned int* pAcc)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    unsigned int wCheckSumData = 0;

    SendVector(checksum_setup, num_bits_checksum_setup); 		//PTJ:CHECKSUM-SETUP, it is taking 100ms > time > 200ms to complete the checksum
	mdelay(200); // 200ms delay
	/*
    if (fIsError = fDetectHiLoTransition())
    {                   //100ms is default
        //TX8SW_CPutString("This Hi-LOW");
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(SECURITY_ERROR);
	}
	*/

    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

#ifdef TSYNC
        SendVector(tsync_enable, num_bits_tsync_enable);
#endif /* TSYNC */

    //Send Read Checksum vector and get Target Checksum
    SendVector(read_checksum_v, 11);     // first 11-bits is ReadCKSum-MSB
    RunClock(2);                         // Two SCLKs between write & read
    bTargetDataIN = bReceiveByte();
    wCheckSumData = (bTargetDataIN & 0xFF)<<8;

    RunClock(1);                         // See Fig. 6
    SendVector(read_checksum_v + 2, 12); // 12 bits starting from 3rd character
    RunClock(2);                         // Read-LSB Command
    bTargetDataIN = bReceiveByte();
    wCheckSumData |= (bTargetDataIN & 0xFF);
    RunClock(1);
    SendVector(read_checksum_v + 4, 1);  // Send the final bit of the command

#ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);
#endif /* TSYNC */

    *pAcc = wCheckSumData;

    return(PASS);
}


// ============================================================================
// ReStartTarget()
// After programming, the target PSoC must be reset to take it out of
// programming mode. This routine performs a reset.
// ============================================================================
void ReStartTarget(void)
{
	printk( "[TSP] %s, %d\n", __func__, __LINE__);

#ifdef RESET_MODE
    // Assert XRES, then release, then disable XRES-Enable
    AssertXRES();
    Delay(XRES_CLK_DELAY);
    DeassertXRES();
#else
    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();
    // Cycle power on the target to cause a reset
    RemoveTargetVDD();
	mdelay(200);
    ApplyTargetVDD();
	mdelay(200);
#endif
}

// ============================================================================
// fVerifySetup()
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifySetup(unsigned char bBankNumber, unsigned char bBlockNumber)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    #ifdef TSYNC
        SendVector(tsync_enable, num_bits_tsync_enable);
    #endif /* TSYNC */

    #ifdef SET_BLOCK_NUM
        SendVector(set_block_num, num_bits_set_block_num);
    #endif

    //Set the drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    #ifdef SET_BLOCK_NUM
        SendByte(set_block_num_end, 3);					//PTJ:
    #endif

    #ifdef TSYNC
        SendVector(tsync_disable, num_bits_tsync_disable);  //PTJ:
    #endif /* TSYNC */

    #ifdef VERIFY_SETUP
        SendVector(verify_setup, num_bits_my_verify_setup);     //PTJ:
    #endif
    if (fIsError = fDetectHiLoTransition()) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(BLOCK_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

    return(PASS);
}


// ============================================================================
// fReadByteLoop()
// Reads the data back from Target SRAM and compares it to expected data in
// Host SRAM
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================

signed char fReadByteLoop(void)
{
    bTargetAddress = 0;
    bTargetDataPtr = 0;
	
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    while(bTargetDataPtr < TARGET_DATABUFF_LEN)
    {
        //Send Read Byte vector and then get a byte from Target
        SendVector(read_byte_v, 4);
        // Set the drive here because SendByte() does not
        SetSDATAStrong();
        SendByte(bTargetAddress,7);

        RunClock(2);       // Run two SCLK cycles between writing and reading
        SetSDATAHiZ();     // Set to HiZ so Target can drive SDATA
        bTargetDataIN = bReceiveByte();

        RunClock(1);
        SendVector(read_byte_v + 1, 1);     // Send the ReadByte Vector End

        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        if (bTargetDataIN != abTargetDataOUT[bTargetDataPtr])
        {
        	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
            return(BLOCK_ERROR);
        }

        bTargetDataPtr++;
        // Increment the address by 2 to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        bTargetAddress += 2;

    }

    return(PASS);
}


// ============================================================================
// fVerifyTargetBlock()
// Verify the block just written to. This can be done byte-by-byte before the
// protection bits are set.
// Returns:
//     0 if successful
//     BLOCK_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fVerifyTargetBlock(unsigned char bBankNumber, unsigned char bBlockNumber)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    SendVector(set_block_number, 11);

	//Set the drive here because SendByte() does not
    SetSDATAStrong();
    SendByte(bBlockNumber,8);
    SendByte(set_block_number_end, 3);

    SendVector(verify_setup, num_bits_my_verify_setup);

    if (fIsError = fDetectHiLoTransition())
    {
    	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(BLOCK_ERROR);
    }
    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

	bTargetAddress = 0;
    bTargetDataPtr = 0;


    while(bTargetDataPtr < TARGET_DATABUFF_LEN) {
        //Send Read Byte vector and then get a byte from Target
        SendVector(read_byte_v, 4);
        // Set the drive here because SendByte() does not
        SetSDATAStrong();
        SendByte(bTargetAddress,7);

        RunClock(2);       // Run two SCLK cycles between writing and reading
        SetSDATAHiZ();     // Set to HiZ so Target can drive SDATA
        bTargetDataIN = bReceiveByte();

        RunClock(1);
        SendVector(read_byte_v + 1, 1);     // Send the ReadByte Vector End

        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return an error.
        if (bTargetDataIN != abTargetDataOUT[bTargetDataPtr])
        {
#ifdef TX_ON
            TX8SW_PutSHexByte(bTargetDataIN);
            TX8SW_PutSHexByte(abTargetDataOUT[bTargetDataPtr]);
#endif
			printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
            return(BLOCK_ERROR);
        }

        bTargetDataPtr++;
        // Increment the address by four to accomodate 6-Bit addressing
        // (puts the 6-bit address into MSBit locations for "SendByte()").
        bTargetAddress += 2;
    }

    return(PASS);
}


// ============================================================================
// fSecureTargetFlash()
// Before calling, load the array, abTargetDataOUT, with the desired security
// settings using LoadArrayWithSecurityData(StartAddress,Length,SecurityType).
// The can be called multiple times with different SecurityTypes as needed for
// particular Flash Blocks. Or set them all the same using the call below:
// Returns:
//     0 if successful
//     SECURITY_ERROR if timed out on handshake to the device.
// ============================================================================
signed char fSecureTargetFlash(void)
{
    unsigned char bTemp;
	
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    // Transfer the temporary RAM array into the target
    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    SetSDATAStrong();
    while(bTargetDataPtr < SECURITY_BYTES_PER_BANK)
    {
        //bTemp = abTargetDataOUT[bTargetDataPtr];
        bTemp = abTargetDataOUT_secure[bTargetDataPtr];

        SendByte(write_byte_start,4);
        SendByte(bTargetAddress, 7);

        SendByte(bTemp, 8);
        SendByte(write_byte_end, 3);

        // SendBytes() uses MSBits, so increment the address by '2' to put
        // the 0..n address into the seven MSBit locations
        bTargetAddress += 2;				//PTJ: inc by 2 in order to support a 128 byte address space
        bTargetDataPtr++;
    }

    SendVector(secure, num_bits_secure);	//PTJ:

    if (fIsError = fDetectHiLoTransition()) {
		printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
        return(SECURITY_ERROR);
    }

    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);
    return(PASS);
}


// ============================================================================
// PTJ: fReadSecurity()
// This reads from SM0 with Read Supervisory SPC command.
// Need to have SPC Test Mode enabled before using these commands?
// Returns:
//     0 if successful
//     __________ if timed out on handshake to the device.
// ============================================================================
signed char fReadSecurity(void)
{
///	printk( "[TSP] %s, %d\n", __func__, __LINE__);

    SendVector(SPCTestMode_enable, num_bits_SPCTestMode_enable);

	bTargetAddress = 0x00;
	while(bTargetAddress < (SECURITY_BYTES_PER_BANK * 2)) {			//PTJ: we do SECURITY_BYTES_PER_BANK * 2 because we bTargetAddress += 2

		//PTJ: TSYNC Enable
		SendVector(tsync_enable, num_bits_tsync_enable);

		SendVector(read_security_pt1, num_bits_read_security_pt1);	//PTJ:
	    // Set the drive here because SendByte() does not.
    	SetSDATAStrong();
	    SendByte(bTargetAddress,7);											//PTJ: hardcode MSb of address as 0 in bit stream
	    SendVector(read_security_pt1_end, num_bits_read_security_pt1_end);

	    //PTJ: TSYNC Disable
	    SendVector(tsync_disable, num_bits_tsync_disable);

	    SendVector(read_security_pt2, num_bits_read_security_pt2);

		SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

	    SendVector(read_security_pt3, num_bits_read_security_pt3);

		SetSDATAStrong();
        SendByte(bTargetAddress,7);

		SendVector(read_security_pt3_end, num_bits_read_security_pt3_end);

	    SendVector(wait_and_poll_end, num_bits_wait_and_poll_end);

		bTargetAddress +=2;
    }

    bTargetAddress = 0x00;
    bTargetDataPtr = 0x00;

    while(bTargetAddress < (SECURITY_BYTES_PER_BANK * 2)) {			//PTJ: we do SECURITY_BYTES_PER_BANK * 2 because we bTargetAddress += 2

        //Send Read Byte vector and then get a byte from Target
        SendVector(read_byte_v, 4);
        // Set the drive here because SendByte() does not
        SetSDATAStrong();
        SendByte(bTargetAddress,7);

        RunClock(2);       // Run two SCLK cycles between writing and reading
        SetSDATAHiZ();     // Set to HiZ so Target can drive SDATA
        bTargetDataIN = bReceiveByte();

        RunClock(1);
        SendVector(read_byte_v + 1, 1);     // Send the ReadByte Vector End

        // Test the Byte that was read from the Target against the original
        // value (already in the 128-Byte array "abTargetDataOUT[]"). If it
        // matches, then bump the address & pointer,loop-back and continue.
        // If it does NOT match abort the loop and return and error.
        //if (bTargetDataIN != abTargetDataOUT[bTargetDataPtr])
        if (bTargetDataIN != abTargetDataOUT_secure[bTargetDataPtr])
        {
			printk( "[TSP] bTargetDataIN = %x\n", bTargetDataIN);
			printk( "[TSP] abTargetDataOUT_secure[%d] = %x\n", bTargetDataPtr, abTargetDataOUT_secure[bTargetDataPtr]);
			        
        	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
            return(BLOCK_ERROR);
        }

        // Increment the address by two to accomodate 7-Bit addressing
        // (puts the 7-bit address into MSBit locations for "SendByte()").
        bTargetDataPtr++;
        bTargetAddress += 2;
    }

    return(PASS);
}




//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
/////
/////						Main.c
/////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


/* ############################################################################
   ###################  CRITICAL PROJECT CONSTRAINTS   ########################
   ############################################################################

   ISSP programming can only occur within a temperature range of 5C to 50C.
   - This project is written without temperature compensation and using
     programming pulse-widths that match those used by programmers such as the
     Mini-Prog and the ISSP Programmer.
     This means that the die temperature of the PSoC device cannot be outside
     of the above temperature range.
     If a wider temperature range is required, contact your Cypress Semi-
     conductor FAE or sales person for assistance.

   The project can be configured to program devices at 5V or at 3.3V.
   - Initialization of the device is different for different voltages. The
     initialization is hardcoded and can only be set for one voltage range.
     The supported voltages ranges are 3.3V (3.0V to 3.6V) and 5V (4.75V to
     5.25V). See the device datasheet for more details. If varying voltage
     ranges must be supported, contact your Cypress Semiconductor FAE or sales
     person for assistance.
   - ISSP programming for the 2.7V range (2.7V to 3.0V) is not supported.

   This program does not support programming all PSoC Devices
   - It does not support obsoleted PSoC devices. A list of devices that are
     not supported is shown here:
         CY8C22x13 - not supported
         CY8C24x23 - not supported (CY8C24x23A is supported)
         CY8C25x43 - not supported
         CY8C26x43 - not supported
   - It does not suport devices that have not been released for sale at the
     time this version was created. If you need to ISSP program a newly released
     device, please contact Cypress Semiconductor Applications, your FAE or
     sales person for assistance.
     The CY8C20x23 devices are not supported at the time of this release.

   ############################################################################
   ##########################################################################*/


/* (((((((((((((((((((((((((((((((((((((())))))))))))))))))))))))))))))))))))))
 PSoC In-System Serial Programming (ISSP) Template
 This PSoC Project is designed to be used as a template for designs that
 require PSoC ISSP Functions.

 This project is based on the AN2026 series of Application Notes. That app
 note should be referenced before any modifications to this project are made.

 The subroutines and files were created in such a way as to allow easy cut &
 paste as needed. There are no customer-specific functions in this project.
 This demo of the code utilizes a PSoC as the Host.

 Some of the subroutines could be merged, or otherwise reduced, but they have
 been written as independently as possible so that the specific steps involved
 within each function can easily be seen. By merging things, some code-space
 savings could be realized.

 As is, and with all features enabled, the project consumes approximately 3500
 bytes of code space, and 19-Bytes of RAM (not including stack usage). The
 Block-Verify requires a 64-Byte buffer for read-back verification. This same
 buffer could be used to hold the (actual) incoming program data.

 Please refer to the compiler-directives file "directives.h" to see the various
 features.

 The pin used in this project are assigned as shown below. The HOST pins are
 arbitrary and any 3 pins could be used (the masks used to control the pins
 must be changed). The TARGET pins cannot be changed, these are fixed function
 pins on the PSoC.
 The PWR pin is used to provide power to the target device if power cycle
 programming mode is used. The compiler directive RESET_MODE in ISSP_directives.h
 is used to select the programming mode. This pin could control the enable on
 a voltage regulator, or could control the gate of a FET that is used to turn
 the power to the PSoC on.
 The TP pin is a Test Point pin that can be used signal from the host processor
 that the program has completed certain tasks. Predefined test points are
 included that can be used to observe the timing for bulk erasing, block
 programming and security programming.

      SIGNAL  HOST  TARGET
      ---------------------
      SDATA   P1.0   P1.0
      SCLK    P1.1   P1.1
      XRES    P2.0   XRES
      PWR     P2.1   Vdd
      TP      P0.7   n/a

 For test & demonstration, this project generates the program data internally.
 It does not take-in the data from an external source such as I2C, UART, SPI,
 etc. However, the program was written in such a way to be portable into such
 designs. The spirit of this project was to keep it stripped to the minimum
 functions required to do the ISSP functions only, thereby making a portable
 framework for integration with other projects.

 The high-level functions have been written in C in order to be portable to
 other processors. The low-level functions that are processor dependent, such
 as toggling pins and implementing specific delays, are all found in the file
 ISSP_Drive_Routines.c. These functions must be converted to equivalent
 functions for the HOST processor.  Care must be taken to meet the timing
 requirements when converting to a new processor. ISSP timing information can
 be found in Application Note AN2026.  All of the sections of this program
 that need to be modified for the host processor have "PROCESSOR_SPECIFIC" in
 the comments. By performing a "Find in files" using "PROCESSOR_SPECIFIC" these
 sections can easily be identified.

 The variables in this project use Hungarian notation. Hungarian prepends a
 lower case letter to each variable that identifies the variable type. The
 prefixes used in this program are defined below:
  b = byte length variable, signed char and unsigned char
  i = 2-byte length variable, signed int and unsigned int
  f = byte length variable used as a flag (TRUE = 0, FALSE != 0)
  ab = an array of byte length variables


 After this program has been ported to the desired host processor the timing
 of the signals must be confirmed.  The maximum SCLK frequency must be checked
 as well as the timing of the bulk erase, block write and security write
 pulses.

 The maximum SCLK frequency for the target device can be found in the device
 datasheet under AC Programming Specifications with a Symbol of "Fsclk".
 An oscilloscope should be used to make sure that no half-cycles (the high
 time or the low time) are shorter than the half-period of the maximum
 freqency. In other words, if the maximum SCLK frequency is 8MHz, there can be
 no high or low pulses shorter than 1/(2*8MHz), or 62.5 nsec.

 The test point (TP) functions, enabled by the define USE_TP, provide an output
 from the host processor that brackets the timing of the internal bulk erase,
 block write and security write programming pulses. An oscilloscope, along with
 break points in the PSoC ICE Debugger should be used to verify the timing of
 the programming.  The Application Note, "Host-Sourced Serial Programming"
 explains how to do these measurements and should be consulted for the expected
 timing of the erase and program pulses.

/* ############################################################################
   ############################################################################

(((((((((((((((((((((((((((((((((((((()))))))))))))))))))))))))))))))))))))) */



/*----------------------------------------------------------------------------
//                               C main line
//----------------------------------------------------------------------------
*/

//#include <m8c.h>        // part specific constants and macros
//#include "PSoCAPI.h"    // PSoC API definitions for all User Modules


// ------ Declarations Associated with ISSP Files & Routines -------
//     Add these to your project as needed.
//#include "ISSP_extern.h"
//#include "ISSP_directives.h"
//#include "ISSP_defs.h"
//#include "ISSP_errors.h"
//#include "Device.h"
/* ------------------------------------------------------------------------- */
unsigned char bBankCounter;
unsigned int  iBlockCounter;
unsigned int  iChecksumData;
unsigned int  iChecksumTarget;

unsigned int  wBinaryAddress;

/* ========================================================================= */
// ErrorTrap()
// Return is not valid from main for PSOC, so this ErrorTrap routine is used.
// For some systems returning an error code will work best. For those, the
// calls to ErrorTrap() should be replaced with a return(bErrorNumber). For
// other systems another method of reporting an error could be added to this
// function -- such as reporting over a communcations port.
/* ========================================================================= */
void ErrorTrap(unsigned char bErrorNumber)
{
	printk( "[TSP] %s, %d : ErrorNumber = %d\n", __func__, __LINE__, bErrorNumber);

#ifndef RESET_MODE
    // Set all pins to highZ to avoid back powering the PSoC through the GPIO
    // protection diodes.
    SetSCLKHiZ();
    SetSDATAHiZ();
    // If Power Cycle programming, turn off the target
    RemoveTargetVDD();
#endif


#ifdef TX_ON
    TX8SW_CPutString("\r\nErrorTrap");
	TX8SW_PutSHexByte(bErrorNumber);
#endif

    //while (1);
    // return(bErrorNumbers);
}

void Delay_int(unsigned int n)  // by KIMC
{
    while(n)
    {
        //asm("nop");
        n -= 1;
    }
}


/* ========================================================================= */
/* MAIN LOOP                                                                 */
/* Based on the diagram in the AN2026                                        */
/* ========================================================================= */
unsigned char make2ChTo1(unsigned char hi, unsigned char lo)
{
    unsigned char ch;
    
    if(hi == 'A' || hi == 'a')
        hi = 0xa;
    else if(hi == 'B' || hi == 'b')
        hi = 0xb;
    else if(hi == 'C' || hi == 'c')
        hi = 0xc;
    else if(hi == 'D' || hi == 'd')
        hi = 0xd;
    else if(hi == 'E' || hi == 'e')
        hi = 0xe;
    else if(hi == 'F' || hi == 'f')
        hi = 0xf;
    else
        hi = hi;

    if(lo == 'A' || lo == 'a')
        lo = 0xa;
    else if(lo == 'B' || lo == 'b')
        lo = 0xb;
    else if(lo == 'C' || lo == 'c')
        lo = 0xc;
    else if(lo == 'D' || lo == 'd')
        lo = 0xd;
    else if(lo == 'E' || lo == 'e')
        lo = 0xe;
    else if(lo == 'F' || lo == 'f')
        lo = 0xf;
    else
        lo = lo;
    
    ch = ((hi&0x0f) << 4) | (lo & 0x0f);

    return ch;
}


unsigned int load_tst200_frimware_data(int HW_ver)
{
	int i,j;
	extern unsigned char tsp_special_update;

	mm_segment_t oldfs;
//	struct file *filp;
	int fd;
	uint8_t* buffer;
	int result = -1;

	unsigned int nBinary_length = 0;
	unsigned int firmwareline, onelinelength;
	unsigned char temp_onelinedata[128];


	if(tsp_special_update == 0)//normal firmware update from phone-binary
	{
		if(HW_ver == 2)
		{
			for(i=0; i<512; i++)
				for(j=0; j<64; j++)
					firmData[i][j] = BinaryData_HW02[i*64 + j];

			return PASS;
		}
		else if(HW_ver == 3)
		{
			for(i=0; i<512; i++)
				for(j=0; j<64; j++)
					firmData[i][j] = BinaryData_HW03[i*64 + j];

			return PASS;
		}
		else if(HW_ver == 4)
		{
			for(i=0; i<512; i++)
				for(j=0; j<64; j++)
					firmData[i][j] = BinaryData_HW04[i*64 + j];

			return PASS;
		}
		else if(HW_ver == 5)
		{
			for(i=0; i<512; i++)
				for(j=0; j<64; j++)
					firmData[i][j] = BinaryData_HW05[i*64 + j];

			return PASS;
		}
		else
		{
			return NO_FIRMWARE_ERROR;
		}
	}
	else if(tsp_special_update == 1)//special firmware update from t-flash
	{
		printk("[TSP] %s, %d\n", __func__, __LINE__ );

		oldfs = get_fs();
		set_fs (KERNEL_DS);   /* set KERNEL address space */

		/* - Get file Size */
		buffer = kmalloc(72192, GFP_KERNEL);/*141*512*/

		if(buffer == NULL) 
		{
			printk("[TSP] firmware_down_using_sdcard : alllocate mem fail\n");
			result = FILE_ACCESS_ERROR;
			goto error;
		}

		if((fd = sys_open("/mnt/sdcard/luisa_tsp.hex", O_RDONLY | O_LARGEFILE, 0)) > 0)
		{
			if(sys_read(fd, buffer, 72192) > 0) 
			{
				sys_close(fd);
				printk("[TSP] firmware_down_using_sdcard : read file success\n");	
			}
			else
			{
				sys_close(fd);
				result = FILE_ACCESS_ERROR;
				printk("[TSP] firmware_down_using_sdcard : file read fail\n");				
				goto error;				
			}
		}
		else
		{
			result = FILE_ACCESS_ERROR;
			printk("[TSP] firmware_down_using_sdcard : file open fail\n");
			goto error;
		}
		
		printk("\n[TSP] firmware_down_using_sdcard : firmware_data : START \n");
		
		for(firmwareline=0; firmwareline<512; firmwareline++)
		{
			i = 0;

			strncpy(temp_onelinedata, buffer + 141*firmwareline + 9, 128);
			for(onelinelength=0; onelinelength<64; onelinelength++)
			{
				firmData[firmwareline][onelinelength] = make2ChTo1(temp_onelinedata[i], temp_onelinedata[i+1]);
				i += 2;
			}
		}

		printk("\n[TSP] firmware_down_using_sdcard : firmware_data : END \n");
		
		/*
		for(i=0; i<512; i++)
		{
			for(j=0; j<64; j++)
				printk("%x ", firmData[i][j]);

			printk("\n");
		}

		printk("\n[TSP] firmware_data : END \n");
		*/
		
		result = PASS;

		error:
			if(buffer)
				kfree(buffer);

			set_fs(oldfs);


		return result;
	}
	else
	{
		return NO_FIRMWARE_ERROR;
	}



#if 0
	//unsigned char *pData = NULL;
	//unsigned char *temp;
	unsigned int nBinary_length = 0;
	unsigned int i, j, firmwareline, onelinelength;
	unsigned int TSP_FILE_SIZE = 256;
	unsigned int TSP_blk = 0;

	FSFILE* fp;
	int  nRead;
	char pathName[256];
	
	extern unsigned char sv_tch_special_update;
	extern unsigned char tch_ven_id;
	extern unsigned char tch_mod_rev;
	extern unsigned char tch_fw_ver;

	OSTASK_Sleep(100);
	//Delay10us(1000*100);//100ms

	if(sv_tch_special_update == 1)
		strcpy(pathName, "/Mount/Mmc/collins_test.hex");
	else if(tch_mod_rev == 0x01)
		strcpy(pathName, "/SystemFS/Driver/TouchFirmware/TST200/ModRev_01/collins_01_lastest.hex");	
	else
		return NO_FIRMWARE_ERROR;

	OSTASK_Sleep(100);
	//Delay10us(1000*100);//100ms


    //------------------------------
    // Open a file
    //------------------------------

	fp = FS_Open(pathName, "rb");
	
    if (fp == NULL)
    {
//	    FS_Close(fp);
		
        //return MCSDL_RET_FILE_ACCESS_FAILED;
        return FILE_ACCESS_ERROR;
    }

    //------------------------------
    // Read binary file
    //------------------------------

	FS_Read(pData, 1, 141*515, fp);		// Read binary file
	


    //------------------------------
    // Close file
    //------------------------------

    FS_Close(fp);



	for(firmwareline=0; firmwareline<515; firmwareline++)
	{
		i = 0;

		if(firmwareline < 512)
		{
			strncpy(temp_onelinedata, pData + 141*firmwareline + 9, 128);
			for(onelinelength=0; onelinelength<64; onelinelength++)
			{
				firmData[firmwareline][onelinelength] = make2ChTo1(temp_onelinedata[i], temp_onelinedata[i+1]);
				i += 2;
			}
		}
		else if(firmwareline == 512)
		{
		}
		else
		{
			strncpy(temp_onelinedata, pData + 141*(firmwareline-1) + 17 + 9, 128);
			for(onelinelength=0; onelinelength<64; onelinelength++)
			{
				firmData[firmwareline-1][onelinelength] = make2ChTo1(temp_onelinedata[i], temp_onelinedata[i+1]);
				i += 2;
			}
		}
	}

	for(j=0; j<128; j++)
	{
		if(j<64)
			abTargetDataOUT_secure[j] = firmData[512][j];
		else
			abTargetDataOUT_secure[j] = firmData[513][j-64];
	}
#endif	
	return PASS;
}


int cypress_update(HW_ver)
{
	unsigned int i;
	unsigned int aIndex;
	
    unsigned int bTry=0;
    // -- This example section of commands show the high-level calls to -------
    // -- perform Target Initialization, SilcionID Test, Bulk-Erase, Target ---
    // -- RAM Load, FLASH-Block Program, and Target Checksum Verification. ----

	if (fIsError = load_tst200_frimware_data(HW_ver))
    {
        ErrorTrap(fIsError);
		return fIsError;
    }

    // >>>> ISSP Programming Starts Here <<<<

    #ifdef TX_ON
    TX8SW_Start();

    TX8SW_PutCRLF();
    TX8SW_PutCRLF();
    TX8SW_PutCRLF();
    TX8SW_PutCRLF();
    TX8SW_CPutString("\r\nStart");
    #endif

    // Acquire the device through reset or power cycle
    #ifdef RESET_MODE
        for (bTry=0 ; bTry < 5 ; bTry++)    // when failure, retrying the initialization.
        {
            fIsError = fXRESInitializeTargetForISSP();

            if(!fIsError)
            {
                break;
            }
            else
            {
                ReStartTarget();
            }
        }
        if (fIsError)
        {
            ErrorTrap(fIsError);
        }
    #else
        // Initialize the Host & Target for ISSP operations
        for (bTry=0 ; bTry < 5 ; bTry++)    // when failure, retrying the initialization.
        {
            fIsError = fPowerCycleInitializeTargetForISSP();

            if(!fIsError)
            {
                break;
            }
            else
            {
                ReStartTarget();
            }
        }
        if (fIsError)
        {
            ErrorTrap(fIsError);
			return fIsError;
        }

    #endif  // initialization mode

    #ifdef TX_ON
	TX8SW_CPutString("\r\n InitializeTargetForISSP");
    #endif

    // Run the SiliconID Verification, and proceed according to result.
    if (fIsError = fVerifySiliconID())
    {
        ErrorTrap(fIsError);
		return fIsError;
    }

    #ifdef TX_ON
	TX8SW_CPutString("\r\n fVerifySiliconID");
    #endif

	/* Disable watchdog and interrupt */
	//TchDrv_DownloadDisableIRQ();	// Disable Baseband touch interrupt ISR.
	//TchDrv_DownloadDisableWD();		// Disable Baseband watchdog timer

    // Bulk-Erase the Device.
    if (fIsError = fEraseTarget())
    {
        ErrorTrap(fIsError);
		goto MCSDL_DOWNLOAD_FINISH;
    }

    #ifdef TX_ON
    TX8SW_CPutString("\r\n fEraseTarget");
    TX8SW_CPutString("\r\n Program Flash Blocks Start");
    #endif

    //==============================================================//
    // Program Flash blocks with predetermined data. In the final application
    // this data should come from the HEX output of PSoC Designer.
    iChecksumData = 0;     // Calculte the device checksum as you go
    wBinaryAddress = 0 ;

    for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)		//PTJ: NUM_BANKS should be 1 for Krypton
    {
        for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
        {

            //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1, and TSYNC Enable
            if (fIsError = fReadWriteSetup())
            {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;				
            }
            #ifdef TX_ON
			TX8SW_PutChar('#');
            #endif

            //----------------------------------------------------------------------------//
            // need to modify - user code
#if 1//please check
			aIndex = iBlockCounter*2;
			
			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}
#endif
            //LoadProgramData(bBankCounter, (unsigned char)iBlockCounter);
            iChecksumData += iLoadTarget();
            //----------------------------------------------------------------------------//

            if (fIsError = fProgramTargetBlock(bBankCounter,(unsigned char)iBlockCounter))
            {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;				
            }

            // READ-STATUS after PROGRAM-AND-VERIFY
            if (fIsError = fReadStatus())
            {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;				
            }
        }
    }

    #ifdef TX_ON
	TX8SW_CPutString("\r\n Program Flash Blocks End");
    #endif

    #if 1   // Verify

        #ifdef TX_ON
		TX8SW_CPutString("\r\n Verify Start");
        #endif
        //=======================================================//
        //PTJ: Doing Verify
        //PTJ: this code isnt needed in the program flow because we use PROGRAM-AND-VERIFY (ProgramAndVerify SROM Func)
        //PTJ: which has Verify built into it.
    // Verify included for completeness in case host desires to do a stand-alone verify at a later date.

        wBinaryAddress = 0;

        for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
        {
            for (iBlockCounter=0; iBlockCounter<BLOCKS_PER_BANK; iBlockCounter++)
            {
                //LoadProgramData(bBankCounter, (unsigned char)iBlockCounter);
#if 1//please check
			aIndex = iBlockCounter*2;
			
			for(i=0;i<TARGET_DATABUFF_LEN;i++)
			{
				if(i<64)
				{
					abTargetDataOUT[i] = firmData[aIndex][i];
				}
				else
				{
					abTargetDataOUT[i] = firmData[aIndex+1][i-64];
				}
			}
#endif

                //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1, and TSYNC Enable
                if (fIsError = fReadWriteSetup())
                {
                    ErrorTrap(fIsError);
					goto MCSDL_DOWNLOAD_FINISH;	
                }

                if (fIsError = fVerifySetup(bBankCounter, (unsigned char)iBlockCounter))
                {
                    ErrorTrap(fIsError);
					goto MCSDL_DOWNLOAD_FINISH;
                }

                #ifdef TX_ON
                    TX8SW_PutChar('.');
                #endif

                if (fIsError = fReadStatus())
                {
                    ErrorTrap(fIsError);
					goto MCSDL_DOWNLOAD_FINISH;
                }

                //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1, and TSYNC Enable
                if (fIsError = fReadWriteSetup())
                {
                    ErrorTrap(fIsError);
					goto MCSDL_DOWNLOAD_FINISH;
                }

                if (fIsError = fReadByteLoop())
                {
                    ErrorTrap(fIsError);
					goto MCSDL_DOWNLOAD_FINISH;
                }
            }
        }

        #ifdef TX_ON
		TX8SW_CPutString("\r\n Verify End");
        #endif

    #endif // end Verify


    #if 1   // program security

        #ifdef TX_ON
		TX8SW_CPutString("\r\n Security Start");
        #endif

        //=======================================================//
        // Program security data into target PSoC. In the final application this
        // data should come from the HEX output of PSoC Designer.
        for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
        {
            //PTJ: READ-WRITE-SETUP used here to select SRAM Bank 1
            if (fIsError = fReadWriteSetup()) {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;
            }

            //----------------------------------------------------------------------------//
            // need to modify - user code
			#if 1//please check            
            // Load one bank of security data from hex file into buffer
            if (fIsError = fLoadSecurityData(bBankCounter))
            {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;
            }
			#endif
            //----------------------------------------------------------------------------//

            // Secure one bank of the target flash
            if (fIsError = fSecureTargetFlash())
            {
                ErrorTrap(fIsError);
				goto MCSDL_DOWNLOAD_FINISH;
            }
        }
		/*
        if (fIsError = fReadSecurity())
        {
            ErrorTrap(fIsError);
			goto MCSDL_DOWNLOAD_FINISH;
        }
        */

        #ifdef TX_ON
            TX8SW_CPutString("\r\n End Security data");
        #endif

    #endif  // program security

    #ifdef TX_ON
	TX8SW_CPutString("\r\n CheckSum Start");
    #endif


    //=======================================================//
    //PTJ: Doing Checksum
    iChecksumTarget = 0;
    for (bBankCounter=0; bBankCounter<NUM_BANKS; bBankCounter++)
    {
        if (fIsError = fAccTargetBankChecksum(&iChecksumTarget))
        {
        	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
			ErrorTrap(fIsError);
			goto MCSDL_DOWNLOAD_FINISH;
        }
    }
	
    #ifdef TX_ON
    TX8SW_CPutString("\r\n Checksum : iChecksumTarget (0x");
    TX8SW_PutSHexInt(iChecksumTarget);
    TX8SW_CPutString("), iChecksumData (0x");
    TX8SW_PutSHexInt(iChecksumData);
    TX8SW_CPutString(")");
    #endif

    if ((iChecksumTarget & 0xFFFF) != (iChecksumData & 0xFFFF))
    {
    	printk( "[TSP] %s, %d : Error\n", __func__, __LINE__);
    	fIsError = CHECKSUM_ERROR;
        ErrorTrap(fIsError);
		goto MCSDL_DOWNLOAD_FINISH;
    }

    #ifdef TX_ON
	TX8SW_CPutString("\r\n Doing Checksum");
    #endif


    // *** SUCCESS ***
    // At this point, the Target has been successfully Initialize, ID-Checked,
    // Bulk-Erased, Block-Loaded, Block-Programmed, Block-Verified, and Device-
    // Checksum Verified.


    // You may want to restart Your Target PSoC Here.
    ReStartTarget();

    #ifdef TX_ON
	TX8SW_CPutString("\r\n ReStartTarget");
    #endif

#if 0

    while(1) {
        // Continue with other functions or return to the calling routine
    }
#endif	

MCSDL_DOWNLOAD_FINISH :

//	Delay10us(50*1000);
//	Delay10us(50*1000);
	
	/* Enable watchdog and interrupt */
//	TchDrv_DownloadEnableWD();
//	TchDrv_DownloadEnableIRQ();

//	Delay10us(50*1000);
//	Delay10us(50*1000);
//	Delay10us(50*1000);
//	Delay10us(50*1000);

	ReStartTarget();

	return fIsError;

}

