/**************************************************************************************************
  Filename:       simpleBLECentral_Main.c
  Revised:        $Date: 2011-02-24 15:48:00 -0800 (Thu, 24 Feb 2011) $
  Revision:       $Revision: 11 $

  Description:    This file contains the main and callback functions for
                  the Simple BLE Central sample application.


  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */
#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/



typedef unsigned int  uint;
typedef unsigned char uchar;


#define LED2 P1_1       // P1.1口控制LED1

//static uint count;             //用于定时器计数
//
//int osal_continuous_scan_flag = 0;


//extern uint8 GAPCentralRole_StartDiscovery( uint8 mode, uint8 activeScan, uint8 whiteList );



/****************************************************************************
* 名    称: InitLed()
* 功    能: 设置LED灯相应的IO口
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitLed(void)
{
    P1DIR |= 0x02;           //P1.0定义为输出
    LED2 = 0;                //使LED1灯下电默认为熄灭     
}

/****************************************************************************
* 名    称: InitT3()
* 功    能: 定时器初始化，系统不配置工作时钟时默认是2分频，即16MHz
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void InitT3()
{     
    T3CTL |= 0x08 ;          //开溢出中断     
    T3IE = 1;                //开总中断和T3中断
    T3CTL |= 0xE0;           //128分频,128/16000000*N=0.5S,N=62500
    T3CTL &= ~0x03;          //自动重装 00－>0xff  62500/255=245(次)
    T3CTL |= 0x10;           //启动
    EA = 1;                  //开总中断
}




/**************************************************************************************************
 * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
    
    /* Initialize hardware */
    HAL_BOARD_INIT();
    
    // Initialize board I/O
    InitBoard( OB_COLD );
    
    /* Initialze the HAL driver */
    HalDriverInit(); 
    
    /* Initialize NV system */
    osal_snv_init();
    
    /* Initialize LL */
    
    /* Initialize the operating system */
    osal_init_system();
    
    /* Enable interrupts */
    HAL_ENABLE_INTERRUPTS();
    
    // Final board initialization
    InitBoard( OB_READY );
    
#if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
#endif
    
    InitLed();		     //设置LED灯相应的IO口
    InitT3();            //设置T3相应的寄存器
    
    /* Start OSAL */
    osal_start_system(); // No Return from here
    
    
    

    return 0;
}

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
