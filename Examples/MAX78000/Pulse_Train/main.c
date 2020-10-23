/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*
******************************************************************************/

/**
 * @file main.c
 * @brief Configures and starts four different pulse trains on GPIO LEDs.
 * @details PT0 setup as 2Hz continuous signal that outputs 10110b
 *          PT1 setup as 10Hz continuous square wave
 *
 * @note Interrupts for pulse trains are enabled but the interrupt handler only clears the flags.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>

#include <mxc.h>

/***** Definitions *****/
#define    ALL_PT    0x03
#define		SQ_WV_HZ			10			//Square wave frequency
#define		CONT_WV_BPS 		2			//Continuous wave bits/sec
#define		CONT_WV_PATTERN		0x16		//Continuous wave bit pattern (pattern will only output 5 least significant bits)

#define		FTHR_OUT_SQ			0			//Output square
#define		FTHR_OUT_CONT		1

#if (FTHR_OUT_SQ && FTHR_OUT_CONT) || !(FTHR_OUT_SQ || FTHR_OUT_CONT)
#error "Please select either FTHR_OUT_SQ or FTHR_OUT_CONT."
#endif

/***** Globals *****/

/***** Functions *****/

// *****************************************************************************
void PT_IRQHandler(void)
{
    printf("flags = 0x%08x\n", MXC_PT_GetFlags());
    
    MXC_PT_ClearFlags(ALL_PT);
}

// *****************************************************************************
void ContinuousPulseTrain(int ch)
{
    //Setup GPIO to PT output function
    //GPIO P0.18 uses PT0
    
    //setup PT configuration
    mxc_pt_cfg_t ptConfig;
    ptConfig.channel = ch;                  //PT0
    ptConfig.bps = CONT_WV_BPS;             //bit rate
    ptConfig.ptLength = 5;                          //bits
    ptConfig.pattern = CONT_WV_PATTERN;
    ptConfig.loop = 0;                              //continuous loop
    ptConfig.loopDelay = 0;
    
    MXC_PT_Config(&ptConfig);
    
    if(ch) {
    	MXC_PT_Start(MXC_F_PTG_ENABLE_PT1);
    }
    else {
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT0);
}
}

// *****************************************************************************
void SquareWave(int ch)
{
    //Setup GPIO to PT output function
    //GPIO P0.19 uses PT1
    
    uint32_t freq = SQ_WV_HZ;			//Hz
    MXC_PT_SqrWaveConfig(ch, freq);     //PT1
    
    if(ch) {
    MXC_PT_Start(MXC_F_PTG_ENABLE_PT1);
}
    else {
    	MXC_PT_Start(MXC_F_PTG_ENABLE_PT0);
    }
}

// *****************************************************************************
int main(void)
{
#ifdef BOARD_EVKIT_V1
    printf("\n***** Pulse Train Demo *****\n");
    printf("PT0 (P0.18) = Outputs continuous pattern of 0x%x at %dbps\n", CONT_WV_PATTERN, CONT_WV_BPS);
    printf("PT1 (P0.19) = Outputs %dHz continuous square wave\n", SQ_WV_HZ);
#else
    printf("\n***** Pulse Train Demo *****\n");
    printf("This example outputs either a continuous pattern of 0x%x at\n", CONT_WV_PATTERN);
    printf("%dbps or a %dHz continuous square wave on PT channel 1 (P0.19).\n", CONT_WV_BPS, SQ_WV_HZ);
#endif
    
    NVIC_EnableIRQ(PT_IRQn);         //enabled default interrupt handler
    MXC_PT_EnableInt(ALL_PT);        //enabled interrupts for all PT
    MXC_PT_Init(MXC_PT_CLK_DIV1);    //initialize pulse trains
    
#ifdef BOARD_EVKIT_V1 						//Using Standard EV Kit
    ContinuousPulseTrain(0);
    SquareWave(1);
#else									//Using Featherboard
	if(FTHR_OUT_CONT) {
		ContinuousPulseTrain(1);
	}
	else {
		SquareWave(1);
	}
#endif
    
    while (1) {}
}
