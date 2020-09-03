/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************/

#include	"hpf.h"

static INT16	x, Prev_x, R;
static INT32	y, Prev_y;

/*********************************************************************************
*
*   Prototype:      void HPF_init()
*
*   Description:    The function initializes states of high pass filter 
*
*   Parameters:    
*
*   Return:         None
*
*   Globals:    	None
*
*   Limitations:    None
*
************************************************************************************/
void HPF_init(void)
{
 	R = 32604; //.995
	x = 0;
	y = 0;
	Prev_y = y;
	Prev_x = x;
} 

/*********************************************************************************
*
*   Prototype:      INT16 HPF( INT16 input)
*
*   Description:    The function is a 1-d order IIR high pass filter (cutoff frequency - 100 Hz)
*					input and output are 10 bits each
*   Parameters:    INT16 input samples
*				
*    filter is  y(n)=x(n)-x(n-1)+Ay(n-1) and A =.995*2^15
*   Return:         output
*
*
************************************************************************************/
INT16 HPF(INT16 input)
{
	INT16 A;
	INT16 output;
	INT32 tmp;

	x=input;

	tmp = (R * Prev_y);
	A = (INT16)((tmp + (1 << 14)) >> 15);
	y = x - Prev_x + A;

	// Flow control
	if(y>32767)  //NEW
	y = 32767;    //NEW
	if(y< -32768)  //NEW
	y = -32768;    //NEW
	Prev_y = y;
	Prev_x = x;

//	output = (INT8)((y + (1 << 7)) >> 8);	// want o/p = 8 bits
//	output = (INT16)((y + (1 << 5)) >> 6);	// want o/p = 10 bits  OLD
//   	output = (INT16)((y + (1 << 4)) >> 5);	// want o/p = 10 bits  NEW

	output = (INT16)y;
	// clipping: Note for RTL: make sure output is clipped to 10 bits
//	if(output > 511)
//		output = 511;
//	else if(output < -512)
//		output = -512;

	return	(output);		
}
