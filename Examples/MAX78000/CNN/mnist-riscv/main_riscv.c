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

// mnist-riscv
// Created using ./ai8xize.py --verbose --log --test-dir sdk/Examples/MAX78000/CNN --prefix mnist-riscv --checkpoint-file trained/ai85-mnist-qat8-q.pth.tar --config-file networks/mnist-chw-ai85.yaml --softmax --device MAX78000 --compact-data --mexpress --timer 0 --display-checkpoint --riscv --riscv-debug

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "fcr_regs.h"
#include "sema_regs.h"
#include "cnn.h"
#include "sampledata.h"

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
  printf("\n*** FAIL ***\n\n");
  while (1);
}

// 1-channel 28x28 data input (784 bytes / 196 32-bit words):
// CHW 28x28, channel 0
__attribute__ ((section(".rvflash_section")))
static const uint32_t input_0[] = SAMPLE_INPUT_0;

void load_input(void)
{
  // This function loads the sample data input -- replace with actual data

  memcpy32((uint32_t *) 0x50400000, input_0, 196);
}

// Expected output of layer 4 for mnist-riscv given the sample input
int check_output(void)
{
  if ((*((volatile uint32_t *) 0x50401000)) != 0xfffecf58) return CNN_FAIL; // 0,0,0
  if ((*((volatile uint32_t *) 0x50401004)) != 0x00016cc1) return CNN_FAIL; // 0,0,1
  if ((*((volatile uint32_t *) 0x50401008)) != 0x0000b1ac) return CNN_FAIL; // 0,0,2
  if ((*((volatile uint32_t *) 0x5040100c)) != 0x0000749a) return CNN_FAIL; // 0,0,3
  if ((*((volatile uint32_t *) 0x50409000)) != 0xffff7268) return CNN_FAIL; // 0,0,4
  if ((*((volatile uint32_t *) 0x50409004)) != 0xfffe0062) return CNN_FAIL; // 0,0,5
  if ((*((volatile uint32_t *) 0x50409008)) != 0xfffbd7f1) return CNN_FAIL; // 0,0,6
  if ((*((volatile uint32_t *) 0x5040900c)) != 0x00065b68) return CNN_FAIL; // 0,0,7
  if ((*((volatile uint32_t *) 0x50411000)) != 0xfffdbad1) return CNN_FAIL; // 0,0,8
  if ((*((volatile uint32_t *) 0x50411004)) != 0x000061f1) return CNN_FAIL; // 0,0,9

  return CNN_OK;
}

// Classification layer:
static int32_t ml_data[CNN_NUM_OUTPUTS];
static q15_t ml_softmax[CNN_NUM_OUTPUTS];

void softmax_layer(void)
{
  cnn_unload((uint32_t *) ml_data);
  softmax_q17p14_q15((const q31_t *) ml_data, CNN_NUM_OUTPUTS, ml_softmax);
}

int main(void)
{
  int i;
  int digs, tens;

  Debug_Init(); // Set up RISCV JTAG
  MXC_ICC_Enable(MXC_ICC1); // Enable cache

  // Enable peripheral, enable CNN interrupt, turn on CNN clock
  // CNN clock: 50 MHz div 1
  cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);

  printf("\n*** RISC-V CNN Inference Test ***\n");

  cnn_init(); // Bring state machine into consistent state
  cnn_load_weights(); // Load kernels
  cnn_load_bias();
  cnn_configure(); // Configure state machine
  load_input(); // Load data input
  cnn_start(); // Start CNN processing

  while (cnn_time == 0)
    asm volatile("wfi"); // Wait for CNN

  if (check_output() != CNN_OK) fail();
  softmax_layer();

  printf("\n*** PASS ***\n\n");

#ifdef CNN_INFERENCE_TIMER
  printf("Approximate inference time: %u us\n\n", cnn_time);
#endif

  cnn_disable(); // Shut down CNN clock, disable peripheral

  printf("Classification results:\n");
  for (i = 0; i < CNN_NUM_OUTPUTS; i++) {
    digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
    tens = digs % 10;
    digs = digs / 10;
    printf("[%7d] -> Class %d: %d.%d%%\n", ml_data[i], i, digs, tens);
  }

  // Signal the Cortex-M4
  MXC_SEMA->irq0 = MXC_F_SEMA_IRQ0_EN | MXC_F_SEMA_IRQ0_CM4_IRQ;

  return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 10,883,968 ops (10,751,808 macc; 128,576 comp; 3,584 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 71,148 bytes out of 442,368 bytes total (16%)
  Bias memory:   10 bytes out of 2,048 bytes total (0%)
*/

