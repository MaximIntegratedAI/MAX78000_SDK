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

// kws20
// Created using ./ai8xize.py -e --verbose --top-level cnn -L --test-dir sdk/Examples/MAX78000/CNN --prefix kws20 --checkpoint-file trained/ai85-kws20.pth.tar --config-file networks/kws20-hwc.yaml --device 85 --compact-data --mexpress --softmax --embedded-code --display-checkpoint

// Configuring 10 layers:
// Layer 0: 128x128 (HWC/little data), no pooling, conv1d with kernel size 1, stride 1, pad 0, 100x128 output
// Layer 1: 100x128 (HWC/little data), no pooling, conv1d with kernel size 1, stride 1, pad 0, 100x128 output
// Layer 2: 100x128 (HWC/little data), no pooling, conv1d with kernel size 1, stride 1, pad 0, 50x128 output
// Layer 3: 50x128 (HWC/little data), no pooling, conv1d with kernel size 1, stride 1, pad 0, 16x128 output
// Layer 4: 16x16x8 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 32x16x8 output
// Layer 5: 32x16x8 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x16x8 output
// Layer 6: 64x16x8 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 64x16x8 output
// Layer 7: 64x16x8 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 30x16x8 output
// Layer 8: 30x16x8 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 7x16x8 output
// Layer 9: 7x16x8 (flattened HWC/little data), no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, 21x1x1 output

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_sys.h"
#include "bbfc_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "led.h"
#include "tmr.h"
#include "tornadocnn.h"
#include "weights.h"
#include "sampledata.h"

uint32_t cnn_time; // Stopwatch

void fail(void)
{
  printf("\n*** FAIL ***\n\n");
  while (1);
}

void cnn_wait(void)
{
  while ((*((volatile uint32_t *) 0x50100000) & (1<<12)) != 1<<12) ;
  CNN_COMPLETE; // Signal that processing is complete
  cnn_time = MXC_TMR_SW_Stop(MXC_TMR0);
}

void memcpy32(uint32_t *dst, const uint32_t *src, int n)
{
  while (n-- > 0) {
    *dst++ = *src++;
  }
}

// 128-channel 128x1 data input:
// HWC (little data): 128x1, channels 0 to 3
// HWC (little data): 128x1, channels 64 to 67
static const uint32_t input_0[] = INPUT_0;

// HWC (little data): 128x1, channels 4 to 7
// HWC (little data): 128x1, channels 68 to 71
static const uint32_t input_4[] = INPUT_4;

// HWC (little data): 128x1, channels 8 to 11
// HWC (little data): 128x1, channels 72 to 75
static const uint32_t input_8[] = INPUT_8;

// HWC (little data): 128x1, channels 12 to 15
// HWC (little data): 128x1, channels 76 to 79
static const uint32_t input_12[] = INPUT_12;

// HWC (little data): 128x1, channels 16 to 19
// HWC (little data): 128x1, channels 80 to 83
static const uint32_t input_16[] = INPUT_16;

// HWC (little data): 128x1, channels 20 to 23
// HWC (little data): 128x1, channels 84 to 87
static const uint32_t input_20[] = INPUT_20;

// HWC (little data): 128x1, channels 24 to 27
// HWC (little data): 128x1, channels 88 to 91
static const uint32_t input_24[] = INPUT_24;

// HWC (little data): 128x1, channels 28 to 31
// HWC (little data): 128x1, channels 92 to 95
static const uint32_t input_28[] = INPUT_28;

// HWC (little data): 128x1, channels 32 to 35
// HWC (little data): 128x1, channels 96 to 99
static const uint32_t input_32[] = INPUT_32;

// HWC (little data): 128x1, channels 36 to 39
// HWC (little data): 128x1, channels 100 to 103
static const uint32_t input_36[] = INPUT_36;

// HWC (little data): 128x1, channels 40 to 43
// HWC (little data): 128x1, channels 104 to 107
static const uint32_t input_40[] = INPUT_40;

// HWC (little data): 128x1, channels 44 to 47
// HWC (little data): 128x1, channels 108 to 111
static const uint32_t input_44[] = INPUT_44;

// HWC (little data): 128x1, channels 48 to 51
// HWC (little data): 128x1, channels 112 to 115
static const uint32_t input_48[] = INPUT_48;

// HWC (little data): 128x1, channels 52 to 55
// HWC (little data): 128x1, channels 116 to 119
static const uint32_t input_52[] = INPUT_52;

// HWC (little data): 128x1, channels 56 to 59
// HWC (little data): 128x1, channels 120 to 123
static const uint32_t input_56[] = INPUT_56;

// HWC (little data): 128x1, channels 60 to 63
// HWC (little data): 128x1, channels 124 to 127
static const uint32_t input_60[] = INPUT_60;

void load_input(void)
{
  memcpy32((uint32_t *) 0x50400000, input_0, 256);
  memcpy32((uint32_t *) 0x50408000, input_4, 256);
  memcpy32((uint32_t *) 0x50410000, input_8, 256);
  memcpy32((uint32_t *) 0x50418000, input_12, 256);
  memcpy32((uint32_t *) 0x50800000, input_16, 256);
  memcpy32((uint32_t *) 0x50808000, input_20, 256);
  memcpy32((uint32_t *) 0x50810000, input_24, 256);
  memcpy32((uint32_t *) 0x50818000, input_28, 256);
  memcpy32((uint32_t *) 0x50c00000, input_32, 256);
  memcpy32((uint32_t *) 0x50c08000, input_36, 256);
  memcpy32((uint32_t *) 0x50c10000, input_40, 256);
  memcpy32((uint32_t *) 0x50c18000, input_44, 256);
  memcpy32((uint32_t *) 0x51000000, input_48, 256);
  memcpy32((uint32_t *) 0x51008000, input_52, 256);
  memcpy32((uint32_t *) 0x51010000, input_56, 256);
  memcpy32((uint32_t *) 0x51018000, input_60, 256);
}

// Kernels:
static const uint32_t kernels_0[] = KERNELS_0;
static const uint32_t kernels_1[] = KERNELS_1;
static const uint32_t kernels_2[] = KERNELS_2;
static const uint32_t kernels_3[] = KERNELS_3;
static const uint32_t kernels_4[] = KERNELS_4;
static const uint32_t kernels_5[] = KERNELS_5;
static const uint32_t kernels_6[] = KERNELS_6;
static const uint32_t kernels_7[] = KERNELS_7;
static const uint32_t kernels_8[] = KERNELS_8;
static const uint32_t kernels_9[] = KERNELS_9;
static const uint32_t kernels_10[] = KERNELS_10;
static const uint32_t kernels_11[] = KERNELS_11;
static const uint32_t kernels_12[] = KERNELS_12;
static const uint32_t kernels_13[] = KERNELS_13;
static const uint32_t kernels_14[] = KERNELS_14;
static const uint32_t kernels_15[] = KERNELS_15;
static const uint32_t kernels_16[] = KERNELS_16;
static const uint32_t kernels_17[] = KERNELS_17;
static const uint32_t kernels_18[] = KERNELS_18;
static const uint32_t kernels_19[] = KERNELS_19;
static const uint32_t kernels_20[] = KERNELS_20;
static const uint32_t kernels_21[] = KERNELS_21;
static const uint32_t kernels_22[] = KERNELS_22;
static const uint32_t kernels_23[] = KERNELS_23;
static const uint32_t kernels_24[] = KERNELS_24;
static const uint32_t kernels_25[] = KERNELS_25;
static const uint32_t kernels_26[] = KERNELS_26;
static const uint32_t kernels_27[] = KERNELS_27;
static const uint32_t kernels_28[] = KERNELS_28;
static const uint32_t kernels_29[] = KERNELS_29;
static const uint32_t kernels_30[] = KERNELS_30;
static const uint32_t kernels_31[] = KERNELS_31;
static const uint32_t kernels_32[] = KERNELS_32;
static const uint32_t kernels_33[] = KERNELS_33;
static const uint32_t kernels_34[] = KERNELS_34;
static const uint32_t kernels_35[] = KERNELS_35;
static const uint32_t kernels_36[] = KERNELS_36;
static const uint32_t kernels_37[] = KERNELS_37;
static const uint32_t kernels_38[] = KERNELS_38;
static const uint32_t kernels_39[] = KERNELS_39;
static const uint32_t kernels_40[] = KERNELS_40;
static const uint32_t kernels_41[] = KERNELS_41;
static const uint32_t kernels_42[] = KERNELS_42;
static const uint32_t kernels_43[] = KERNELS_43;
static const uint32_t kernels_44[] = KERNELS_44;
static const uint32_t kernels_45[] = KERNELS_45;
static const uint32_t kernels_46[] = KERNELS_46;
static const uint32_t kernels_47[] = KERNELS_47;
static const uint32_t kernels_48[] = KERNELS_48;
static const uint32_t kernels_49[] = KERNELS_49;
static const uint32_t kernels_50[] = KERNELS_50;
static const uint32_t kernels_51[] = KERNELS_51;
static const uint32_t kernels_52[] = KERNELS_52;
static const uint32_t kernels_53[] = KERNELS_53;
static const uint32_t kernels_54[] = KERNELS_54;
static const uint32_t kernels_55[] = KERNELS_55;
static const uint32_t kernels_56[] = KERNELS_56;
static const uint32_t kernels_57[] = KERNELS_57;
static const uint32_t kernels_58[] = KERNELS_58;
static const uint32_t kernels_59[] = KERNELS_59;
static const uint32_t kernels_60[] = KERNELS_60;
static const uint32_t kernels_61[] = KERNELS_61;
static const uint32_t kernels_62[] = KERNELS_62;
static const uint32_t kernels_63[] = KERNELS_63;

void load_kernels(void)
{
  *((volatile uint8_t *) 0x50180001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50180000, kernels_0, 1249);
  *((volatile uint8_t *) 0x50184001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50184000, kernels_1, 1249);
  *((volatile uint8_t *) 0x50188001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50188000, kernels_2, 1249);
  *((volatile uint8_t *) 0x5018c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5018c000, kernels_3, 1249);
  *((volatile uint8_t *) 0x50190001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50190000, kernels_4, 1249);
  *((volatile uint8_t *) 0x50194001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50194000, kernels_5, 1249);
  *((volatile uint8_t *) 0x50198001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50198000, kernels_6, 1249);
  *((volatile uint8_t *) 0x5019c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5019c000, kernels_7, 576);
  *((volatile uint8_t *) 0x501a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a0000, kernels_8, 576);
  *((volatile uint8_t *) 0x501a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a4000, kernels_9, 576);
  *((volatile uint8_t *) 0x501a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a8000, kernels_10, 576);
  *((volatile uint8_t *) 0x501ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501ac000, kernels_11, 576);
  *((volatile uint8_t *) 0x501b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b0000, kernels_12, 576);
  *((volatile uint8_t *) 0x501b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b4000, kernels_13, 576);
  *((volatile uint8_t *) 0x501b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b8000, kernels_14, 576);
  *((volatile uint8_t *) 0x501bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501bc000, kernels_15, 576);
  *((volatile uint8_t *) 0x50580001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50580000, kernels_16, 576);
  *((volatile uint8_t *) 0x50584001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50584000, kernels_17, 576);
  *((volatile uint8_t *) 0x50588001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50588000, kernels_18, 576);
  *((volatile uint8_t *) 0x5058c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5058c000, kernels_19, 576);
  *((volatile uint8_t *) 0x50590001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50590000, kernels_20, 576);
  *((volatile uint8_t *) 0x50594001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50594000, kernels_21, 576);
  *((volatile uint8_t *) 0x50598001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50598000, kernels_22, 576);
  *((volatile uint8_t *) 0x5059c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5059c000, kernels_23, 576);
  *((volatile uint8_t *) 0x505a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a0000, kernels_24, 576);
  *((volatile uint8_t *) 0x505a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a4000, kernels_25, 576);
  *((volatile uint8_t *) 0x505a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a8000, kernels_26, 576);
  *((volatile uint8_t *) 0x505ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505ac000, kernels_27, 576);
  *((volatile uint8_t *) 0x505b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b0000, kernels_28, 576);
  *((volatile uint8_t *) 0x505b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b4000, kernels_29, 576);
  *((volatile uint8_t *) 0x505b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b8000, kernels_30, 576);
  *((volatile uint8_t *) 0x505bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505bc000, kernels_31, 576);
  *((volatile uint8_t *) 0x50980001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50980000, kernels_32, 576);
  *((volatile uint8_t *) 0x50984001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50984000, kernels_33, 576);
  *((volatile uint8_t *) 0x50988001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50988000, kernels_34, 592);
  *((volatile uint8_t *) 0x5098c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5098c000, kernels_35, 592);
  *((volatile uint8_t *) 0x50990001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50990000, kernels_36, 592);
  *((volatile uint8_t *) 0x50994001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50994000, kernels_37, 592);
  *((volatile uint8_t *) 0x50998001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50998000, kernels_38, 592);
  *((volatile uint8_t *) 0x5099c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5099c000, kernels_39, 592);
  *((volatile uint8_t *) 0x509a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a0000, kernels_40, 592);
  *((volatile uint8_t *) 0x509a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a4000, kernels_41, 592);
  *((volatile uint8_t *) 0x509a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a8000, kernels_42, 592);
  *((volatile uint8_t *) 0x509ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509ac000, kernels_43, 592);
  *((volatile uint8_t *) 0x509b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b0000, kernels_44, 592);
  *((volatile uint8_t *) 0x509b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b4000, kernels_45, 592);
  *((volatile uint8_t *) 0x509b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b8000, kernels_46, 592);
  *((volatile uint8_t *) 0x509bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509bc000, kernels_47, 592);
  *((volatile uint8_t *) 0x50d80001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d80000, kernels_48, 592);
  *((volatile uint8_t *) 0x50d84001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d84000, kernels_49, 592);
  *((volatile uint8_t *) 0x50d88001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d88000, kernels_50, 592);
  *((volatile uint8_t *) 0x50d8c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d8c000, kernels_51, 592);
  *((volatile uint8_t *) 0x50d90001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d90000, kernels_52, 592);
  *((volatile uint8_t *) 0x50d94001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d94000, kernels_53, 592);
  *((volatile uint8_t *) 0x50d98001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d98000, kernels_54, 592);
  *((volatile uint8_t *) 0x50d9c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d9c000, kernels_55, 592);
  *((volatile uint8_t *) 0x50da0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da0000, kernels_56, 592);
  *((volatile uint8_t *) 0x50da4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da4000, kernels_57, 592);
  *((volatile uint8_t *) 0x50da8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da8000, kernels_58, 592);
  *((volatile uint8_t *) 0x50dac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dac000, kernels_59, 592);
  *((volatile uint8_t *) 0x50db0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db0000, kernels_60, 592);
  *((volatile uint8_t *) 0x50db4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db4000, kernels_61, 592);
  *((volatile uint8_t *) 0x50db8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db8000, kernels_62, 592);
  *((volatile uint8_t *) 0x50dbc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dbc000, kernels_63, 592);
}

int cnn_load(void)
{
  *((volatile uint32_t *) 0x50001000) = 0x00000000; // AON control
  *((volatile uint32_t *) 0x50100000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50100004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50100008) = 0x00000009; // Layer count

  *((volatile uint32_t *) 0x50500000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50500004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50500008) = 0x00000009; // Layer count

  *((volatile uint32_t *) 0x50900000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50900004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50900008) = 0x00000009; // Layer count

  *((volatile uint32_t *) 0x50d00000) = 0x00100008; // Stop SM
  *((volatile uint32_t *) 0x50d00004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50d00008) = 0x00000009; // Layer count

  load_kernels();

  // Layer 0 group 0
  *((volatile uint32_t *) 0x50100010) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50100310) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100590) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a10) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50100610) = 0x00000678; // Mask offset and count
  *((volatile uint32_t *) 0x50100110) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50100710) = 0xffffffff; // Mask and processor enables

  // Layer 0 group 1
  *((volatile uint32_t *) 0x50500010) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50500310) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a10) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50500610) = 0x00000678; // Mask offset and count
  *((volatile uint32_t *) 0x50500110) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50500710) = 0xffffffff; // Mask and processor enables

  // Layer 0 group 2
  *((volatile uint32_t *) 0x50900010) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50900310) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a10) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50900610) = 0x00000678; // Mask offset and count
  *((volatile uint32_t *) 0x50900110) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50900710) = 0xffffffff; // Mask and processor enables

  // Layer 0 group 3
  *((volatile uint32_t *) 0x50d00010) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50d00310) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00590) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a10) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50d00610) = 0x00000678; // Mask offset and count
  *((volatile uint32_t *) 0x50d00110) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50d00710) = 0xffffffff; // Mask and processor enables

  // Layer 1 group 0
  *((volatile uint32_t *) 0x50100014) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50100414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100514) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50100594) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a14) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50100614) = 0x06c00d38; // Mask offset and count
  *((volatile uint32_t *) 0x50100114) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50100714) = 0xffffffff; // Mask and processor enables

  // Layer 1 group 1
  *((volatile uint32_t *) 0x50500014) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50500414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500514) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50500594) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a14) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50500614) = 0x06c00d38; // Mask offset and count
  *((volatile uint32_t *) 0x50500114) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50500714) = 0xffffffff; // Mask and processor enables

  // Layer 1 group 2
  *((volatile uint32_t *) 0x50900014) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50900414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900514) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50900594) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a14) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50900614) = 0x06c00d38; // Mask offset and count
  *((volatile uint32_t *) 0x50900114) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50900714) = 0xffffffff; // Mask and processor enables

  // Layer 1 group 3
  *((volatile uint32_t *) 0x50d00014) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50d00414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00514) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d00594) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a14) = 0x00019811; // Layer control 2
  *((volatile uint32_t *) 0x50d00614) = 0x06c00d38; // Mask offset and count
  *((volatile uint32_t *) 0x50d00114) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50d00714) = 0x000f000f; // Mask and processor enables

  // Layer 2 group 0
  *((volatile uint32_t *) 0x50100018) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50100318) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100598) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a18) = 0x00018801; // Layer control 2
  *((volatile uint32_t *) 0x50100618) = 0x0d801098; // Mask offset and count
  *((volatile uint32_t *) 0x50100118) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50100718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 1
  *((volatile uint32_t *) 0x50500018) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50500318) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500598) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a18) = 0x00018801; // Layer control 2
  *((volatile uint32_t *) 0x50500618) = 0x0d801098; // Mask offset and count
  *((volatile uint32_t *) 0x50500118) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50500718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 2
  *((volatile uint32_t *) 0x50900018) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50900318) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900598) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a18) = 0x00018801; // Layer control 2
  *((volatile uint32_t *) 0x50900618) = 0x0d801098; // Mask offset and count
  *((volatile uint32_t *) 0x50900118) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50900718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 3
  *((volatile uint32_t *) 0x50d00018) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50d00318) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00598) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a18) = 0x00018801; // Layer control 2
  *((volatile uint32_t *) 0x50d00618) = 0x0d801098; // Mask offset and count
  *((volatile uint32_t *) 0x50d00118) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50d00718) = 0x000f000f; // Mask and processor enables

  // Layer 3 group 0
  *((volatile uint32_t *) 0x5010001c) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x5010041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5010049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5010051c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x5010059c) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a1c) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x5010061c) = 0x10e01158; // Mask offset and count
  *((volatile uint32_t *) 0x5010011c) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x5010071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 1
  *((volatile uint32_t *) 0x5050001c) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x5050041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5050049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5050051c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x5050059c) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a1c) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x5050061c) = 0x10e01158; // Mask offset and count
  *((volatile uint32_t *) 0x5050011c) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x5050071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 2
  *((volatile uint32_t *) 0x5090001c) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x5090041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5090049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5090051c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x5090059c) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a1c) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x5090061c) = 0x10e01158; // Mask offset and count
  *((volatile uint32_t *) 0x5090011c) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x5090071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 3
  *((volatile uint32_t *) 0x50d0001c) = 0x0000007f; // Rows
  *((volatile uint32_t *) 0x50d0041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d0049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d0051c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d0059c) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a1c) = 0x00007800; // Layer control 2
  *((volatile uint32_t *) 0x50d0061c) = 0x10e01158; // Mask offset and count
  *((volatile uint32_t *) 0x50d0011c) = 0x00001100; // 1D
  *((volatile uint32_t *) 0x50d0071c) = 0x00030003; // Mask and processor enables

  // Layer 4 group 0
  *((volatile uint32_t *) 0x50100020) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x501000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50100320) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005a0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50100a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50100620) = 0x020002f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a0) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50100720) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 1
  *((volatile uint32_t *) 0x50500020) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x505000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50500320) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005a0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50500620) = 0x020002f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a0) = 0x00000007; // TRAM ptr max

  // Layer 4 group 2
  *((volatile uint32_t *) 0x50900020) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x509000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50900320) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005a0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50900620) = 0x020002f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a0) = 0x00000007; // TRAM ptr max

  // Layer 4 group 3
  *((volatile uint32_t *) 0x50d00020) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d00320) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005a0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00620) = 0x020002f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a0) = 0x00000007; // TRAM ptr max

  // Layer 5 group 0
  *((volatile uint32_t *) 0x50100024) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x501000a4) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50100424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100524) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005a4) = 0x00006b20; // Layer control
  *((volatile uint32_t *) 0x50100a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50100624) = 0x030004f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a4) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50100724) = 0xfff0fff0; // Mask and processor enables

  // Layer 5 group 1
  *((volatile uint32_t *) 0x50500024) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x505000a4) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50500424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500524) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50500624) = 0x030004f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a4) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50500724) = 0xffffffff; // Mask and processor enables

  // Layer 5 group 2
  *((volatile uint32_t *) 0x50900024) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x509000a4) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50900424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900524) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50900624) = 0x030004f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a4) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50900724) = 0x000f000f; // Mask and processor enables

  // Layer 5 group 3
  *((volatile uint32_t *) 0x50d00024) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d000a4) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d00424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00524) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a24) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00624) = 0x030004f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a4) = 0x00000007; // TRAM ptr max

  // Layer 6 group 0
  *((volatile uint32_t *) 0x50100028) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x501000a8) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50100328) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005a8) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50100628) = 0x050006f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006a8) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50100728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 1
  *((volatile uint32_t *) 0x50500028) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x505000a8) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50500328) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50500628) = 0x050006f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006a8) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50500728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 2
  *((volatile uint32_t *) 0x50900028) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x509000a8) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50900328) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50900628) = 0x050006f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006a8) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50900728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 3
  *((volatile uint32_t *) 0x50d00028) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d000a8) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d00328) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a8) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005a8) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a28) = 0x0001f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00628) = 0x050006f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a8) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00728) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 0
  *((volatile uint32_t *) 0x5010002c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x501000ac) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x5010032c) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x5010042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5010052c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005ac) = 0x0000eb20; // Layer control
  *((volatile uint32_t *) 0x50100a2c) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x5010062c) = 0x070007f8; // Mask offset and count
  *((volatile uint32_t *) 0x501006ac) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5010072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 1
  *((volatile uint32_t *) 0x5050002c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x505000ac) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x5050032c) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x5050042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5050052c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005ac) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a2c) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x5050062c) = 0x070007f8; // Mask offset and count
  *((volatile uint32_t *) 0x505006ac) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5050072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 2
  *((volatile uint32_t *) 0x5090002c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x509000ac) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x5090032c) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x5090042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5090052c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005ac) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a2c) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x5090062c) = 0x070007f8; // Mask offset and count
  *((volatile uint32_t *) 0x509006ac) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5090072c) = 0xffffffff; // Mask and processor enables

  // Layer 7 group 3
  *((volatile uint32_t *) 0x50d0002c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d000ac) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d0032c) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d0042c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004ac) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d0052c) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005ac) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a2c) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50d0062c) = 0x070007f8; // Mask offset and count
  *((volatile uint32_t *) 0x50d006ac) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50d0072c) = 0xffffffff; // Mask and processor enables

  // Layer 8 group 0
  *((volatile uint32_t *) 0x50100030) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x501000b0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50100430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005b0) = 0x0000cb20; // Layer control
  *((volatile uint32_t *) 0x50100a30) = 0x00003000; // Layer control 2
  *((volatile uint32_t *) 0x50100630) = 0x08000830; // Mask offset and count
  *((volatile uint32_t *) 0x501006b0) = 0x00000007; // TRAM ptr max

  // Layer 8 group 1
  *((volatile uint32_t *) 0x50500030) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x505000b0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50500430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005b0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a30) = 0x00003000; // Layer control 2
  *((volatile uint32_t *) 0x50500630) = 0x08000830; // Mask offset and count
  *((volatile uint32_t *) 0x505006b0) = 0x00000007; // TRAM ptr max

  // Layer 8 group 2
  *((volatile uint32_t *) 0x50900030) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x509000b0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50900430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005b0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a30) = 0x00003000; // Layer control 2
  *((volatile uint32_t *) 0x50900630) = 0x08000830; // Mask offset and count
  *((volatile uint32_t *) 0x509006b0) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50900730) = 0xfffcfffc; // Mask and processor enables

  // Layer 8 group 3
  *((volatile uint32_t *) 0x50d00030) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d000b0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d00430) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004b0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005b0) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a30) = 0x00003000; // Layer control 2
  *((volatile uint32_t *) 0x50d00630) = 0x08000830; // Mask offset and count
  *((volatile uint32_t *) 0x50d006b0) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00730) = 0xffffffff; // Mask and processor enables

  // Layer 9 group 0
  *((volatile uint32_t *) 0x50100334) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x501003b4) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50100434) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004b4) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005b4) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50100a34) = 0x0000a00f; // Layer control 2
  *((volatile uint32_t *) 0x50100634) = 0x48009bf8; // Mask offset and count
  *((volatile uint32_t *) 0x50100134) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x501006b4) = 0x007f00ff; // TRAM ptr max
  *((volatile uint32_t *) 0x501007b4) = 0x081c0000; // Post processing register
  *((volatile uint32_t *) 0x50100734) = 0x007f007f; // Mask and processor enables

  // Layer 9 group 1
  *((volatile uint32_t *) 0x50500334) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x505003b4) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50500434) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004b4) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005b4) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50500a34) = 0x0000a00f; // Layer control 2
  *((volatile uint32_t *) 0x50500634) = 0x48009bf8; // Mask offset and count
  *((volatile uint32_t *) 0x50500134) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x505006b4) = 0x007f00ff; // TRAM ptr max
  *((volatile uint32_t *) 0x505007b4) = 0x081c0000; // Post processing register

  // Layer 9 group 2
  *((volatile uint32_t *) 0x50900334) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x509003b4) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50900434) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004b4) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005b4) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50900a34) = 0x0000a00f; // Layer control 2
  *((volatile uint32_t *) 0x50900634) = 0x48009bf8; // Mask offset and count
  *((volatile uint32_t *) 0x50900134) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x509006b4) = 0x007f00ff; // TRAM ptr max
  *((volatile uint32_t *) 0x509007b4) = 0x081c0000; // Post processing register

  // Layer 9 group 3
  *((volatile uint32_t *) 0x50d00334) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d003b4) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50d00434) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004b4) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005b4) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50d00a34) = 0x0000a00f; // Layer control 2
  *((volatile uint32_t *) 0x50d00634) = 0x48009bf8; // Mask offset and count
  *((volatile uint32_t *) 0x50d00134) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x50d006b4) = 0x007f00ff; // TRAM ptr max
  *((volatile uint32_t *) 0x50d007b4) = 0x081c0000; // Post processing register

  load_input(); // Load data input

  *((volatile uint32_t *) 0x50100000) = 0x00100808; // Enable group 0
  *((volatile uint32_t *) 0x50500000) = 0x00100809; // Enable group 1
  *((volatile uint32_t *) 0x50900000) = 0x00100809; // Enable group 2
  *((volatile uint32_t *) 0x50d00000) = 0x00100809; // Enable group 3

  CNN_START; // Allow capture of processing time
  *((volatile uint32_t *) 0x50100000) = 0x00100009; // Master enable group 0

  return 1;
}

// kws20
// Expected output of layer 9
int cnn_check(void)
{
  int rv = 1;
  if ((*((volatile uint32_t *) 0x50402000)) != 0xfffdcb58) return 0; // 0,0,0
  if ((*((volatile uint32_t *) 0x50402004)) != 0xfffdcf28) return 0; // 0,0,1
  if ((*((volatile uint32_t *) 0x50402008)) != 0xfffbf8a9) return 0; // 0,0,2
  if ((*((volatile uint32_t *) 0x5040200c)) != 0xfffaec9a) return 0; // 0,0,3
  if ((*((volatile uint32_t *) 0x5040a000)) != 0xfffdb6a3) return 0; // 0,0,4
  if ((*((volatile uint32_t *) 0x5040a004)) != 0xffffaba7) return 0; // 0,0,5
  if ((*((volatile uint32_t *) 0x5040a008)) != 0xfffb7344) return 0; // 0,0,6
  if ((*((volatile uint32_t *) 0x5040a00c)) != 0xfffd0bdd) return 0; // 0,0,7
  if ((*((volatile uint32_t *) 0x50412000)) != 0xfffdc014) return 0; // 0,0,8
  if ((*((volatile uint32_t *) 0x50412004)) != 0xfffd823a) return 0; // 0,0,9
  if ((*((volatile uint32_t *) 0x50412008)) != 0xfffdbd1e) return 0; // 0,0,10
  if ((*((volatile uint32_t *) 0x5041200c)) != 0xfffdd4d9) return 0; // 0,0,11
  if ((*((volatile uint32_t *) 0x5041a000)) != 0xfffb3af7) return 0; // 0,0,12
  if ((*((volatile uint32_t *) 0x5041a004)) != 0x0001b7af) return 0; // 0,0,13
  if ((*((volatile uint32_t *) 0x5041a008)) != 0xfffbdad1) return 0; // 0,0,14
  if ((*((volatile uint32_t *) 0x5041a00c)) != 0xfffaca1a) return 0; // 0,0,15
  if ((*((volatile uint32_t *) 0x50802000)) != 0xfffd56ca) return 0; // 0,0,16
  if ((*((volatile uint32_t *) 0x50802004)) != 0xfffc2d7c) return 0; // 0,0,17
  if ((*((volatile uint32_t *) 0x50802008)) != 0xfff91fa1) return 0; // 0,0,18
  if ((*((volatile uint32_t *) 0x5080200c)) != 0xfffebfc4) return 0; // 0,0,19
  if ((*((volatile uint32_t *) 0x5080a000)) != 0xfffe6b07) return 0; // 0,0,20
  return rv;
}

// Custom unload for this network:
// 32-bit data, shape: [21, 1, 1]
void cnn_unload(uint32_t *out_buf)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *) 0x50402000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  addr = (volatile uint32_t *) 0x5040a000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  addr = (volatile uint32_t *) 0x50412000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  addr = (volatile uint32_t *) 0x5041a000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  addr = (volatile uint32_t *) 0x50802000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
  addr = (volatile uint32_t *) 0x5080a000;
  *out_buf++ = *addr++;
}

// Classification layer:
#define NUM_OUTPUTS 21
static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];

int softmax_layer(void)
{
  cnn_unload((uint32_t *) ml_data);
  softmax_q17p14_q15((const q31_t *) ml_data, NUM_OUTPUTS, ml_softmax);

  return 1;
}

int main(void)
{
  int i;
  int digs, tens;

  MXC_ICC_Enable(MXC_ICC0); // Enable cache

  // Switch to 100 MHz clock
  MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
  SystemCoreClockUpdate();

  // Reset all domains, restore power to CNN
  MXC_BBFC->reg3 = 0xf; // Reset
  MXC_BBFC->reg1 = 0xf; // Mask memory
  MXC_BBFC->reg0 = 0xf; // Power
  MXC_BBFC->reg2 = 0x0; // Iso
  MXC_BBFC->reg3 = 0x0; // Reset

  MXC_GCR->pclkdiv &= ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV | MXC_F_GCR_PCLKDIV_CNNCLKSEL);
  MXC_GCR->pclkdiv |= MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1; // CNN clock: 100 MHz div 2
  MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

  printf("\n*** CNN Test ***\n");

  if (!cnn_load()) {
	  fail();
  }
  MXC_TMR_SW_Start(MXC_TMR0);
  cnn_wait();

  if (!cnn_check()) {
	  fail();
  }
  if (!softmax_layer()) {
	  fail();
  }

  printf("\n*** PASS ***\n\n");

  printf("Time for CNN: %d us\n\n", cnn_time);

  // Disable power to CNN
  MXC_BBFC->reg3 = 0xf; // Reset
  MXC_BBFC->reg1 = 0x0; // Mask memory
  MXC_BBFC->reg0 = 0x0; // Power
  MXC_BBFC->reg2 = 0xf; // Iso
  MXC_BBFC->reg3 = 0x0; // Reset

  printf("Classification results:\n");
  for (i = 0; i < NUM_OUTPUTS; i++) {
    digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
    tens = digs % 10;
    digs = digs / 10;
    printf("[%7d] -> Class %d: %d.%d%%\n", ml_data[i], i, digs, tens);
  }

  return 0;
}

