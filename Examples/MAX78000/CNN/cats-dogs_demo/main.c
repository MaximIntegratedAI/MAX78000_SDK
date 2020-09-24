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

// cats_vs_dogs_chw-fifo
// Created using ./ai8xize.py -e --verbose --top-level cnn -L --test-dir demos --prefix cats_vs_dogs_chw-fifo --checkpoint-file trained/ai85-catsdogs-chw.pth.tar --config-file networks/cats-dogs-chw.yaml --device 85 --compact-data --mexpress --softmax --display-checkpoint --fifo --boost 2.5

// Configuring 7 layers:
// Layer 0: 3x64x64 (CHW/big data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 15x64x64 output
// Layer 1: 15x64x64 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 30x32x32 output
// Layer 2: 30x32x32 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 60x16x16 output
// Layer 3: 60x16x16 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 30x8x8 output
// Layer 4: 30x8x8 (HWC/little data), 2x2 max pool with stride 2/2, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 30x4x4 output
// Layer 5: 30x4x4 (HWC/little data), no pooling, conv2d with kernel size 3x3, stride 1/1, pad 1/1, 30x4x4 output
// Layer 6: 30x4x4 (flattened HWC/little data), no pooling, conv2d with kernel size 1x1, stride 1/1, pad 0/0, 2x1x1 output

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc_device.h"
#include "mxc_sys.h"
#include "bbfc_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "led.h"
#include "tmr.h"
#include "tft.h"
#include "pb.h"
#include "tornadocnn.h"
#include "weights.h"
#include "sampledata.h"
#include "mxc_delay.h"
#include "camera.h"
#include "bitmap.h"

// Comment out USE_SAMPLEDATA to use Camera module
//#define USE_SAMPLEDATA

#define CAMERA_TO_LCD   (1)
#define IMAGE_SIZE_X  (64)
#define IMAGE_SIZE_Y  (64)

#define TFT_BUFF_SIZE   50    // TFT buffer size
#define NUM_OUTPUTS     2     // number of classes
const char classes[NUM_OUTPUTS][10] = {"Cat", "Dog"};

uint32_t cnn_time; // Stopwatch
uint32_t input_0_camera[1024];
uint32_t input_1_camera[1024];
uint32_t input_2_camera[1024];

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

#ifdef USE_SAMPLEDATA
  // Data input: CHW (big data): 3x64x64
  static const uint32_t input_0[] = INPUT_0;
  static const uint32_t input_1[] = INPUT_1;
  static const uint32_t input_2[] = INPUT_2;
#endif

/* **************************************************************************** */
void load_input(void)
{
  int i;
#ifdef USE_SAMPLEDATA
  const uint32_t *in0 = input_0;
  const uint32_t *in1 = input_1;
  const uint32_t *in2 = input_2;
#else
  const uint32_t *in0 = input_0_camera;
  const uint32_t *in1 = input_1_camera;
  const uint32_t *in2 = input_2_camera;
#endif

  for (i = 0; i < 1024; i++) {
    while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
    *((volatile uint32_t *) 0x50000008) = *in0++; // Write FIFO 0
    while (((*((volatile uint32_t *) 0x50000004) & 2)) != 0); // Wait for FIFO 1
    *((volatile uint32_t *) 0x5000000c) = *in1++; // Write FIFO 1
    while (((*((volatile uint32_t *) 0x50000004) & 4)) != 0); // Wait for FIFO 2
    *((volatile uint32_t *) 0x50000010) = *in2++; // Write FIFO 2
  }
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

/* **************************************************************************** */
void load_kernels(void)
{
  *((volatile uint8_t *) 0x50180001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50180000, kernels_0, 315);
  *((volatile uint8_t *) 0x50184001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50184000, kernels_1, 315);
  *((volatile uint8_t *) 0x50188001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50188000, kernels_2, 315);
  *((volatile uint8_t *) 0x5018c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5018c000, kernels_3, 315);
  *((volatile uint8_t *) 0x50190001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50190000, kernels_4, 315);
  *((volatile uint8_t *) 0x50194001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50194000, kernels_5, 315);
  *((volatile uint8_t *) 0x50198001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50198000, kernels_6, 315);
  *((volatile uint8_t *) 0x5019c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5019c000, kernels_7, 315);
  *((volatile uint8_t *) 0x501a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a0000, kernels_8, 315);
  *((volatile uint8_t *) 0x501a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a4000, kernels_9, 315);
  *((volatile uint8_t *) 0x501a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501a8000, kernels_10, 315);
  *((volatile uint8_t *) 0x501ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501ac000, kernels_11, 315);
  *((volatile uint8_t *) 0x501b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b0000, kernels_12, 315);
  *((volatile uint8_t *) 0x501b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b4000, kernels_13, 315);
  *((volatile uint8_t *) 0x501b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501b8000, kernels_14, 315);
  *((volatile uint8_t *) 0x501bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x501bc000, kernels_15, 315);
  *((volatile uint8_t *) 0x50580001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50580000, kernels_16, 315);
  *((volatile uint8_t *) 0x50584001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50584000, kernels_17, 315);
  *((volatile uint8_t *) 0x50588001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50588000, kernels_18, 315);
  *((volatile uint8_t *) 0x5058c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5058c000, kernels_19, 315);
  *((volatile uint8_t *) 0x50590001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50590000, kernels_20, 315);
  *((volatile uint8_t *) 0x50594001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50594000, kernels_21, 315);
  *((volatile uint8_t *) 0x50598001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50598000, kernels_22, 315);
  *((volatile uint8_t *) 0x5059c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5059c000, kernels_23, 315);
  *((volatile uint8_t *) 0x505a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a0000, kernels_24, 315);
  *((volatile uint8_t *) 0x505a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a4000, kernels_25, 315);
  *((volatile uint8_t *) 0x505a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505a8000, kernels_26, 315);
  *((volatile uint8_t *) 0x505ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505ac000, kernels_27, 315);
  *((volatile uint8_t *) 0x505b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b0000, kernels_28, 315);
  *((volatile uint8_t *) 0x505b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b4000, kernels_29, 315);
  *((volatile uint8_t *) 0x505b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505b8000, kernels_30, 306);
  *((volatile uint8_t *) 0x505bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x505bc000, kernels_31, 306);
  *((volatile uint8_t *) 0x50980001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50980000, kernels_32, 306);
  *((volatile uint8_t *) 0x50984001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50984000, kernels_33, 306);
  *((volatile uint8_t *) 0x50988001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50988000, kernels_34, 311);
  *((volatile uint8_t *) 0x5098c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5098c000, kernels_35, 311);
  *((volatile uint8_t *) 0x50990001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50990000, kernels_36, 311);
  *((volatile uint8_t *) 0x50994001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50994000, kernels_37, 311);
  *((volatile uint8_t *) 0x50998001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50998000, kernels_38, 311);
  *((volatile uint8_t *) 0x5099c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x5099c000, kernels_39, 311);
  *((volatile uint8_t *) 0x509a0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a0000, kernels_40, 311);
  *((volatile uint8_t *) 0x509a4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a4000, kernels_41, 311);
  *((volatile uint8_t *) 0x509a8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509a8000, kernels_42, 311);
  *((volatile uint8_t *) 0x509ac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509ac000, kernels_43, 311);
  *((volatile uint8_t *) 0x509b0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b0000, kernels_44, 311);
  *((volatile uint8_t *) 0x509b4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b4000, kernels_45, 311);
  *((volatile uint8_t *) 0x509b8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509b8000, kernels_46, 311);
  *((volatile uint8_t *) 0x509bc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x509bc000, kernels_47, 311);
  *((volatile uint8_t *) 0x50d80001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d80000, kernels_48, 311);
  *((volatile uint8_t *) 0x50d84001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d84000, kernels_49, 311);
  *((volatile uint8_t *) 0x50d88001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d88000, kernels_50, 311);
  *((volatile uint8_t *) 0x50d8c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d8c000, kernels_51, 311);
  *((volatile uint8_t *) 0x50d90001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d90000, kernels_52, 311);
  *((volatile uint8_t *) 0x50d94001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d94000, kernels_53, 311);
  *((volatile uint8_t *) 0x50d98001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d98000, kernels_54, 311);
  *((volatile uint8_t *) 0x50d9c001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50d9c000, kernels_55, 311);
  *((volatile uint8_t *) 0x50da0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da0000, kernels_56, 311);
  *((volatile uint8_t *) 0x50da4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da4000, kernels_57, 311);
  *((volatile uint8_t *) 0x50da8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50da8000, kernels_58, 311);
  *((volatile uint8_t *) 0x50dac001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dac000, kernels_59, 311);
  *((volatile uint8_t *) 0x50db0001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db0000, kernels_60, 311);
  *((volatile uint8_t *) 0x50db4001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db4000, kernels_61, 311);
  *((volatile uint8_t *) 0x50db8001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50db8000, kernels_62, 311);
  *((volatile uint8_t *) 0x50dbc001) = 0x01; // Set address
  memcpy32((uint32_t *) 0x50dbc000, kernels_63, 311);
}

static const uint8_t bias_0[] = BIAS_0;

/* **************************************************************************** */
void memcpy_8to32(uint32_t *dst, const uint8_t *src, size_t n)
{
  while (n-- > 0) {
    *dst++ = *src++;
  }
}

/* **************************************************************************** */
void load_bias(void)
{
  memcpy_8to32((uint32_t *) 0x50108000, bias_0, sizeof(uint8_t) * 2);
}

int cnn_load(void)
{
  *((volatile uint32_t *) 0x50001000) = 0x00000000; // AON control
  *((volatile uint32_t *) 0x50100000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50100004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50100008) = 0x00000006; // Layer count

  *((volatile uint32_t *) 0x50500000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50500004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50500008) = 0x00000006; // Layer count

  *((volatile uint32_t *) 0x50900000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50900004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50900008) = 0x00000006; // Layer count

  *((volatile uint32_t *) 0x50d00000) = 0x00108008; // Stop SM
  *((volatile uint32_t *) 0x50d00004) = 0x0000040e; // SRAM control
  *((volatile uint32_t *) 0x50d00008) = 0x00000006; // Layer count

  load_kernels();
  load_bias();

  // Layer 0 group 0
  *((volatile uint32_t *) 0x50100010) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50100090) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50100310) = 0x00012000; // SRAM write ptr
  *((volatile uint32_t *) 0x50100410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100590) = 0x00006b60; // Layer control
  *((volatile uint32_t *) 0x50100a10) = 0x00007000; // Layer control 2
  *((volatile uint32_t *) 0x50100610) = 0x00000070; // Mask offset and count
  *((volatile uint32_t *) 0x50100690) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x50100710) = 0x00010001; // Mask and processor enables
  *((volatile uint32_t *) 0x50100810) = 0x00000400; // Stream processing start
  *((volatile uint32_t *) 0x50100990) = 0x00000400; // Input frame size

  // Layer 0 group 1
  *((volatile uint32_t *) 0x50500010) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50500090) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50500310) = 0x00012000; // SRAM write ptr
  *((volatile uint32_t *) 0x50500410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500590) = 0x00000b60; // Layer control
  *((volatile uint32_t *) 0x50500a10) = 0x00007000; // Layer control 2
  *((volatile uint32_t *) 0x50500610) = 0x00000070; // Mask offset and count
  *((volatile uint32_t *) 0x50500690) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x50500710) = 0x00010001; // Mask and processor enables
  *((volatile uint32_t *) 0x50500810) = 0x00000400; // Stream processing start
  *((volatile uint32_t *) 0x50500990) = 0x00000400; // Input frame size

  // Layer 0 group 2
  *((volatile uint32_t *) 0x50900010) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50900090) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50900310) = 0x00012000; // SRAM write ptr
  *((volatile uint32_t *) 0x50900410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900590) = 0x00000b60; // Layer control
  *((volatile uint32_t *) 0x50900a10) = 0x00007000; // Layer control 2
  *((volatile uint32_t *) 0x50900610) = 0x00000070; // Mask offset and count
  *((volatile uint32_t *) 0x50900690) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x50900710) = 0x00010001; // Mask and processor enables
  *((volatile uint32_t *) 0x50900810) = 0x00000400; // Stream processing start
  *((volatile uint32_t *) 0x50900990) = 0x00000400; // Input frame size

  // Layer 0 group 3
  *((volatile uint32_t *) 0x50d00010) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50d00090) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50d00310) = 0x00012000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00410) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00490) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00590) = 0x00000b60; // Layer control
  *((volatile uint32_t *) 0x50d00a10) = 0x00007000; // Layer control 2
  *((volatile uint32_t *) 0x50d00610) = 0x00000070; // Mask offset and count
  *((volatile uint32_t *) 0x50d00690) = 0x0000003f; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00810) = 0x00000400; // Stream processing start
  *((volatile uint32_t *) 0x50d00990) = 0x00000400; // Input frame size

  // Layer 1 group 0
  *((volatile uint32_t *) 0x50100014) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50100094) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50100194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50100294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100314) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100594) = 0x0000cba0; // Layer control
  *((volatile uint32_t *) 0x50100a14) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50100614) = 0x000000e8; // Mask offset and count
  *((volatile uint32_t *) 0x50100694) = 0x0000001f; // TRAM ptr max

  // Layer 1 group 1
  *((volatile uint32_t *) 0x50500014) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50500094) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50500194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50500294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500314) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a14) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50500614) = 0x000000e8; // Mask offset and count
  *((volatile uint32_t *) 0x50500694) = 0x0000001f; // TRAM ptr max

  // Layer 1 group 2
  *((volatile uint32_t *) 0x50900014) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50900094) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50900194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50900294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900314) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a14) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50900614) = 0x000000e8; // Mask offset and count
  *((volatile uint32_t *) 0x50900694) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x50900714) = 0xfff0fff0; // Mask and processor enables

  // Layer 1 group 3
  *((volatile uint32_t *) 0x50d00014) = 0x00010041; // Rows
  *((volatile uint32_t *) 0x50d00094) = 0x00010041; // Columns
  *((volatile uint32_t *) 0x50d00194) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00214) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d00294) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00314) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00414) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00494) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00594) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a14) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50d00614) = 0x000000e8; // Mask offset and count
  *((volatile uint32_t *) 0x50d00694) = 0x0000001f; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00714) = 0x00070007; // Mask and processor enables

  // Layer 2 group 0
  *((volatile uint32_t *) 0x50100018) = 0x00010021; // Rows
  *((volatile uint32_t *) 0x50100098) = 0x00010021; // Columns
  *((volatile uint32_t *) 0x50100198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50100298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100318) = 0x00002000; // SRAM write ptr
  *((volatile uint32_t *) 0x50100418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50100498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50100598) = 0x00006ba0; // Layer control
  *((volatile uint32_t *) 0x50100a18) = 0x0001d800; // Layer control 2
  *((volatile uint32_t *) 0x50100618) = 0x00800258; // Mask offset and count
  *((volatile uint32_t *) 0x50100698) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x50100718) = 0xfff0fff0; // Mask and processor enables

  // Layer 2 group 1
  *((volatile uint32_t *) 0x50500018) = 0x00010021; // Rows
  *((volatile uint32_t *) 0x50500098) = 0x00010021; // Columns
  *((volatile uint32_t *) 0x50500198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50500298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500318) = 0x00002000; // SRAM write ptr
  *((volatile uint32_t *) 0x50500418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50500498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50500598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a18) = 0x0001d800; // Layer control 2
  *((volatile uint32_t *) 0x50500618) = 0x00800258; // Mask offset and count
  *((volatile uint32_t *) 0x50500698) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x50500718) = 0xffffffff; // Mask and processor enables

  // Layer 2 group 2
  *((volatile uint32_t *) 0x50900018) = 0x00010021; // Rows
  *((volatile uint32_t *) 0x50900098) = 0x00010021; // Columns
  *((volatile uint32_t *) 0x50900198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50900298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900318) = 0x00002000; // SRAM write ptr
  *((volatile uint32_t *) 0x50900418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50900498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50900598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a18) = 0x0001d800; // Layer control 2
  *((volatile uint32_t *) 0x50900618) = 0x00800258; // Mask offset and count
  *((volatile uint32_t *) 0x50900698) = 0x0000000f; // TRAM ptr max
  *((volatile uint32_t *) 0x50900718) = 0x00030003; // Mask and processor enables

  // Layer 2 group 3
  *((volatile uint32_t *) 0x50d00018) = 0x00010021; // Rows
  *((volatile uint32_t *) 0x50d00098) = 0x00010021; // Columns
  *((volatile uint32_t *) 0x50d00198) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00218) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d00298) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00318) = 0x00002000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00418) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d00498) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00518) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d00598) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a18) = 0x0001d800; // Layer control 2
  *((volatile uint32_t *) 0x50d00618) = 0x00800258; // Mask offset and count
  *((volatile uint32_t *) 0x50d00698) = 0x0000000f; // TRAM ptr max

  // Layer 3 group 0
  *((volatile uint32_t *) 0x5010001c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x5010009c) = 0x00010011; // Columns
  *((volatile uint32_t *) 0x5010019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5010021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5010029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5010031c) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x5010041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5010049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5010059c) = 0x0000eba0; // Layer control
  *((volatile uint32_t *) 0x50100a1c) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x5010061c) = 0x02600348; // Mask offset and count
  *((volatile uint32_t *) 0x5010069c) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5010071c) = 0xfff0fff0; // Mask and processor enables

  // Layer 3 group 1
  *((volatile uint32_t *) 0x5050001c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x5050009c) = 0x00010011; // Columns
  *((volatile uint32_t *) 0x5050019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5050021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5050029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5050031c) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x5050041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5050049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5050059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a1c) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x5050061c) = 0x02600348; // Mask offset and count
  *((volatile uint32_t *) 0x5050069c) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5050071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 2
  *((volatile uint32_t *) 0x5090001c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x5090009c) = 0x00010011; // Columns
  *((volatile uint32_t *) 0x5090019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x5090021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x5090029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x5090031c) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x5090041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x5090049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x5090059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a1c) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x5090061c) = 0x02600348; // Mask offset and count
  *((volatile uint32_t *) 0x5090069c) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x5090071c) = 0xffffffff; // Mask and processor enables

  // Layer 3 group 3
  *((volatile uint32_t *) 0x50d0001c) = 0x00010011; // Rows
  *((volatile uint32_t *) 0x50d0009c) = 0x00010011; // Columns
  *((volatile uint32_t *) 0x50d0019c) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d0021c) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d0029c) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d0031c) = 0x00002800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d0041c) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d0049c) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d0059c) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a1c) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50d0061c) = 0x02600348; // Mask offset and count
  *((volatile uint32_t *) 0x50d0069c) = 0x00000007; // TRAM ptr max
  *((volatile uint32_t *) 0x50d0071c) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 0
  *((volatile uint32_t *) 0x50100020) = 0x00010009; // Rows
  *((volatile uint32_t *) 0x501000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x501001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50100220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x501002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50100320) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50100420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005a0) = 0x00006ba0; // Layer control
  *((volatile uint32_t *) 0x50100a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50100620) = 0x03400438; // Mask offset and count
  *((volatile uint32_t *) 0x501006a0) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x50100720) = 0xfff0fff0; // Mask and processor enables

  // Layer 4 group 1
  *((volatile uint32_t *) 0x50500020) = 0x00010009; // Rows
  *((volatile uint32_t *) 0x505000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x505001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50500220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x505002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50500320) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50500420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50500a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50500620) = 0x03400438; // Mask offset and count
  *((volatile uint32_t *) 0x505006a0) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x50500720) = 0xffffffff; // Mask and processor enables

  // Layer 4 group 2
  *((volatile uint32_t *) 0x50900020) = 0x00010009; // Rows
  *((volatile uint32_t *) 0x509000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x509001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50900220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x509002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50900320) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50900420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50900a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50900620) = 0x03400438; // Mask offset and count
  *((volatile uint32_t *) 0x509006a0) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x50900720) = 0x00030003; // Mask and processor enables

  // Layer 4 group 3
  *((volatile uint32_t *) 0x50d00020) = 0x00010009; // Rows
  *((volatile uint32_t *) 0x50d000a0) = 0x00010009; // Columns
  *((volatile uint32_t *) 0x50d001a0) = 0x00000001; // Pooling rows
  *((volatile uint32_t *) 0x50d00220) = 0x00000001; // Pooling columns
  *((volatile uint32_t *) 0x50d002a0) = 0x00000001; // Stride
  *((volatile uint32_t *) 0x50d00320) = 0x00010000; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00420) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a0) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00520) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005a0) = 0x00000ba0; // Layer control
  *((volatile uint32_t *) 0x50d00a20) = 0x0000f800; // Layer control 2
  *((volatile uint32_t *) 0x50d00620) = 0x03400438; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a0) = 0x00000003; // TRAM ptr max

  // Layer 5 group 0
  *((volatile uint32_t *) 0x50100024) = 0x00010005; // Rows
  *((volatile uint32_t *) 0x501000a4) = 0x00010005; // Columns
  *((volatile uint32_t *) 0x50100324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50100424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x501005a4) = 0x0000cb20; // Layer control
  *((volatile uint32_t *) 0x50100a24) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50100624) = 0x03600448; // Mask offset and count
  *((volatile uint32_t *) 0x501006a4) = 0x00000003; // TRAM ptr max

  // Layer 5 group 1
  *((volatile uint32_t *) 0x50500024) = 0x00010005; // Rows
  *((volatile uint32_t *) 0x505000a4) = 0x00010005; // Columns
  *((volatile uint32_t *) 0x50500324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50500424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x505005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50500a24) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50500624) = 0x03600448; // Mask offset and count
  *((volatile uint32_t *) 0x505006a4) = 0x00000003; // TRAM ptr max

  // Layer 5 group 2
  *((volatile uint32_t *) 0x50900024) = 0x00010005; // Rows
  *((volatile uint32_t *) 0x509000a4) = 0x00010005; // Columns
  *((volatile uint32_t *) 0x50900324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50900424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x509005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50900a24) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50900624) = 0x03600448; // Mask offset and count
  *((volatile uint32_t *) 0x509006a4) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x50900724) = 0xfffcfffc; // Mask and processor enables

  // Layer 5 group 3
  *((volatile uint32_t *) 0x50d00024) = 0x00010005; // Rows
  *((volatile uint32_t *) 0x50d000a4) = 0x00010005; // Columns
  *((volatile uint32_t *) 0x50d00324) = 0x00000800; // SRAM write ptr
  *((volatile uint32_t *) 0x50d00424) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a4) = 0x00000001; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d005a4) = 0x00000b20; // Layer control
  *((volatile uint32_t *) 0x50d00a24) = 0x0000e800; // Layer control 2
  *((volatile uint32_t *) 0x50d00624) = 0x03600448; // Mask offset and count
  *((volatile uint32_t *) 0x50d006a4) = 0x00000003; // TRAM ptr max
  *((volatile uint32_t *) 0x50d00724) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 0
  *((volatile uint32_t *) 0x50100328) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x501003a8) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50100428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x501004a8) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50100528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x501005a8) = 0x00012920; // Layer control
  *((volatile uint32_t *) 0x50100a28) = 0x0000080f; // Layer control 2
  *((volatile uint32_t *) 0x50100628) = 0x26402738; // Mask offset and count
  *((volatile uint32_t *) 0x50100128) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x501007a8) = 0x00001000; // Post processing register
  *((volatile uint32_t *) 0x50100728) = 0xffffffff; // Mask and processor enables

  // Layer 6 group 1
  *((volatile uint32_t *) 0x50500328) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x505003a8) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50500428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x505004a8) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50500528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x505005a8) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50500a28) = 0x0000080f; // Layer control 2
  *((volatile uint32_t *) 0x50500628) = 0x26402738; // Mask offset and count
  *((volatile uint32_t *) 0x50500128) = 0x00000100; // 1D
  *((volatile uint32_t *) 0x50500728) = 0x3fff3fff; // Mask and processor enables

  // Layer 6 group 2
  *((volatile uint32_t *) 0x50900328) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x509003a8) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50900428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x509004a8) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50900528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x509005a8) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50900a28) = 0x0000080f; // Layer control 2
  *((volatile uint32_t *) 0x50900628) = 0x26402738; // Mask offset and count
  *((volatile uint32_t *) 0x50900128) = 0x00000100; // 1D

  // Layer 6 group 3
  *((volatile uint32_t *) 0x50d00328) = 0x00000400; // SRAM write ptr
  *((volatile uint32_t *) 0x50d003a8) = 0x00000001; // Write ptr time slot offs
  *((volatile uint32_t *) 0x50d00428) = 0x00002000; // Write ptr mask offs
  *((volatile uint32_t *) 0x50d004a8) = 0x00000004; // Write ptr multi-pass channel offs
  *((volatile uint32_t *) 0x50d00528) = 0x00000800; // SRAM read ptr
  *((volatile uint32_t *) 0x50d005a8) = 0x00010920; // Layer control
  *((volatile uint32_t *) 0x50d00a28) = 0x0000080f; // Layer control 2
  *((volatile uint32_t *) 0x50d00628) = 0x26402738; // Mask offset and count
  *((volatile uint32_t *) 0x50d00128) = 0x00000100; // 1D

  *((volatile uint32_t *) 0x50000000) = 0x00007908; // FIFO control
  *((volatile uint32_t *) 0x50100000) = 0x00108808; // Enable group 0
  *((volatile uint32_t *) 0x50500000) = 0x00108809; // Enable group 1
  *((volatile uint32_t *) 0x50900000) = 0x00108809; // Enable group 2
  *((volatile uint32_t *) 0x50d00000) = 0x00108809; // Enable group 3

  CNN_START; // Allow capture of processing time
  *((volatile uint32_t *) 0x50100000) = 0x00108809; // Master enable group 0

  load_input(); // Load data input


  return 1;
}

/* **************************************************************************** */
// cats_vs_dogs_chw-fifo
// Expected output of layer 6
int cnn_check(void)
{
  int rv = 1;
  if ((*((volatile uint32_t *) 0x50401000)) != 0x000009b9) return 0; // 0,0,0
  if ((*((volatile uint32_t *) 0x50401004)) != 0xfffffb88) return 0; // 0,0,1
  return rv;
}

/* **************************************************************************** */
// Custom unload for this network:
// 32-bit data, shape: [2, 1, 1]
void cnn_unload(uint32_t *out_buf)
{
  volatile uint32_t *addr;

  addr = (volatile uint32_t *) 0x50401000;
  *out_buf++ = *addr++;
  *out_buf++ = *addr++;
}

// Classification layer:
static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];

/* **************************************************************************** */
int softmax_layer(void)
{
  cnn_unload((uint32_t *) ml_data);
  softmax_q17p14_q15((const q31_t *) ml_data, NUM_OUTPUTS, ml_softmax);

  return 1;
}

/* **************************************************************************** */
static uint8_t signed_to_unsigned(int8_t val) {
        uint8_t value;
        if (val < 0) {
                value = ~val + 1;
                return (128 - value);
        }
        return val + 128;
}

/* **************************************************************************** */
int8_t unsigned_to_signed(uint8_t val) {
        return val - 128;
}

/* **************************************************************************** */
void TFT_Print(char *str, int x, int y, int font) {
  // fonts id
  text_t text;
  text.data = str;
  text.len = 36;

  MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

/* **************************************************************************** */
void lcd_show_sampledata(uint32_t *data0, uint32_t *data1, uint32_t *data2, int length) {
  int i;
  int j;
  int x;
  int y;
  int r;
  int g;
  int b;
  int scale = 2.2;
    
  uint32_t color;
  uint8_t *ptr0;
  uint8_t *ptr1;
  uint8_t *ptr2;

  x = 47;
  y = 15;
  for (i = 0; i < length; i++) {
    ptr0 = (uint8_t *)&data0[i];
    ptr1 = (uint8_t *)&data1[i];
    ptr2 = (uint8_t *)&data2[i];
    for (j = 0; j < 4; j++) {
      r = ptr0[j];
      g = ptr1[j];
      b = ptr2[j];        
      color  = (0x01000100 | ((b & 0xF8) << 13) | ((g & 0x1C) << 19) | ((g & 0xE0) >> 5) | (r & 0xF8));
      MXC_TFT_WritePixel(x * scale, y * scale, scale, scale, color);
      x += 1;
      if (x >= (IMAGE_SIZE_X + 47)) {
        x = 47;
        y += 1;
        if ((y + 6) >= (IMAGE_SIZE_Y + 15)) return;
      }
    }
  }
}

/* **************************************************************************** */
void process_camera_img(uint32_t *data0, uint32_t *data1, uint32_t *data2)
{
 	uint8_t   *frame_buffer;
	uint32_t  imgLen;
	uint32_t  w, h, x, y;
  uint8_t *ptr0;
  uint8_t *ptr1;
  uint8_t *ptr2;
  uint8_t *buffer;

	camera_get_image(&frame_buffer, &imgLen, &w, &h);
  ptr0 = (uint8_t *)data0;
  ptr1 = (uint8_t *)data1;
  ptr2 = (uint8_t *)data2;
  buffer = frame_buffer;
  for (y = 0; y < h; y++) {
    for (x = 0; x < w; x++, ptr0++, ptr1++, ptr2++) {
      *ptr0 = (*buffer); buffer++;
      *ptr1 = (*buffer); buffer++;
      *ptr2 = (*buffer); buffer++;
    }
  }
}

/* **************************************************************************** */
void capture_camera_img(void) {
  camera_start_capture_image();
  while (1) {
    if (camera_is_image_rcv()) {
      return;
    }
  }
}

/* **************************************************************************** */
void convert_img_unsigned_to_signed(uint32_t *data0, uint32_t *data1, uint32_t *data2) {
  uint8_t *ptr0;
  uint8_t *ptr1;
  uint8_t *ptr2;
  ptr0 = (uint8_t *)data0;
  ptr1 = (uint8_t *)data1;
  ptr2 = (uint8_t *)data2;
  for(int i=0; i<4096; i++) {
    *ptr0 = unsigned_to_signed(*ptr0); ptr0++;
    *ptr1 = unsigned_to_signed(*ptr1); ptr1++;
    *ptr2 = unsigned_to_signed(*ptr2); ptr2++;
  }
}

/* **************************************************************************** */
void convert_img_signed_to_unsigned(uint32_t *data0, uint32_t *data1, uint32_t *data2) {
  uint8_t *ptr0;
  uint8_t *ptr1;
  uint8_t *ptr2;
  ptr0 = (uint8_t *)data0;
  ptr1 = (uint8_t *)data1;
  ptr2 = (uint8_t *)data2;
  for(int i=0; i<4096; i++) {
    *ptr0 = signed_to_unsigned(*ptr0); ptr0++;
    *ptr1 = signed_to_unsigned(*ptr1); ptr1++;
    *ptr2 = signed_to_unsigned(*ptr2); ptr2++;
  }
}

/* **************************************************************************** */
int main(void)
{
  int i;
  int digs, tens;
  int ret = 0;
  char buff[TFT_BUFF_SIZE];
  int result[NUM_OUTPUTS] = {0};

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

  // Configure P2.5, turn on the CNN Boost
  mxc_gpio_cfg_t gpio_out;
  gpio_out.port = MXC_GPIO2;
  gpio_out.mask = MXC_GPIO_PIN_5;
  gpio_out.pad = MXC_GPIO_PAD_NONE;
  gpio_out.func = MXC_GPIO_FUNC_OUT;
  MXC_GPIO_Config(&gpio_out);
  MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

  printf("\n*** CNN Test ***\n");

  // Initialize TFT display.
  printf("Init LCD.\n");
  mxc_gpio_cfg_t tft_reset_pin = {MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
  MXC_TFT_Init(MXC_SPI0, 1, &tft_reset_pin, NULL);
  MXC_TFT_ClearScreen();
  MXC_TFT_ShowImage(0, 0, img_1_bmp);

  // Initialize camera.
  printf("Init Camera.\n");
  camera_init();

  ret = camera_setup(IMAGE_SIZE_X, IMAGE_SIZE_Y, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA);
  if (ret != STATUS_OK) {
      printf("Error returned from setting up camera. Error %d\n", ret);
      return -1;
  }

  MXC_Delay(1000000);
  MXC_TFT_SetPalette(logo_white_bg_darkgrey_bmp);
  MXC_TFT_SetBackGroundColor(4);

  MXC_TFT_ShowImage(1, 1, logo_white_bg_darkgrey_bmp);

  memset(buff,32,TFT_BUFF_SIZE);
  sprintf(buff, "MAXIM INTEGRATED             ");
  TFT_Print(buff, 55, 50, urw_gothic_13_white_bg_grey);

  sprintf(buff, "Cats-vs-Dogs Demo        ");
  TFT_Print(buff, 55, 90, urw_gothic_12_white_bg_grey);

  sprintf(buff, "PRESS PB1 TO START!          ");
  TFT_Print(buff, 55, 130, urw_gothic_13_white_bg_grey);

  int frame = 0;

  while (1) {
    printf("********** Press PB1 to capture an image **********\r\n");
    while(!PB_Get(0));
    MXC_TFT_ClearScreen();
    MXC_TFT_ShowImage(1, 1, logo_white_bg_darkgrey_bmp);
    sprintf(buff, "CAPTURING IMAGE....           ");
    TFT_Print(buff, 55, 110, urw_gothic_13_white_bg_grey);

#ifdef USE_SAMPLEDATA
    // Copy the sampledata reference to the camera buffer as a test.
    printf("\nCapturing sampledata %d times\n", ++frame);
    memcpy32(input_0_camera, input_0, 1024);
    memcpy32(input_1_camera, input_1, 1024);
    memcpy32(input_2_camera, input_2, 1024);
    convert_img_signed_to_unsigned(input_0_camera, input_1_camera, input_2_camera);
#else
    // Capture a single camera frame.
    printf("\nCapture a camera frame %d\n", ++frame);
    capture_camera_img();
    // Copy the image data to the CNN input arrays.
    printf("Copy camera frame to CNN input buffers.\n");
    process_camera_img(input_0_camera, input_1_camera, input_2_camera);
#endif

    // Show the input data on the lcd.
    MXC_TFT_ClearScreen();
    MXC_TFT_ShowImage(1, 1, logo_white_bg_darkgrey_bmp);
    printf("Show camera frame on LCD.\n");
    lcd_show_sampledata(input_0_camera, input_1_camera, input_2_camera, 1024);
    convert_img_unsigned_to_signed(input_0_camera, input_1_camera, input_2_camera);

    if (!cnn_load()) fail();
    MXC_TMR_SW_Start(MXC_TMR0);
    cnn_wait();

    if (!softmax_layer()) fail();

    printf("Time for CNN: %d us\n\n", cnn_time);

    printf("Classification results:\n");
    for (i = 0; i < NUM_OUTPUTS; i++) {
      digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
      tens = digs % 10;
      digs = digs / 10;
      result[i] = digs;
      printf("[%7d] -> Class %d %8s: %d.%d%%\r\n", ml_data[i], i, classes[i], digs, tens);
    }
    printf("\n");
    memset(buff,32,TFT_BUFF_SIZE);
    sprintf(buff, "Image Detected :                ");
    TFT_Print(buff, 10, 150, urw_gothic_12_white_bg_grey);
    memset(buff,0,TFT_BUFF_SIZE);
    sprintf(buff, "Probability :                   ");
    TFT_Print(buff, 10, 180, urw_gothic_12_white_bg_grey);

    memset(buff,32,TFT_BUFF_SIZE);
    if (result[0] > result[1]) {
      sprintf(buff, "CAT                           ");
      TFT_Print(buff, 195, 150, urw_gothic_12_white_bg_grey);
      sprintf(buff, "%d%%", result[0]);
      TFT_Print(buff, 135, 180, urw_gothic_12_white_bg_grey);

    } else if (result[1] > result[0]) {
      sprintf(buff, "DOG                           ");
      TFT_Print(buff, 195, 150, urw_gothic_12_white_bg_grey);
      sprintf(buff, "%d%%", result[1]);
      TFT_Print(buff, 135, 180, urw_gothic_12_white_bg_grey);

    } else {
      sprintf(buff, "Unknown                       ");
      TFT_Print(buff, 195, 150, urw_gothic_12_white_bg_grey);
      memset(buff,32,TFT_BUFF_SIZE);
      sprintf(buff, "NA                            ");
      TFT_Print(buff, 135, 180, urw_gothic_12_white_bg_grey);
    }

    sprintf(buff, "PRESS PB1 TO CAPTURE IMAGE      ");
    TFT_Print(buff, 10, 210, urw_gothic_12_white_bg_grey);
  }

  return 0;
}
