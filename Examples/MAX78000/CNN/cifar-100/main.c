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

// cifar-100
// Created using ./ai8xize.py --verbose --log --test-dir sdk/Examples/MAX78000/CNN --prefix cifar-100 --checkpoint-file trained/ai85-cifar100-qat8-q.pth.tar --config-file networks/cifar100-simple.yaml --softmax --device MAX78000 --compact-data --mexpress --timer 0 --display-checkpoint --boost 2.5

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "mxc.h"
#include "cnn.h"
#include "sampledata.h"

volatile uint32_t cnn_time; // Stopwatch

void fail(void)
{
  printf("\n*** FAIL ***\n\n");
  while (1);
}

// 3-channel 32x32 data input (3072 bytes total / 1024 bytes per channel):
// HWC 32x32, channels 0 to 2
static const uint32_t input_0[] = SAMPLE_INPUT_0;

void load_input(void)
{
  // This function loads the sample data input -- replace with actual data

  memcpy32((uint32_t *) 0x50400000, input_0, 1024);
}

// Expected output of layer 13 for cifar-100 given the sample input
int check_output(void)
{
  if ((*((volatile uint32_t *) 0x50400000)) != 0x000416b6) return CNN_FAIL; // 0,0,0
  if ((*((volatile uint32_t *) 0x50400004)) != 0x00004938) return CNN_FAIL; // 0,0,1
  if ((*((volatile uint32_t *) 0x50400008)) != 0xfffff657) return CNN_FAIL; // 0,0,2
  if ((*((volatile uint32_t *) 0x5040000c)) != 0xffff8016) return CNN_FAIL; // 0,0,3
  if ((*((volatile uint32_t *) 0x50408000)) != 0xfffe6bda) return CNN_FAIL; // 0,0,4
  if ((*((volatile uint32_t *) 0x50408004)) != 0x000079fa) return CNN_FAIL; // 0,0,5
  if ((*((volatile uint32_t *) 0x50408008)) != 0x00000af9) return CNN_FAIL; // 0,0,6
  if ((*((volatile uint32_t *) 0x5040800c)) != 0xffff6250) return CNN_FAIL; // 0,0,7
  if ((*((volatile uint32_t *) 0x50410000)) != 0xffff358e) return CNN_FAIL; // 0,0,8
  if ((*((volatile uint32_t *) 0x50410004)) != 0x000217e9) return CNN_FAIL; // 0,0,9
  if ((*((volatile uint32_t *) 0x50410008)) != 0x0001d0c1) return CNN_FAIL; // 0,0,10
  if ((*((volatile uint32_t *) 0x5041000c)) != 0xffff5256) return CNN_FAIL; // 0,0,11
  if ((*((volatile uint32_t *) 0x50418000)) != 0xfffdc056) return CNN_FAIL; // 0,0,12
  if ((*((volatile uint32_t *) 0x50418004)) != 0x00007407) return CNN_FAIL; // 0,0,13
  if ((*((volatile uint32_t *) 0x50418008)) != 0x000022a0) return CNN_FAIL; // 0,0,14
  if ((*((volatile uint32_t *) 0x5041800c)) != 0x00002126) return CNN_FAIL; // 0,0,15
  if ((*((volatile uint32_t *) 0x50800000)) != 0x0001532b) return CNN_FAIL; // 0,0,16
  if ((*((volatile uint32_t *) 0x50800004)) != 0xffff8e1b) return CNN_FAIL; // 0,0,17
  if ((*((volatile uint32_t *) 0x50800008)) != 0x0000c441) return CNN_FAIL; // 0,0,18
  if ((*((volatile uint32_t *) 0x5080000c)) != 0x00002c46) return CNN_FAIL; // 0,0,19
  if ((*((volatile uint32_t *) 0x50808000)) != 0xffff33da) return CNN_FAIL; // 0,0,20
  if ((*((volatile uint32_t *) 0x50808004)) != 0xfffd948c) return CNN_FAIL; // 0,0,21
  if ((*((volatile uint32_t *) 0x50808008)) != 0x000058ab) return CNN_FAIL; // 0,0,22
  if ((*((volatile uint32_t *) 0x5080800c)) != 0x00006ca3) return CNN_FAIL; // 0,0,23
  if ((*((volatile uint32_t *) 0x50810000)) != 0xffff81d9) return CNN_FAIL; // 0,0,24
  if ((*((volatile uint32_t *) 0x50810004)) != 0x000179a7) return CNN_FAIL; // 0,0,25
  if ((*((volatile uint32_t *) 0x50810008)) != 0xffffa4e7) return CNN_FAIL; // 0,0,26
  if ((*((volatile uint32_t *) 0x5081000c)) != 0xfffdd2c4) return CNN_FAIL; // 0,0,27
  if ((*((volatile uint32_t *) 0x50818000)) != 0x00007756) return CNN_FAIL; // 0,0,28
  if ((*((volatile uint32_t *) 0x50818004)) != 0xffff3267) return CNN_FAIL; // 0,0,29
  if ((*((volatile uint32_t *) 0x50818008)) != 0xfffd4ead) return CNN_FAIL; // 0,0,30
  if ((*((volatile uint32_t *) 0x5081800c)) != 0xffff4240) return CNN_FAIL; // 0,0,31
  if ((*((volatile uint32_t *) 0x50c00000)) != 0x0000a9a5) return CNN_FAIL; // 0,0,32
  if ((*((volatile uint32_t *) 0x50c00004)) != 0x00004b00) return CNN_FAIL; // 0,0,33
  if ((*((volatile uint32_t *) 0x50c00008)) != 0xfffeda17) return CNN_FAIL; // 0,0,34
  if ((*((volatile uint32_t *) 0x50c0000c)) != 0xffff1e0a) return CNN_FAIL; // 0,0,35
  if ((*((volatile uint32_t *) 0x50c08000)) != 0xffff505b) return CNN_FAIL; // 0,0,36
  if ((*((volatile uint32_t *) 0x50c08004)) != 0xfffe76f5) return CNN_FAIL; // 0,0,37
  if ((*((volatile uint32_t *) 0x50c08008)) != 0xfffd0699) return CNN_FAIL; // 0,0,38
  if ((*((volatile uint32_t *) 0x50c0800c)) != 0x000119b4) return CNN_FAIL; // 0,0,39
  if ((*((volatile uint32_t *) 0x50c10000)) != 0x00006811) return CNN_FAIL; // 0,0,40
  if ((*((volatile uint32_t *) 0x50c10004)) != 0xfffe8a56) return CNN_FAIL; // 0,0,41
  if ((*((volatile uint32_t *) 0x50c10008)) != 0xfffda76a) return CNN_FAIL; // 0,0,42
  if ((*((volatile uint32_t *) 0x50c1000c)) != 0xfffeb44b) return CNN_FAIL; // 0,0,43
  if ((*((volatile uint32_t *) 0x50c18000)) != 0xffff1bfb) return CNN_FAIL; // 0,0,44
  if ((*((volatile uint32_t *) 0x50c18004)) != 0x0000f390) return CNN_FAIL; // 0,0,45
  if ((*((volatile uint32_t *) 0x50c18008)) != 0xffff2ddd) return CNN_FAIL; // 0,0,46
  if ((*((volatile uint32_t *) 0x50c1800c)) != 0x0001743a) return CNN_FAIL; // 0,0,47
  if ((*((volatile uint32_t *) 0x51000000)) != 0xfffe93ab) return CNN_FAIL; // 0,0,48
  if ((*((volatile uint32_t *) 0x51000004)) != 0xfffec2a2) return CNN_FAIL; // 0,0,49
  if ((*((volatile uint32_t *) 0x51000008)) != 0xfffe75a2) return CNN_FAIL; // 0,0,50
  if ((*((volatile uint32_t *) 0x5100000c)) != 0x00003336) return CNN_FAIL; // 0,0,51
  if ((*((volatile uint32_t *) 0x50400010)) != 0x00000995) return CNN_FAIL; // 0,0,52
  if ((*((volatile uint32_t *) 0x50400014)) != 0x00026cf4) return CNN_FAIL; // 0,0,53
  if ((*((volatile uint32_t *) 0x50400018)) != 0x000052bc) return CNN_FAIL; // 0,0,54
  if ((*((volatile uint32_t *) 0x5040001c)) != 0xfffe9089) return CNN_FAIL; // 0,0,55
  if ((*((volatile uint32_t *) 0x50408010)) != 0x00008189) return CNN_FAIL; // 0,0,56
  if ((*((volatile uint32_t *) 0x50408014)) != 0x0003d280) return CNN_FAIL; // 0,0,57
  if ((*((volatile uint32_t *) 0x50408018)) != 0xffffe7b5) return CNN_FAIL; // 0,0,58
  if ((*((volatile uint32_t *) 0x5040801c)) != 0xffff41f6) return CNN_FAIL; // 0,0,59
  if ((*((volatile uint32_t *) 0x50410010)) != 0x0000eb24) return CNN_FAIL; // 0,0,60
  if ((*((volatile uint32_t *) 0x50410014)) != 0x000013ad) return CNN_FAIL; // 0,0,61
  if ((*((volatile uint32_t *) 0x50410018)) != 0x00007e9b) return CNN_FAIL; // 0,0,62
  if ((*((volatile uint32_t *) 0x5041001c)) != 0xfffe4680) return CNN_FAIL; // 0,0,63
  if ((*((volatile uint32_t *) 0x50418010)) != 0xfffdd81f) return CNN_FAIL; // 0,0,64
  if ((*((volatile uint32_t *) 0x50418014)) != 0xffff0639) return CNN_FAIL; // 0,0,65
  if ((*((volatile uint32_t *) 0x50418018)) != 0xfffb2fac) return CNN_FAIL; // 0,0,66
  if ((*((volatile uint32_t *) 0x5041801c)) != 0x0000ce67) return CNN_FAIL; // 0,0,67
  if ((*((volatile uint32_t *) 0x50800010)) != 0xfffe306e) return CNN_FAIL; // 0,0,68
  if ((*((volatile uint32_t *) 0x50800014)) != 0x00008b5e) return CNN_FAIL; // 0,0,69
  if ((*((volatile uint32_t *) 0x50800018)) != 0x00009927) return CNN_FAIL; // 0,0,70
  if ((*((volatile uint32_t *) 0x5080001c)) != 0x00005cd5) return CNN_FAIL; // 0,0,71
  if ((*((volatile uint32_t *) 0x50808010)) != 0xffff5343) return CNN_FAIL; // 0,0,72
  if ((*((volatile uint32_t *) 0x50808014)) != 0xfffe9bef) return CNN_FAIL; // 0,0,73
  if ((*((volatile uint32_t *) 0x50808018)) != 0xfffce6b5) return CNN_FAIL; // 0,0,74
  if ((*((volatile uint32_t *) 0x5080801c)) != 0xfffb712f) return CNN_FAIL; // 0,0,75
  if ((*((volatile uint32_t *) 0x50810010)) != 0xffffaa12) return CNN_FAIL; // 0,0,76
  if ((*((volatile uint32_t *) 0x50810014)) != 0x00006b76) return CNN_FAIL; // 0,0,77
  if ((*((volatile uint32_t *) 0x50810018)) != 0xffff12ae) return CNN_FAIL; // 0,0,78
  if ((*((volatile uint32_t *) 0x5081001c)) != 0xffffedbd) return CNN_FAIL; // 0,0,79
  if ((*((volatile uint32_t *) 0x50818010)) != 0xffff9008) return CNN_FAIL; // 0,0,80
  if ((*((volatile uint32_t *) 0x50818014)) != 0x000047e4) return CNN_FAIL; // 0,0,81
  if ((*((volatile uint32_t *) 0x50818018)) != 0x00010b7e) return CNN_FAIL; // 0,0,82
  if ((*((volatile uint32_t *) 0x5081801c)) != 0x0003fe63) return CNN_FAIL; // 0,0,83
  if ((*((volatile uint32_t *) 0x50c00010)) != 0x0000aa4f) return CNN_FAIL; // 0,0,84
  if ((*((volatile uint32_t *) 0x50c00014)) != 0xffff4706) return CNN_FAIL; // 0,0,85
  if ((*((volatile uint32_t *) 0x50c00018)) != 0xffffe47c) return CNN_FAIL; // 0,0,86
  if ((*((volatile uint32_t *) 0x50c0001c)) != 0xffffeead) return CNN_FAIL; // 0,0,87
  if ((*((volatile uint32_t *) 0x50c08010)) != 0x00003bf0) return CNN_FAIL; // 0,0,88
  if ((*((volatile uint32_t *) 0x50c08014)) != 0x00005846) return CNN_FAIL; // 0,0,89
  if ((*((volatile uint32_t *) 0x50c08018)) != 0xffff7713) return CNN_FAIL; // 0,0,90
  if ((*((volatile uint32_t *) 0x50c0801c)) != 0xfffe8bee) return CNN_FAIL; // 0,0,91
  if ((*((volatile uint32_t *) 0x50c10010)) != 0x00028fd3) return CNN_FAIL; // 0,0,92
  if ((*((volatile uint32_t *) 0x50c10014)) != 0xfffee186) return CNN_FAIL; // 0,0,93
  if ((*((volatile uint32_t *) 0x50c10018)) != 0x00017a26) return CNN_FAIL; // 0,0,94
  if ((*((volatile uint32_t *) 0x50c1001c)) != 0xfffe6829) return CNN_FAIL; // 0,0,95
  if ((*((volatile uint32_t *) 0x50c18010)) != 0x00015200) return CNN_FAIL; // 0,0,96
  if ((*((volatile uint32_t *) 0x50c18014)) != 0xfffd8f91) return CNN_FAIL; // 0,0,97
  if ((*((volatile uint32_t *) 0x50c18018)) != 0xffff8341) return CNN_FAIL; // 0,0,98
  if ((*((volatile uint32_t *) 0x50c1801c)) != 0xffffb9dc) return CNN_FAIL; // 0,0,99

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

  MXC_ICC_Enable(MXC_ICC0); // Enable cache

  // Switch to 100 MHz clock
  MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
  SystemCoreClockUpdate();

  printf("Waiting...\n");

  // DO NOT DELETE THIS LINE:
  MXC_Delay(SEC(2)); // Let debugger interrupt if needed

  // Enable peripheral, enable CNN interrupt, turn on CNN clock
  // CNN clock: 50 MHz div 1
  cnn_enable(MXC_S_GCR_PCLKDIV_CNNCLKSEL_PCLK, MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1);
  cnn_boost_enable(MXC_GPIO2, MXC_GPIO_PIN_5); // Turn on the boost circuit

  printf("\n*** CNN Inference Test ***\n");

  cnn_init(); // Bring state machine into consistent state
  cnn_load_weights(); // Load kernels
  // cnn_load_bias(); // Not used in this network
  cnn_configure(); // Configure state machine
  load_input(); // Load data input
  cnn_start(); // Start CNN processing

  while (cnn_time == 0)
    __WFI(); // Wait for CNN

  cnn_boost_disable(MXC_GPIO2, MXC_GPIO_PIN_5); // Turn off the boost circuit

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

  return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 18,607,744 ops (18,461,184 macc; 146,560 comp; 0 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 381,792 bytes out of 442,368 bytes total (86%)
  Bias memory:   0 bytes out of 2,048 bytes total (0%)
*/

