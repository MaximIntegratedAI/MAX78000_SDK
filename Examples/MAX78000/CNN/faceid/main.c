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

// faceid
// Created using ./ai8xize.py --verbose --log --test-dir sdk/Examples/MAX78000/CNN --prefix faceid --checkpoint-file trained/ai85-faceid-qat8-q.pth.tar --config-file networks/faceid.yaml --fifo --device MAX78000 --compact-data --mexpress --timer 0 --display-checkpoint

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

// Data input: HWC 3x160x120 (57600 bytes total / 19200 bytes per channel):
static const uint32_t input_0[] = SAMPLE_INPUT_0;
void load_input(void)
{
  // This function loads the sample data input -- replace with actual data

  int i;
  const uint32_t *in0 = input_0;

  for (i = 0; i < 19200; i++) {
    while (((*((volatile uint32_t *) 0x50000004) & 1)) != 0); // Wait for FIFO 0
    *((volatile uint32_t *) 0x50000008) = *in0++; // Write FIFO 0
  }
}

// Expected output of layer 8 for faceid given the sample input
int check_output(void)
{
  if ((*((volatile uint32_t *) 0x50400000)) != 0x050aff04) return CNN_FAIL; // 0,0,0-3
  if ((*((volatile uint32_t *) 0x50408000)) != 0xfd05e6e1) return CNN_FAIL; // 0,0,4-7
  if ((*((volatile uint32_t *) 0x50410000)) != 0xfc00e20a) return CNN_FAIL; // 0,0,8-11
  if ((*((volatile uint32_t *) 0x50418000)) != 0xfcf4f6fb) return CNN_FAIL; // 0,0,12-15
  if ((*((volatile uint32_t *) 0x50800000)) != 0x0af10000) return CNN_FAIL; // 0,0,16-19
  if ((*((volatile uint32_t *) 0x50808000)) != 0xf4f905fb) return CNN_FAIL; // 0,0,20-23
  if ((*((volatile uint32_t *) 0x50810000)) != 0xfaf8f900) return CNN_FAIL; // 0,0,24-27
  if ((*((volatile uint32_t *) 0x50818000)) != 0x122a1600) return CNN_FAIL; // 0,0,28-31
  if ((*((volatile uint32_t *) 0x50c00000)) != 0xfbfa0b0f) return CNN_FAIL; // 0,0,32-35
  if ((*((volatile uint32_t *) 0x50c08000)) != 0x090bf8f6) return CNN_FAIL; // 0,0,36-39
  if ((*((volatile uint32_t *) 0x50c10000)) != 0x0a20e115) return CNN_FAIL; // 0,0,40-43
  if ((*((volatile uint32_t *) 0x50c18000)) != 0xfbe5f7f2) return CNN_FAIL; // 0,0,44-47
  if ((*((volatile uint32_t *) 0x51000000)) != 0x1821fe04) return CNN_FAIL; // 0,0,48-51
  if ((*((volatile uint32_t *) 0x51008000)) != 0xf1fa0d08) return CNN_FAIL; // 0,0,52-55
  if ((*((volatile uint32_t *) 0x51010000)) != 0x09f503f7) return CNN_FAIL; // 0,0,56-59
  if ((*((volatile uint32_t *) 0x51018000)) != 0x0af917ed) return CNN_FAIL; // 0,0,60-63
  if ((*((volatile uint32_t *) 0x50400004)) != 0x09ec19ff) return CNN_FAIL; // 0,0,64-67
  if ((*((volatile uint32_t *) 0x50408004)) != 0x130aef08) return CNN_FAIL; // 0,0,68-71
  if ((*((volatile uint32_t *) 0x50410004)) != 0xf9f1fc01) return CNN_FAIL; // 0,0,72-75
  if ((*((volatile uint32_t *) 0x50418004)) != 0x0fe8e6fe) return CNN_FAIL; // 0,0,76-79
  if ((*((volatile uint32_t *) 0x50800004)) != 0xfcfde8f8) return CNN_FAIL; // 0,0,80-83
  if ((*((volatile uint32_t *) 0x50808004)) != 0xf3fff922) return CNN_FAIL; // 0,0,84-87
  if ((*((volatile uint32_t *) 0x50810004)) != 0xf8161508) return CNN_FAIL; // 0,0,88-91
  if ((*((volatile uint32_t *) 0x50818004)) != 0xfe03f715) return CNN_FAIL; // 0,0,92-95
  if ((*((volatile uint32_t *) 0x50c00004)) != 0x0afd0206) return CNN_FAIL; // 0,0,96-99
  if ((*((volatile uint32_t *) 0x50c08004)) != 0x14121802) return CNN_FAIL; // 0,0,100-103
  if ((*((volatile uint32_t *) 0x50c10004)) != 0x0f1a0f1b) return CNN_FAIL; // 0,0,104-107
  if ((*((volatile uint32_t *) 0x50c18004)) != 0x0914120c) return CNN_FAIL; // 0,0,108-111
  if ((*((volatile uint32_t *) 0x51000004)) != 0xf3e5fa03) return CNN_FAIL; // 0,0,112-115
  if ((*((volatile uint32_t *) 0x51008004)) != 0x06f8f2e9) return CNN_FAIL; // 0,0,116-119
  if ((*((volatile uint32_t *) 0x51010004)) != 0x12e01c01) return CNN_FAIL; // 0,0,120-123
  if ((*((volatile uint32_t *) 0x51018004)) != 0xf50f03ee) return CNN_FAIL; // 0,0,124-127
  if ((*((volatile uint32_t *) 0x50400008)) != 0x11000111) return CNN_FAIL; // 0,0,128-131
  if ((*((volatile uint32_t *) 0x50408008)) != 0x01021105) return CNN_FAIL; // 0,0,132-135
  if ((*((volatile uint32_t *) 0x50410008)) != 0x1005daf0) return CNN_FAIL; // 0,0,136-139
  if ((*((volatile uint32_t *) 0x50418008)) != 0x08edfd01) return CNN_FAIL; // 0,0,140-143
  if ((*((volatile uint32_t *) 0x50800008)) != 0xf5fdf4f7) return CNN_FAIL; // 0,0,144-147
  if ((*((volatile uint32_t *) 0x50808008)) != 0xfe190000) return CNN_FAIL; // 0,0,148-151
  if ((*((volatile uint32_t *) 0x50810008)) != 0x0c0c0409) return CNN_FAIL; // 0,0,152-155
  if ((*((volatile uint32_t *) 0x50818008)) != 0x16fdff09) return CNN_FAIL; // 0,0,156-159
  if ((*((volatile uint32_t *) 0x50c00008)) != 0xfc03fb1d) return CNN_FAIL; // 0,0,160-163
  if ((*((volatile uint32_t *) 0x50c08008)) != 0x0afd0b0f) return CNN_FAIL; // 0,0,164-167
  if ((*((volatile uint32_t *) 0x50c10008)) != 0x14ecf1f5) return CNN_FAIL; // 0,0,168-171
  if ((*((volatile uint32_t *) 0x50c18008)) != 0xff000aef) return CNN_FAIL; // 0,0,172-175
  if ((*((volatile uint32_t *) 0x51000008)) != 0xf809f1f8) return CNN_FAIL; // 0,0,176-179
  if ((*((volatile uint32_t *) 0x51008008)) != 0xf8ff07fb) return CNN_FAIL; // 0,0,180-183
  if ((*((volatile uint32_t *) 0x51010008)) != 0x040518fc) return CNN_FAIL; // 0,0,184-187
  if ((*((volatile uint32_t *) 0x51018008)) != 0xef0d1cfc) return CNN_FAIL; // 0,0,188-191
  if ((*((volatile uint32_t *) 0x5040000c)) != 0x0100f1e7) return CNN_FAIL; // 0,0,192-195
  if ((*((volatile uint32_t *) 0x5040800c)) != 0xf60d0c03) return CNN_FAIL; // 0,0,196-199
  if ((*((volatile uint32_t *) 0x5041000c)) != 0x00fb050b) return CNN_FAIL; // 0,0,200-203
  if ((*((volatile uint32_t *) 0x5041800c)) != 0x0c00f414) return CNN_FAIL; // 0,0,204-207
  if ((*((volatile uint32_t *) 0x5080000c)) != 0x0d0012ee) return CNN_FAIL; // 0,0,208-211
  if ((*((volatile uint32_t *) 0x5080800c)) != 0xf8f10925) return CNN_FAIL; // 0,0,212-215
  if ((*((volatile uint32_t *) 0x5081000c)) != 0xf80a0108) return CNN_FAIL; // 0,0,216-219
  if ((*((volatile uint32_t *) 0x5081800c)) != 0xfffde900) return CNN_FAIL; // 0,0,220-223
  if ((*((volatile uint32_t *) 0x50c0000c)) != 0xfe15fbf5) return CNN_FAIL; // 0,0,224-227
  if ((*((volatile uint32_t *) 0x50c0800c)) != 0xf31efafe) return CNN_FAIL; // 0,0,228-231
  if ((*((volatile uint32_t *) 0x50c1000c)) != 0xf4f709f7) return CNN_FAIL; // 0,0,232-235
  if ((*((volatile uint32_t *) 0x50c1800c)) != 0xf4fb1aea) return CNN_FAIL; // 0,0,236-239
  if ((*((volatile uint32_t *) 0x5100000c)) != 0x0504f310) return CNN_FAIL; // 0,0,240-243
  if ((*((volatile uint32_t *) 0x5100800c)) != 0x2d1c0c0d) return CNN_FAIL; // 0,0,244-247
  if ((*((volatile uint32_t *) 0x5101000c)) != 0xf4f5e8e8) return CNN_FAIL; // 0,0,248-251
  if ((*((volatile uint32_t *) 0x5101800c)) != 0x0705f415) return CNN_FAIL; // 0,0,252-255
  if ((*((volatile uint32_t *) 0x50400010)) != 0x0913d91d) return CNN_FAIL; // 0,0,256-259
  if ((*((volatile uint32_t *) 0x50408010)) != 0x0502000c) return CNN_FAIL; // 0,0,260-263
  if ((*((volatile uint32_t *) 0x50410010)) != 0x11d80a23) return CNN_FAIL; // 0,0,264-267
  if ((*((volatile uint32_t *) 0x50418010)) != 0xfdf500fe) return CNN_FAIL; // 0,0,268-271
  if ((*((volatile uint32_t *) 0x50800010)) != 0x1b1f0916) return CNN_FAIL; // 0,0,272-275
  if ((*((volatile uint32_t *) 0x50808010)) != 0xfbf1f80e) return CNN_FAIL; // 0,0,276-279
  if ((*((volatile uint32_t *) 0x50810010)) != 0xfe0bf9ee) return CNN_FAIL; // 0,0,280-283
  if ((*((volatile uint32_t *) 0x50818010)) != 0x05e4d4ed) return CNN_FAIL; // 0,0,284-287
  if ((*((volatile uint32_t *) 0x50c00010)) != 0xfef21909) return CNN_FAIL; // 0,0,288-291
  if ((*((volatile uint32_t *) 0x50c08010)) != 0x0b0cddf9) return CNN_FAIL; // 0,0,292-295
  if ((*((volatile uint32_t *) 0x50c10010)) != 0x1cf3ddf0) return CNN_FAIL; // 0,0,296-299
  if ((*((volatile uint32_t *) 0x50c18010)) != 0xe213f3f6) return CNN_FAIL; // 0,0,300-303
  if ((*((volatile uint32_t *) 0x51000010)) != 0xfe000211) return CNN_FAIL; // 0,0,304-307
  if ((*((volatile uint32_t *) 0x51008010)) != 0x1004ea17) return CNN_FAIL; // 0,0,308-311
  if ((*((volatile uint32_t *) 0x51010010)) != 0xeb01f207) return CNN_FAIL; // 0,0,312-315
  if ((*((volatile uint32_t *) 0x51018010)) != 0x060608e8) return CNN_FAIL; // 0,0,316-319
  if ((*((volatile uint32_t *) 0x50400014)) != 0x1d0af2fa) return CNN_FAIL; // 0,0,320-323
  if ((*((volatile uint32_t *) 0x50408014)) != 0x05f5fc00) return CNN_FAIL; // 0,0,324-327
  if ((*((volatile uint32_t *) 0x50410014)) != 0x02fbfe00) return CNN_FAIL; // 0,0,328-331
  if ((*((volatile uint32_t *) 0x50418014)) != 0x08151df8) return CNN_FAIL; // 0,0,332-335
  if ((*((volatile uint32_t *) 0x50800014)) != 0xf70df705) return CNN_FAIL; // 0,0,336-339
  if ((*((volatile uint32_t *) 0x50808014)) != 0xe303e5fb) return CNN_FAIL; // 0,0,340-343
  if ((*((volatile uint32_t *) 0x50810014)) != 0xed25ee00) return CNN_FAIL; // 0,0,344-347
  if ((*((volatile uint32_t *) 0x50818014)) != 0x07f7fb02) return CNN_FAIL; // 0,0,348-351
  if ((*((volatile uint32_t *) 0x50c00014)) != 0x0620f120) return CNN_FAIL; // 0,0,352-355
  if ((*((volatile uint32_t *) 0x50c08014)) != 0x04fe08ef) return CNN_FAIL; // 0,0,356-359
  if ((*((volatile uint32_t *) 0x50c10014)) != 0xeb0201f1) return CNN_FAIL; // 0,0,360-363
  if ((*((volatile uint32_t *) 0x50c18014)) != 0xf9e5f21e) return CNN_FAIL; // 0,0,364-367
  if ((*((volatile uint32_t *) 0x51000014)) != 0xd50bf3ec) return CNN_FAIL; // 0,0,368-371
  if ((*((volatile uint32_t *) 0x51008014)) != 0x040610fd) return CNN_FAIL; // 0,0,372-375
  if ((*((volatile uint32_t *) 0x51010014)) != 0x22ff1bfe) return CNN_FAIL; // 0,0,376-379
  if ((*((volatile uint32_t *) 0x51018014)) != 0x01ed0efa) return CNN_FAIL; // 0,0,380-383
  if ((*((volatile uint32_t *) 0x50400018)) != 0x120f06f3) return CNN_FAIL; // 0,0,384-387
  if ((*((volatile uint32_t *) 0x50408018)) != 0xf800060f) return CNN_FAIL; // 0,0,388-391
  if ((*((volatile uint32_t *) 0x50410018)) != 0xec0d18ee) return CNN_FAIL; // 0,0,392-395
  if ((*((volatile uint32_t *) 0x50418018)) != 0xe4f01bf9) return CNN_FAIL; // 0,0,396-399
  if ((*((volatile uint32_t *) 0x50800018)) != 0xf503f020) return CNN_FAIL; // 0,0,400-403
  if ((*((volatile uint32_t *) 0x50808018)) != 0xf403eee0) return CNN_FAIL; // 0,0,404-407
  if ((*((volatile uint32_t *) 0x50810018)) != 0xf6fee9e0) return CNN_FAIL; // 0,0,408-411
  if ((*((volatile uint32_t *) 0x50818018)) != 0xf8fbf407) return CNN_FAIL; // 0,0,412-415
  if ((*((volatile uint32_t *) 0x50c00018)) != 0xf7050bf6) return CNN_FAIL; // 0,0,416-419
  if ((*((volatile uint32_t *) 0x50c08018)) != 0x03f01304) return CNN_FAIL; // 0,0,420-423
  if ((*((volatile uint32_t *) 0x50c10018)) != 0x0fedec0a) return CNN_FAIL; // 0,0,424-427
  if ((*((volatile uint32_t *) 0x50c18018)) != 0xf8edfc1d) return CNN_FAIL; // 0,0,428-431
  if ((*((volatile uint32_t *) 0x51000018)) != 0xfd040407) return CNN_FAIL; // 0,0,432-435
  if ((*((volatile uint32_t *) 0x51008018)) != 0xf7ef0d0d) return CNN_FAIL; // 0,0,436-439
  if ((*((volatile uint32_t *) 0x51010018)) != 0x00100bfd) return CNN_FAIL; // 0,0,440-443
  if ((*((volatile uint32_t *) 0x51018018)) != 0xe41c0cf7) return CNN_FAIL; // 0,0,444-447
  if ((*((volatile uint32_t *) 0x5040001c)) != 0xf8e00f14) return CNN_FAIL; // 0,0,448-451
  if ((*((volatile uint32_t *) 0x5040801c)) != 0x05edf806) return CNN_FAIL; // 0,0,452-455
  if ((*((volatile uint32_t *) 0x5041001c)) != 0x10fafdf9) return CNN_FAIL; // 0,0,456-459
  if ((*((volatile uint32_t *) 0x5041801c)) != 0xea09f001) return CNN_FAIL; // 0,0,460-463
  if ((*((volatile uint32_t *) 0x5080001c)) != 0xdc030cff) return CNN_FAIL; // 0,0,464-467
  if ((*((volatile uint32_t *) 0x5080801c)) != 0x1b220713) return CNN_FAIL; // 0,0,468-471
  if ((*((volatile uint32_t *) 0x5081001c)) != 0xf2edf1f4) return CNN_FAIL; // 0,0,472-475
  if ((*((volatile uint32_t *) 0x5081801c)) != 0x0709fcf7) return CNN_FAIL; // 0,0,476-479
  if ((*((volatile uint32_t *) 0x50c0001c)) != 0x1b10f0f3) return CNN_FAIL; // 0,0,480-483
  if ((*((volatile uint32_t *) 0x50c0801c)) != 0x1103fbfc) return CNN_FAIL; // 0,0,484-487
  if ((*((volatile uint32_t *) 0x50c1001c)) != 0x0901e6f8) return CNN_FAIL; // 0,0,488-491
  if ((*((volatile uint32_t *) 0x50c1801c)) != 0x1015fb02) return CNN_FAIL; // 0,0,492-495
  if ((*((volatile uint32_t *) 0x5100001c)) != 0x051afdf0) return CNN_FAIL; // 0,0,496-499
  if ((*((volatile uint32_t *) 0x5100801c)) != 0x1debe8ed) return CNN_FAIL; // 0,0,500-503
  if ((*((volatile uint32_t *) 0x5101001c)) != 0xf9171704) return CNN_FAIL; // 0,0,504-507
  if ((*((volatile uint32_t *) 0x5101801c)) != 0xe6160809) return CNN_FAIL; // 0,0,508-511

  return CNN_OK;
}

static int32_t ml_data32[(CNN_NUM_OUTPUTS + 3) / 4];

int main(void)
{
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

  printf("\n*** CNN Inference Test ***\n");

  cnn_init(); // Bring state machine into consistent state
  cnn_load_weights(); // Load kernels
  // cnn_load_bias(); // Not used in this network
  cnn_configure(); // Configure state machine
  cnn_start(); // Start CNN processing
  load_input(); // Load data input via FIFO

  while (cnn_time == 0)
    __WFI(); // Wait for CNN

  if (check_output() != CNN_OK) fail();
  cnn_unload((uint32_t *) ml_data32);

  printf("\n*** PASS ***\n\n");

#ifdef CNN_INFERENCE_TIMER
  printf("Approximate data loading and inference time: %d us\n\n", cnn_time);
#endif

  cnn_disable(); // Shut down CNN clock, disable peripheral


  return 0;
}

/*
  SUMMARY OF OPS
  Hardware: 56,295,040 ops (55,234,560 macc; 1,052,800 comp; 7,680 add; 0 mul; 0 bitwise)

  RESOURCE USAGE
  Weight memory: 176,048 bytes out of 442,368 bytes total (40%)
  Bias memory:   0 bytes out of 2,048 bytes total (0%)
*/

