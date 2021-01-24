/* ****************************************************************************
 * Copyright (C) 2019 Maxim Integrated Products, Inc., All Rights Reserved.
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
 *************************************************************************** */

#include "adc.h"
#include "adc_reva_regs.h"
#include "dma_reva_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mxc_lock.h"
#include <stdio.h>

int MXC_ADC_RevA_Init (mxc_adc_reva_regs_t* adc);
int MXC_ADC_RevA_Shutdown (mxc_adc_reva_regs_t* adc);
int MXC_ADC_RevA_Busy (mxc_adc_reva_regs_t* adc);
void MXC_ADC_RevA_EnableInt (mxc_adc_reva_regs_t* adc, uint32_t flags);
void MXC_ADC_RevA_DisableInt (mxc_adc_reva_regs_t* adc, uint32_t flags);
int MXC_ADC_RevA_GetFlags (mxc_adc_reva_regs_t* adc);
void MXC_ADC_RevA_ClearFlags (mxc_adc_reva_regs_t* adc, uint32_t flags);
int MXC_ADC_RevA_SetConversionSpeed (mxc_adc_reva_regs_t* adc, uint32_t hz);
int MXC_ADC_RevA_GetConversionSpeed (uint8_t divider);
void MXC_ADC_RevA_SetDataAlignment (mxc_adc_reva_regs_t* adc, int msbJustify);
void MXC_ADC_RevA_SetExtScale (mxc_adc_reva_regs_t* adc, mxc_adc_scale_t scale);
void MXC_ADC_RevA_EnableMonitor (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_DisableMonitor (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorHighThreshold (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor, uint32_t threshold);
int MXC_ADC_RevA_GetMonitorHighThreshold (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorLowThreshold (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor, uint32_t threshold);
int MXC_ADC_RevA_GetMonitorLowThreshold (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_SetMonitorChannel (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel);
int MXC_ADC_RevA_GetMonitorChannel (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_t monitor);
void MXC_ADC_RevA_EnableMonitorAsync (mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback);
void MXC_ADC_RevA_DisableMonitorAsync (mxc_adc_monitor_t monitor);
int MXC_ADC_RevA_StartConversion (mxc_adc_reva_regs_t* adc, mxc_adc_chsel_t channel);
int MXC_ADC_RevA_StartConversionAsync (mxc_adc_reva_regs_t* adc, mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback);
int MXC_ADC_RevA_StartConversionDMA (mxc_adc_reva_regs_t* adc, mxc_adc_chsel_t channel, uint16_t *data, void (*callback) (int, int));
int MXC_ADC_RevA_Handler (mxc_adc_reva_regs_t* adc);
int MXC_ADC_RevA_Convert (mxc_adc_reva_regs_t* adc, mxc_adc_conversion_req_t* req);
int MXC_ADC_RevA_ConvertAsync (mxc_adc_reva_regs_t* adc, mxc_adc_conversion_req_t req);
void MXC_ADC_RevA_Monitor (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_req_t req);
void MXC_ADC_RevA_MonitorAsync (mxc_adc_reva_regs_t* adc, mxc_adc_monitor_req_t req);
int MXC_ADC_RevA_GetData (mxc_adc_reva_regs_t* adc, uint16_t *outdata);
