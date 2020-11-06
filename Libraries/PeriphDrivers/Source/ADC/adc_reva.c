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
#include "dma.h"
#include "adc_reva.h"
#include "adc_regs.h"

#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include <stdio.h>


#define MXC_CONVERSION_INTERRUPT        0x1
#define MXC_CONVERSION_REQ_INTERRUPT    0X2
#define MXC_MONITOR_INTERRUPT           0X4
#define MXC_CONVERSION_SPEED_6MHZ       5847
// Mask for all Interrupt Enable Fields
#define ADC_IE_MASK (MXC_F_ADC_INTR_DONE_IE     | MXC_F_ADC_INTR_REF_READY_IE | \
                     MXC_F_ADC_INTR_HI_LIMIT_IE | MXC_F_ADC_INTR_LO_LIMIT_IE  | \
                     MXC_F_ADC_INTR_OVERFLOW_IE )

#define ADC_IF_MASK (MXC_F_ADC_INTR_DONE_IF     | MXC_F_ADC_INTR_REF_READY_IF | \
                     MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF  | \
                     MXC_F_ADC_INTR_OVERFLOW_IF )

#define MXC_MONITOR_NUM 4

static mxc_adc_monitor_req_t* states[MXC_MONITOR_NUM];
static mxc_adc_complete_cb_t async_callback;
static mxc_adc_conversion_req_t* async_req;
static volatile uint8_t flag;      //indicates  to irqhandler where to store data

int MXC_ADC_RevA_Init(void)
{
    //set adc frequency to work at approximately 6 MHZ
    //NOTE: cannot use RevA version because MCR registers must be set
    MXC_ADC_SetConversionSpeed(MXC_CONVERSION_SPEED_6MHZ);
    //clear adc reference ready interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_REF_READY_IF);
    //turn on adc
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_PWR;
    //turn on reference buffer power
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_REFBUF_PWR;
    
    //wait until hardware set adc_intr.ref_ready_if to 1
    while (!(MXC_ADC->intr & MXC_F_ADC_INTR_REF_READY_IF));
    
    //clear adc reference ready interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_REF_READY_IF);
    
    //set lock variables initially to NULL
    async_callback = NULL;
    
    for (uint8_t i = 0; i < MXC_MONITOR_NUM ; i++) {
        states[i] = NULL;
    }
    
    async_req = NULL;
    
    return E_NO_ERROR;
}

int MXC_ADC_RevA_Shutdown(void)
{
    // Disable ADC Power
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_PWR;
    // Disable Reference Buffer Power
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REFBUF_PWR;
    // Disable ADC Clock
    MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_CLK_EN;
    
    if (async_callback != NULL) {
        MXC_FreeLock((uint32_t*) &async_callback);
    }
    
    for (uint8_t i = 0; i < MXC_MONITOR_NUM ; i++) {
        if (states[i] != NULL) {
            MXC_FreeLock((uint32_t*) &states[i]);
        }
    }
    
    if (async_req != NULL) {
        MXC_FreeLock((uint32_t*) &async_req);
    }
    
    return E_NO_ERROR;
}

int MXC_ADC_RevA_Busy(void)
{
    return (MXC_ADC->status & MXC_F_ADC_STATUS_AFE_PWR_UP_ACTIVE) >> MXC_F_ADC_STATUS_AFE_PWR_UP_ACTIVE_POS;
}

void MXC_ADC_RevA_EnableInt(uint32_t flags)
{
    MXC_ADC->intr = ((MXC_ADC->intr | flags) & ADC_IE_MASK);
}

void MXC_ADC_RevA_DisableInt(uint32_t flags)
{
    MXC_ADC->intr &= ~(flags & ADC_IE_MASK);
}

int MXC_ADC_RevA_GetFlags(void)
{
    return (MXC_ADC->intr & ADC_IF_MASK);
}

void MXC_ADC_RevA_ClearFlags(uint32_t flags)
{
    // Write 1 to clear flags
    MXC_ADC->intr |= (flags & ADC_IF_MASK);
}

int MXC_ADC_RevA_SetConversionSpeed(uint32_t hz)
{
    //enable clock
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_CLK_EN;
    return E_NO_ERROR;
}

int MXC_ADC_RevA_GetConversionSpeed(uint8_t divider)
{
    uint32_t adc_clock_freq = PeripheralClock / divider;
    return adc_clock_freq / 1024;
}

void MXC_ADC_RevA_SetDataAlignment(int msbJustify)
{
    if (msbJustify) {
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_DATA_ALIGN;
    }
    else {
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_DATA_ALIGN;
    }
}

void MXC_ADC_RevA_SetExtScale(mxc_adc_scale_t scale)
{
    //clear adc division bits
    MXC_ADC->ctrl &= ~(MXC_S_ADC_CTRL_ADC_DIVSEL_DIV4);
    
    switch (scale) {
    case MXC_ADC_SCALE_2X: //.05 ,ref scale 1, adc_divsel 0, input scale 0
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_SCALE;
        break;
        
    case MXC_ADC_SCALE_1:
        //input scale = 0, adc_divsel = 0, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_SCALE;
        break;
        
    case MXC_ADC_SCALE_2:
        //input scale = 1, adc_divsel = 0, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_SCALE;
        break;
        
    case MXC_ADC_SCALE_3:
        //input scale = 0, adc_divsel = 2, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_SCALE;
        MXC_ADC->ctrl |= MXC_V_ADC_CTRL_ADC_DIVSEL_DIV3;
        break;
        
    case MXC_ADC_SCALE_4:
        //input scale = 1, adc_divsel = 0x1, ref scale = 0,
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_SCALE;
        MXC_ADC->ctrl |= MXC_S_ADC_CTRL_ADC_DIVSEL_DIV2;
        break;
        
    case MXC_ADC_SCALE_6:
        //input scale = 1, adc_divsel = 0x2, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_SCALE;
        MXC_ADC->ctrl |= MXC_S_ADC_CTRL_ADC_DIVSEL_DIV3;
        break;
        
    case MXC_ADC_SCALE_8:
        //input scale = 1, adc_divsel = 0x3, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl |= MXC_F_ADC_CTRL_SCALE;
        MXC_ADC->ctrl |= MXC_S_ADC_CTRL_ADC_DIVSEL_DIV4;
        break;
        
    default:
        //input scale = 0, adc_divsel = 0, ref scale = 0
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_REF_SCALE;
        MXC_ADC->ctrl &= ~MXC_F_ADC_CTRL_SCALE;
        break;
    }
}

void MXC_ADC_RevA_EnableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    MXC_ADC->limit[monitor] |= (MXC_F_ADC_LIMIT_CH_HI_LIMIT_EN | MXC_F_ADC_LIMIT_CH_LO_LIMIT_EN);
    
}

void MXC_ADC_RevA_DisableMonitor(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    MXC_ADC->limit[monitor] &= ~(MXC_F_ADC_LIMIT_CH_HI_LIMIT_EN | MXC_F_ADC_LIMIT_CH_LO_LIMIT_EN);
}

void MXC_ADC_RevA_SetMonitorHighThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    //clear current high threshold
    MXC_ADC->limit[monitor] &= ~MXC_F_ADC_LIMIT_CH_HI_LIMIT;
    //set new high threshold
    MXC_ADC->limit[monitor] |= (threshold << MXC_F_ADC_LIMIT_CH_HI_LIMIT_POS) & MXC_F_ADC_LIMIT_CH_HI_LIMIT;
}

int MXC_ADC_RevA_GetMonitorHighThreshold(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    return (MXC_ADC->limit[monitor] & MXC_F_ADC_LIMIT_CH_HI_LIMIT) >> MXC_F_ADC_LIMIT_CH_HI_LIMIT_POS;
}

void MXC_ADC_RevA_SetMonitorLowThreshold(mxc_adc_monitor_t monitor, uint32_t threshold)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    MXC_ADC->limit[monitor] &=  ~MXC_F_ADC_LIMIT_CH_LO_LIMIT;
    MXC_ADC->limit[monitor] |= (threshold << MXC_F_ADC_LIMIT_CH_LO_LIMIT_POS) & MXC_F_ADC_LIMIT_CH_LO_LIMIT;
}

int MXC_ADC_RevA_GetMonitorLowThreshold(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    return (MXC_ADC->limit[monitor] & MXC_F_ADC_LIMIT_CH_LO_LIMIT) >> MXC_F_ADC_LIMIT_CH_LO_LIMIT_POS;
}

void MXC_ADC_RevA_SetMonitorChannel(mxc_adc_monitor_t monitor, mxc_adc_chsel_t channel)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    MXC_ADC->limit[monitor] &= ~MXC_F_ADC_LIMIT_CH_SEL;
    MXC_ADC->limit[monitor] |= (channel << MXC_F_ADC_LIMIT_CH_SEL_POS) & MXC_F_ADC_LIMIT_CH_SEL;
}

int MXC_ADC_RevA_GetMonitorChannel(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    return (MXC_ADC->limit[monitor] & MXC_F_ADC_LIMIT_CH_SEL) >> MXC_F_ADC_LIMIT_CH_SEL_POS;
}

void MXC_ADC_RevA_EnableMonitorAsync(mxc_adc_monitor_t monitor, mxc_adc_monitor_cb_t callback)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM && callback != NULL);
    states[monitor]->callback = callback;
}

void MXC_ADC_RevA_DisableMonitorAsync(mxc_adc_monitor_t monitor)
{
    MXC_ASSERT(monitor < MXC_MONITOR_NUM);
    states[monitor]->callback = NULL;
}

int MXC_ADC_RevA_StartConversion(mxc_adc_chsel_t channel)
{
    uint16_t data;
    int error;
    
    if (channel > MXC_ADC_CH_VREGI) {
        return E_BAD_PARAM;
    }
    
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    //clear ADC done interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
    
    //set start bit
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_START;
    
    if ((error = MXC_ADC_GetData(&data)) != E_NO_ERROR) {
        return error;
    }
    
    return E_NO_ERROR;
}

int MXC_ADC_RevA_StartConversionAsync(mxc_adc_chsel_t channel, mxc_adc_complete_cb_t callback)
{
    //check satus call get lock
    
    if (channel > MXC_ADC_CH_VREGI || callback == NULL) {
        return E_BAD_PARAM;
    }
    
    //check lock
    while (MXC_GetLock((uint32_t*) &async_callback, (uint32_t) callback) != E_NO_ERROR);
    
    flag |= MXC_CONVERSION_INTERRUPT;
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    //clear ADC done interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
    
    //set start bit
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_START;
    
    //enable adc done interrupt
    MXC_ADC_RevA_EnableInt(MXC_F_ADC_INTR_DONE_IE);
    
    return  E_NO_ERROR;
}

int MXC_ADC_RevA_StartConversionDMA(mxc_adc_chsel_t channel, uint16_t* data, void (*callback)(int, int))
{
    uint8_t channelDMA;
    mxc_dma_config_t config;
    mxc_dma_srcdst_t srcdst;
    
    if (channel > MXC_ADC_CH_VREGI) {
        return E_BAD_PARAM;
    }
    
    if (data == NULL) {
        return E_BAD_PARAM;
    }
    
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    //clear ADC done interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
    
    MXC_DMA_Init();
    
    channelDMA = MXC_DMA_AcquireChannel();
    
    config.reqsel = MXC_S_DMA_CTRL_REQUEST_MEMTOMEM;
    config.ch = channelDMA;
    
    config.srcwd = MXC_DMA_WIDTH_HALFWORD;
    config.dstwd = MXC_DMA_WIDTH_HALFWORD;
    
    config.srcinc_en = 0;
    config.dstinc_en = 0;
    
    srcdst.ch       = channelDMA;
    srcdst.source   = (uint16_t*) & (MXC_ADC->data);
    srcdst.dest     = data;
    srcdst.len      = 2;
    
    MXC_DMA_ConfigChannel(config, srcdst);
    
    MXC_DMA_SetCallback(channelDMA, callback);
    
    MXC_DMA_EnableInt(channelDMA);
    MXC_DMA->ch[channelDMA].ctrl |= 2 << MXC_F_DMA_CTRL_BURST_SIZE_POS;
    
    //set start bit
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_START;
    MXC_DMA_Start(channelDMA);
    MXC_DMA->ch[channelDMA].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;
    
    return E_NO_ERROR;
}

int MXC_ADC_RevA_Handler(void)
{
    uint16_t data;
    int error, i;
    
    if ((error = MXC_ADC_GetData(&data)) != E_NO_ERROR) {
        return error;
    }
    
    if (flag & MXC_CONVERSION_INTERRUPT) {
        mxc_adc_complete_cb_t cb = async_callback;
        MXC_FreeLock((uint32_t*) &async_callback);
        MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
        flag &= ~MXC_CONVERSION_INTERRUPT;
        MXC_ADC_RevA_DisableInt(MXC_F_ADC_INTR_DONE_IE);
        //read data
        (cb)(NULL, data);
    }
    
    if (flag & MXC_CONVERSION_REQ_INTERRUPT) {
        mxc_adc_conversion_req_t*   temp = async_req;
        MXC_FreeLock((uint32_t*) &async_req);
        MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
        flag &= ~MXC_CONVERSION_INTERRUPT;
        MXC_ADC_RevA_DisableInt(MXC_F_ADC_INTR_DONE_IE);
        //read data
        temp->rawADCValue = data;
        (temp->callback)(temp, E_NO_ERROR);
        
    }
    
    if ((MXC_ADC_RevA_GetFlags() & (MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF)) && flag & MXC_MONITOR_INTERRUPT) {
        MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF);
        
        for (i = 0; i < MXC_MONITOR_NUM; i++) {
            if (states[i]->callback != NULL) {
                mxc_adc_monitor_req_t* temp = states[i];
                MXC_FreeLock((uint32_t*) &states[i]);
                temp->callback(temp, E_NO_ERROR);
            }
        }
        
        flag &= ~MXC_MONITOR_INTERRUPT;
    }
    
    return E_NO_ERROR;
}

int  MXC_ADC_RevA_Convert(mxc_adc_conversion_req_t* req)
{
    uint16_t data;
    int error;
    
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (req->channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    if (req->channel <= MXC_V_ADC_CTRL_ADC_CHSEL_AIN7) {
        MXC_ADC_RevA_SetExtScale(req->scale);
    }
    
    //clear ADC done interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
    
    //set start bit
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_START;
    
    if ((error = MXC_ADC_GetData(&data)) != E_NO_ERROR) {
        return error;
    }
    
    req->rawADCValue = data;
    
    return E_NO_ERROR;
}

int MXC_ADC_RevA_ConvertAsync(mxc_adc_conversion_req_t req)
{
    //check lock
    if (MXC_GetLock((uint32_t*) &async_req, (uint32_t) &req) != E_NO_ERROR) {
        return E_BUSY;
    }
    
    //save callback function to be called from isr
    flag |= MXC_CONVERSION_REQ_INTERRUPT;
    async_callback = req.callback;
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (req.channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    if (req.channel <= MXC_V_ADC_CTRL_ADC_CHSEL_AIN7) {
        MXC_ADC_RevA_SetExtScale(req.scale);
    }
    
    //clear ADC done interrupt flag
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_DONE_IF);
    
    //enable adc done interrupt
    MXC_ADC_RevA_EnableInt(MXC_F_ADC_INTR_DONE_IE);
    
    //set start bit
    MXC_ADC->ctrl |= MXC_F_ADC_CTRL_START;
    
    return E_NO_ERROR;
}

void MXC_ADC_RevA_Monitor(mxc_adc_monitor_req_t req)
{
    MXC_ASSERT(req.monitor < MXC_MONITOR_NUM);
    
    while (MXC_ADC->status & MXC_F_ADC_STATUS_ACTIVE);
    
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (req.channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    MXC_ADC_RevA_SetMonitorChannel(req.monitor, req.channel);
    
    MXC_ADC_RevA_SetMonitorHighThreshold(req.monitor, req.highThreshold);
    MXC_ADC_RevA_SetMonitorLowThreshold(req.monitor, req.lowThreshold);
    
    if (req.channel <= MXC_ADC_CH_7) {
        MXC_ADC_RevA_SetExtScale(req.scale);
    }
    
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF);
    MXC_ADC_RevA_EnableMonitor(req.monitor);
    
    while (!(MXC_ADC->intr & (MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF)));
}

void MXC_ADC_RevA_MonitorAsync(mxc_adc_monitor_req_t req)
{
    MXC_ASSERT(req.monitor < MXC_MONITOR_NUM);
    
    //check lock
    while (MXC_GetLock((uint32_t*) &states[req.monitor], (uint32_t) &req) != E_NO_ERROR);
    
    while (MXC_ADC->status & MXC_F_ADC_STATUS_ACTIVE);
    
    //clear selction bits
    MXC_ADC->ctrl &= ~(MXC_F_ADC_CTRL_CH_SEL);
    //set selction its to next channel to convert
    MXC_ADC->ctrl |= (req.channel << MXC_F_ADC_CTRL_CH_SEL_POS) & MXC_F_ADC_CTRL_CH_SEL;
    
    MXC_ADC_RevA_SetMonitorChannel(req.monitor, req.channel);
    
    MXC_ADC_RevA_SetMonitorHighThreshold(req.monitor, req.highThreshold);
    MXC_ADC_RevA_SetMonitorLowThreshold(req.monitor, req.lowThreshold);
    
    if (req.channel <= MXC_ADC_CH_7) {
        MXC_ADC_RevA_SetExtScale(req.scale);
    }
    
    MXC_ADC_RevA_ClearFlags(MXC_F_ADC_INTR_HI_LIMIT_IF | MXC_F_ADC_INTR_LO_LIMIT_IF);
    
    MXC_ASSERT(req.callback != NULL);
    MXC_ADC_RevA_EnableMonitorAsync(req.monitor, req.callback);
    flag |= MXC_MONITOR_INTERRUPT;
    MXC_ADC_RevA_EnableInt(MXC_F_ADC_INTR_HI_LIMIT_IE | MXC_F_ADC_INTR_LO_LIMIT_IE);
    MXC_ADC_RevA_EnableMonitor(req.monitor);
}

// ************************************* Function to Read ADC Data *******************************************
int MXC_ADC_RevA_GetData(uint16_t* outdata)
{
    // See if a conversion is in process
    if (MXC_ADC->status & MXC_F_ADC_STATUS_ACTIVE) {
        // Wait for conversion to complete
        while ((MXC_ADC->intr & MXC_F_ADC_INTR_DONE_IF) == 0);
    }
    
    // Read 32-bit value and truncate to 16-bit for output depending on data align bit
    if ((MXC_ADC->ctrl & MXC_F_ADC_CTRL_DATA_ALIGN) == 0) {
        *outdata = (uint16_t)(MXC_ADC->data);    /* LSB justified */
    }
    else {
        *outdata = (uint16_t)(MXC_ADC->data >> 6);    /* MSB justified */
    }
    
    // Check for overflow
    if (MXC_ADC->status & MXC_F_ADC_STATUS_OVERFLOW) {
        return E_OVERFLOW;
    }
    
    return E_NO_ERROR;
}
