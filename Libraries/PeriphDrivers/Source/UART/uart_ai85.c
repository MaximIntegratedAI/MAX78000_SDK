/* ****************************************************************************
 * Copyright (C) 2018 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include "uart.h"
#include "mxc_device.h"
#include "mxc_pins.h"
#include "mxc_assert.h"
#include "uart_revb.h"
#include "uart_common.h"
#include "lpgcr_regs.h"
#include "dma.h"

int MXC_UART_AsyncCallback(mxc_uart_regs_t* uart, int retVal)
{
    return MXC_UART_RevB_AsyncCallback(uart, retVal);
}

int MXC_UART_AsyncStop(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_AsyncStop(uart);
}

int MXC_UART_Init(mxc_uart_regs_t* uart, unsigned int baud, mxc_uart_clock_t clock)
{
    int retval;
    
    retval = MXC_UART_Shutdown(uart);
    
    if (retval) {
        return retval;
    }
    
    switch (clock) {
    case MXC_UART_32K_CLK:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_ERTCO);
        break;
        
    case MXC_UART_8M_CLK:
        MXC_SYS_ClockSourceEnable(MXC_SYS_CLOCK_IBRO);
        break;
        
    default:
        break;
    }
    
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_GPIO_Config(&gpio_cfg_uart0);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;
        
    case 1:
        MXC_GPIO_Config(&gpio_cfg_uart1);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;
        
    case 2:
        MXC_GPIO_Config(&gpio_cfg_uart2);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;
        
    case 3:
        MXC_GPIO_Config(&gpio_cfg_uart3);
        MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_UART3);
        break;
        
    default:
        return E_BAD_PARAM;
    }
    
    return MXC_UART_RevB_Init(uart, baud, clock);
}

int MXC_UART_Shutdown(mxc_uart_regs_t* uart)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART0);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART0);
        break;
        
    case 1:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART1);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART1);
        break;
        
    case 2:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET0_UART2);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART2);
        break;
        
    case 3:
        MXC_SYS_Reset_Periph(MXC_SYS_RESET_UART3);
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_UART3);
        break;
        
    default:
        return E_BAD_PARAM;
    }
    
    return E_NO_ERROR;
}

int MXC_UART_ReadyForSleep(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_ReadyForSleep(uart);
}

int MXC_UART_SetFrequency(mxc_uart_regs_t* uart, unsigned int baud, mxc_uart_clock_t clock)
{
    return MXC_UART_RevB_SetFrequency(uart, baud, clock);
}

int MXC_UART_GetFrequency(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetFrequency(uart);
}

int MXC_UART_SetDataSize(mxc_uart_regs_t* uart, int dataSize)
{
    return MXC_UART_RevB_SetDataSize(uart, dataSize);
}

int MXC_UART_SetStopBits(mxc_uart_regs_t* uart, mxc_uart_stop_t stopBits)
{
    return MXC_UART_RevB_SetStopBits(uart, stopBits);
}

int MXC_UART_SetParity(mxc_uart_regs_t* uart, mxc_uart_parity_t parity)
{
    return MXC_UART_RevB_SetParity(uart, parity);
}

int MXC_UART_SetFlowCtrl(mxc_uart_regs_t* uart, mxc_uart_flow_t flowCtrl, int rtsThreshold)
{
    switch (MXC_UART_GET_IDX(uart)) {
    case 0:
        MXC_GPIO_Config(&gpio_cfg_uart0_flow);
        break;
        
    case 1:
        MXC_GPIO_Config(&gpio_cfg_uart1_flow);
        break;
        
    case 2:
        MXC_GPIO_Config(&gpio_cfg_uart2_flow);
        break;
        
    }
    
    return MXC_UART_RevB_SetFlowCtrl(uart, flowCtrl, rtsThreshold);
    
}

int MXC_UART_SetClockSource(mxc_uart_regs_t* uart, mxc_uart_clock_t clock)
{
    return MXC_UART_RevB_SetClockSource(uart, clock);
}

int MXC_UART_GetActive(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetActive(uart);
}

int MXC_UART_AbortTransmission(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_AbortTransmission(uart);
}

int MXC_UART_ReadCharacterRaw(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_ReadCharacterRaw(uart);
}

int MXC_UART_WriteCharacterRaw(mxc_uart_regs_t* uart, uint8_t character)
{
    return MXC_UART_RevB_WriteCharacterRaw(uart, character);
}
int MXC_UART_ReadCharacter(mxc_uart_regs_t* uart)
{
    return MXC_UART_Common_ReadCharacter(uart);
}

int MXC_UART_WriteCharacter(mxc_uart_regs_t* uart, uint8_t character)
{
    return MXC_UART_Common_WriteCharacter(uart, character);
}

int MXC_UART_Read(mxc_uart_regs_t* uart, uint8_t* buffer, int* len)
{
    return MXC_UART_RevB_Read(uart, buffer, len);
}

int MXC_UART_Write(mxc_uart_regs_t* uart, uint8_t* byte, int* len)
{
    return MXC_UART_RevB_Write(uart, byte, len);
}

unsigned int MXC_UART_ReadRXFIFO(mxc_uart_regs_t* uart, unsigned char* bytes,
                                 unsigned int len)
{
    return MXC_UART_RevB_ReadRXFIFO(uart, bytes, len);
    
}

int MXC_UART_ReadRXFIFODMA(mxc_uart_regs_t* uart, unsigned char* bytes,
                           unsigned int len, mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;
    
    int uart_num = MXC_UART_GET_IDX(uart);
    
    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0RX;
        break;
        
    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1RX;
        break;
        
    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2RX;
        break;
        
    default:
        return E_BAD_PARAM;
        break;
    }
    
    return MXC_UART_RevB_ReadRXFIFODMA(uart, bytes, len, callback, config);
}

unsigned int MXC_UART_GetRXFIFOAvailable(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetRXFIFOAvailable(uart);
}

unsigned int MXC_UART_WriteTXFIFO(mxc_uart_regs_t* uart, unsigned char* bytes,
                                  unsigned int len)
{
    return MXC_UART_RevB_WriteTXFIFO(uart, bytes, len);
}

int MXC_UART_WriteTXFIFODMA(mxc_uart_regs_t* uart, unsigned char* bytes,
                            unsigned int len, mxc_uart_dma_complete_cb_t callback)
{
    mxc_dma_config_t config;
    
    int uart_num = MXC_UART_GET_IDX(uart);
    
    switch (uart_num) {
    case 0:
        config.reqsel = MXC_DMA_REQUEST_UART0TX;
        break;
        
    case 1:
        config.reqsel = MXC_DMA_REQUEST_UART1TX;
        break;
        
    case 2:
        config.reqsel = MXC_DMA_REQUEST_UART2TX;
        break;
        
    default:
        return E_BAD_PARAM;
        break;
    }
    
    return MXC_UART_RevB_WriteTXFIFODMA(uart, bytes, len, callback, config);
}

unsigned int MXC_UART_GetTXFIFOAvailable(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetTXFIFOAvailable(uart);
}

int MXC_UART_ClearRXFIFO(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_ClearRXFIFO(uart);
}

int MXC_UART_ClearTXFIFO(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_ClearTXFIFO(uart);
}

int MXC_UART_SetRXThreshold(mxc_uart_regs_t* uart, unsigned int numBytes)
{
    return MXC_UART_RevB_SetRXThreshold(uart, numBytes);
}

unsigned int MXC_UART_GetRXThreshold(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetRXThreshold(uart);
}

int MXC_UART_SetTXThreshold(mxc_uart_regs_t* uart, unsigned int numBytes)
{
    return E_NOT_SUPPORTED;
}

unsigned int MXC_UART_GetTXThreshold(mxc_uart_regs_t* uart)
{
    return E_NOT_SUPPORTED;
}

unsigned int MXC_UART_GetFlags(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetFlags(uart);
}

int MXC_UART_ClearFlags(mxc_uart_regs_t* uart, unsigned int flags)
{
    return MXC_UART_RevB_ClearFlags(uart, flags);
}

int MXC_UART_EnableInt(mxc_uart_regs_t* uart, unsigned int intEn)
{
    return MXC_UART_RevB_EnableInt(uart, intEn);
}

int MXC_UART_DisableInt(mxc_uart_regs_t* uart, unsigned int intDis)
{
    return MXC_UART_RevB_DisableInt(uart, intDis);
}

unsigned int MXC_UART_GetStatus(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_GetStatus(uart);
}

int MXC_UART_Transaction(mxc_uart_req_t* req)
{
    return MXC_UART_RevB_Transaction(req);
}

int MXC_UART_TransactionAsync(mxc_uart_req_t* req)
{
    return MXC_UART_RevB_TransactionAsync(req);
}

int MXC_UART_TransactionDMA(mxc_uart_req_t* req)
{
    return MXC_UART_RevB_TransactionDMA(req);
}

int MXC_UART_AbortAsync(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_AbortAsync(uart);
}

int MXC_UART_AsyncHandler(mxc_uart_regs_t* uart)
{
    return MXC_UART_RevB_AsyncHandler(uart);
}

int MXC_UART_DMACallback(int ch, int error)
{
    return MXC_UART_RevB_DMACallback(ch, error);
}
