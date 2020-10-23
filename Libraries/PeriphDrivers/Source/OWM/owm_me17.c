/* *****************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
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
 * $Date: 2018-08-28 17:03:02 -0500 (Tue, 28 Aug 2018) $
 * $Revision: 37424 $
 *
 **************************************************************************** */

/* **** Includes **** */
#include <string.h>
#include "owm_reva.h"

/* **** Definitions **** */
#define MXC_OWM_CLK_FREQ    1000000 //1-Wire requires 1MHz clock

/* **** Globals **** */

/* **** Functions **** */

int MXC_OWM_Init(const mxc_owm_cfg_t* cfg)
{
    int err = 0;
    uint32_t mxc_owm_clk, clk_div = 0;
    
    if (cfg == NULL) {
        return E_NULL_PTR;
    }
    
    // Set system level configurations
    mxc_gpio_regs_t* gpio = gpio_cfg_owm.port;
    
    MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_OWIRE);
    gpio->vssel |= gpio_cfg_owm.mask;         // 1-Wire pins need to be at 3.3V.
    
    if ((err = MXC_GPIO_Config(&gpio_cfg_owm)) != E_NO_ERROR) {
        return err;
    }
    
    // Configure clk divisor to get 1MHz OWM clk
    mxc_owm_clk = PeripheralClock;
    
    if (mxc_owm_clk == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_UNINITIALIZED;
    }
    
    // Return error if clk doesn't divide evenly to 1MHz
    if (mxc_owm_clk % MXC_OWM_CLK_FREQ) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_NOT_SUPPORTED;
    }
    
    clk_div = (mxc_owm_clk / (MXC_OWM_CLK_FREQ));
    
    // Can not support lower frequencies
    if (clk_div == 0) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
        return E_NOT_SUPPORTED;
    }
    
    err = MXC_OWM_RevA_Init(cfg);
    if(err == E_BAD_PARAM) {
        MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
    }
    
    return err;
}

void MXC_OWM_Shutdown(void)
{
    MXC_OWM_RevA_Shutdown();
    MXC_SYS_ClockDisable(MXC_SYS_PERIPH_CLOCK_OWIRE);
}

int MXC_OWM_Reset(void)
{
    return MXC_OWM_RevA_Reset();
}

int MXC_OWM_TouchByte(uint8_t data)
{
    return MXC_OWM_RevA_TouchByte(data);
}

int MXC_OWM_WriteByte(uint8_t data)
{
    return MXC_OWM_RevA_WriteByte(data);
}

int MXC_OWM_ReadByte(void)
{
    return MXC_OWM_RevA_ReadByte();
}

int MXC_OWM_TouchBit(uint8_t bit)
{
    return MXC_OWM_RevA_TouchBit(bit);
}

int MXC_OWM_WriteBit(uint8_t bit)
{
    return MXC_OWM_RevA_WriteBit(bit);
}

int MXC_OWM_ReadBit(void)
{
    return MXC_OWM_RevA_ReadBit();
}

int MXC_OWM_Write(uint8_t* data, int len)
{
    return MXC_OWM_RevA_Write(data, len);
}

int MXC_OWM_Read(uint8_t* data, int len)
{
    return MXC_OWM_RevA_Read(data, len);
}

int MXC_OWM_ReadROM(uint8_t* ROMCode)
{
    return MXC_OWM_RevA_ReadROM(ROMCode);
}

int MXC_OWM_MatchROM(uint8_t* ROMCode)
{
    return MXC_OWM_RevA_MatchROM(ROMCode);
}

int MXC_OWM_ODMatchROM(uint8_t* ROMCode)
{
    return MXC_OWM_RevA_ODMatchROM(ROMCode);
}

int MXC_OWM_SkipROM(void)
{
    return MXC_OWM_RevA_SkipROM();
}

int MXC_OWM_ODSkipROM(void)
{
    return MXC_OWM_RevA_ODSkipROM();
}

int MXC_OWM_Resume(void)
{
    return MXC_OWM_RevA_Resume();
}

int MXC_OWM_SearchROM(int newSearch, uint8_t* ROMCode)
{
    return MXC_OWM_RevA_SearchROM(newSearch, ROMCode);
}

void MXC_OWM_ClearFlags(uint32_t mask)
{
    MXC_OWM_RevA_ClearFlags(mask);
}

unsigned MXC_OWM_GetFlags(void)
{
    return MXC_OWM_RevA_GetFlags();
}

void MXC_OWM_SetExtPullup(int enable)
{
    MXC_OWM_RevA_SetExtPullup(enable);
}

void MXC_OWM_SetOverdrive(int enable)
{
    MXC_OWM_RevA_SetOverdrive(enable);
}

void MXC_OWM_EnableInt(int flags)
{
    MXC_OWM_RevA_EnableInt(flags);
}

void MXC_OWM_DisableInt(int flags)
{
    MXC_OWM_RevA_DisableInt(flags);
}

int MXC_OWM_SetForcePresenceDetect(int enable)
{
    return MXC_OWM_RevA_SetForcePresenceDetect(enable);
}

int MXC_OWM_SetInternalPullup(int enable)
{
    return MXC_OWM_RevA_SetInternalPullup(enable);
}

int MXC_OWM_SetExternalPullup(mxc_owm_ext_pu_t ext_pu_mode)
{
    return MXC_OWM_RevA_SetExternalPullup(ext_pu_mode);
}

int MXC_OWM_SystemClockUpdated(void)
{
    return MXC_OWM_RevA_SystemClockUpdated();
}

int MXC_OWM_SetSearchROMAccelerator(int enable)
{
    return MXC_OWM_RevA_SetSearchROMAccelerator(enable);
}

int MXC_OWM_BitBang_Init(int initialState)
{
    return MXC_OWM_RevA_BitBang_Init(initialState);
}

int MXC_OWM_BitBang_Read(void)
{
    return MXC_OWM_RevA_BitBang_Read();
}

int MXC_OWM_BitBang_Write(int state)
{
    return MXC_OWM_RevA_BitBang_Write(state);
}

int MXC_OWM_BitBang_Disable(void)
{
    return MXC_OWM_RevA_BitBang_Disable();
}

