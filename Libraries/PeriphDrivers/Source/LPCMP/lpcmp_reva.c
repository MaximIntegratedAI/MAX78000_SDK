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

#include "lpcmp.h"
#include "dma.h"
#include "lpcmp_reva.h"
#include "lpcmp_regs.h"
#include "pwrseq_regs.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_delay.h"
#include "mxc_sys.h"
#include "mcr_regs.h"
#include "mxc_lock.h"
#include <stdio.h>

int MXC_LPCMP_RevA_Init(void)
{
    MXC_LPCMP0->ctrl[0] |= MXC_F_LPCMP_CTRL_CMPEN;
    return E_NO_ERROR;
}

int MXC_LPCMP_RevA_Shutdown(void)
{
    MXC_LPCMP0->ctrl[0] &= ~MXC_F_LPCMP_CTRL_CMPEN;
    return E_NO_ERROR;    
}

void MXC_LPCMP_RevA_EnableInt(void)
{
    MXC_LPCMP0->ctrl[0] |= MXC_F_LPCMP_CTRL_IRQEN;
}

void MXC_LPCMP_RevA_DisableInt(void)
{
    MXC_LPCMP0->ctrl[0] &= ~MXC_F_LPCMP_CTRL_IRQEN;
}

void MXC_LPCMP_RevA_EnableWakeup(void)
{
    MXC_PWRSEQ->lppwen |= MXC_F_PWRSEQ_LPPWEN_AINCOMP0;
}

void MXC_LPCMP_RevA_DisableWakeup(void)
{
    MXC_PWRSEQ->lppwen &= ~MXC_F_PWRSEQ_LPPWEN_AINCOMP0;   
}

void MXC_LPCMP_RevA_SelectPolarity(mxc_lpcmp_polarity_t polarity)
{
    if(polarity==MXC_LPCMP_POLLOW) {
        MXC_LPCMP0->ctrl[0] &= ~(polarity<<MXC_F_LPCMP_CTRL_POLSEL_POS);    
    }
    else {
        MXC_LPCMP0->ctrl[0] |= (polarity<<MXC_F_LPCMP_CTRL_POLSEL_POS);
    }
}
