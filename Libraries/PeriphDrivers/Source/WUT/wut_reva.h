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
 **************************************************************************** */

/* **** Includes **** */
#include "mxc_device.h"
#include "mxc_assert.h"
#include "mxc_sys.h"
#include "wut.h"
#include "gcr_regs.h"

/* **** Functions **** */
void MXC_WUT_RevA_Init(mxc_wut_regs_t* wut, mxc_wut_pres_t pres);

void MXC_WUT_RevA_Shutdown(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_Enable(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_Disable(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_Config(mxc_wut_regs_t* wut, const mxc_wut_cfg_t *cfg);

uint32_t MXC_WUT_RevA_GetCompare(mxc_wut_regs_t* wut);

uint32_t MXC_WUT_RevA_GetCapture(mxc_wut_regs_t* wut);

uint32_t MXC_WUT_RevA_GetCount(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_IntClear(mxc_wut_regs_t* wut);

uint32_t MXC_WUT_RevA_IntStatus(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_SetCompare(mxc_wut_regs_t* wut, uint32_t cmp_cnt);

void MXC_WUT_RevA_SetCount(mxc_wut_regs_t* wut, uint32_t cnt);

int MXC_WUT_RevA_GetTicks(mxc_wut_regs_t* wut, uint32_t timerClock, uint32_t time, mxc_wut_unit_t units, uint32_t *ticks);

int MXC_WUT_RevA_GetTime(mxc_wut_regs_t* wut, uint32_t timerClock, uint32_t ticks, uint32_t *time, mxc_wut_unit_t *units);

void MXC_WUT_RevA_Edge(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_Store(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_RestoreBBClock(mxc_wut_regs_t* wut, uint32_t dbbFreq, uint32_t timerClock);

uint32_t MXC_WUT_RevA_GetSleepTicks(mxc_wut_regs_t* wut);

void MXC_WUT_RevA_Delay_MS(mxc_wut_regs_t* wut, uint32_t waitMs, uint32_t timerClock);