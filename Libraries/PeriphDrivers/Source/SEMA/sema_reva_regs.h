/**
 * @file    sema_reva_regs.h
 * @brief   Registers, Bit Masks and Bit Positions for the SEMA_REVA Peripheral Module.
 */

/* ****************************************************************************
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
 *
 *************************************************************************** */

#ifndef _SEMA_REVA_REGS_H_
#define _SEMA_REVA_REGS_H_

/* **** Includes **** */
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
 
#if defined (__ICCARM__)
  #pragma system_include
#endif
 
#if defined (__CC_ARM)
  #pragma anon_unions
#endif
/// @cond
/*
    If types are not defined elsewhere (CMSIS) define them here
*/
#ifndef __IO
#define __IO volatile
#endif
#ifndef __I
#define __I  volatile const
#endif
#ifndef __O
#define __O  volatile
#endif
#ifndef __R
#define __R  volatile const
#endif
/// @endcond

/* **** Definitions **** */

/**
 * @ingroup     sema_reva
 * @defgroup    sema_reva_registers SEMA_REVA_Registers
 * @brief       Registers, Bit Masks and Bit Positions for the SEMA_REVA Peripheral Module.
 * @details The Semaphore peripheral allows multiple cores in a system to cooperate when accessing shred resources.
                                     The peripheral contains eight semaphores that can be atomically set and cleared. It is left to the discretion of the software
                                     architect to decide how and when the semaphores are used and how they are allocated. Existing hardware does not have to be
                                     
                                     modified for this type of cooperative sharing, and the use of semaphores is exclusively within the software domain.
 */

/**
 * @ingroup sema_reva_registers
 * Structure type to access the SEMA_REVA Registers.
 */
typedef struct {
    __IO uint32_t semaphores[8];        /**< <tt>\b 0x00:</tt> SEMA_REVA SEMAPHORES Register */
    __R  uint32_t rsv_0x20_0xff[56];
    __IO uint32_t status;               /**< <tt>\b 0x100:</tt> SEMA_REVA STATUS Register */
} mxc_sema_reva_regs_t;

/* Register offsets for module SEMA_REVA */
/**
 * @ingroup    sema_reva_registers
 * @defgroup   SEMA_REVA_Register_Offsets Register Offsets
 * @brief      SEMA_REVA Peripheral Register Offsets from the SEMA_REVA Base Peripheral Address. 
 * @{
 */
 #define MXC_R_SEMA_REVA_SEMAPHORES         ((uint32_t)0x00000000UL) /**< Offset from SEMA_REVA Base Address: <tt> 0x0000</tt> */ 
 #define MXC_R_SEMA_REVA_STATUS             ((uint32_t)0x00000100UL) /**< Offset from SEMA_REVA Base Address: <tt> 0x0100</tt> */ 
/**@} end of group sema_reva_registers */

/**
 * @ingroup  sema_reva_registers
 * @defgroup SEMA_REVA_SEMAPHORES SEMA_REVA_SEMAPHORES
 * @brief    Read to test and set, returns prior value. Write 0 to clear semaphore.
 * @{
 */
 #define MXC_F_SEMA_REVA_SEMAPHORES_SEMA_POS            0 /**< SEMAPHORES_SEMA Position */
 #define MXC_F_SEMA_REVA_SEMAPHORES_SEMA                ((uint32_t)(0x1UL << MXC_F_SEMA_REVA_SEMAPHORES_SEMA_POS)) /**< SEMAPHORES_SEMA Mask */

/**@} end of group SEMA_REVA_SEMAPHORES_Register */

/**
 * @ingroup  sema_reva_registers
 * @defgroup SEMA_REVA_STATUS SEMA_REVA_STATUS
 * @brief    Semaphore status bits. 0 indicates the semaphore is free, 1 indicates taken.
 * @{
 */
 #define MXC_F_SEMA_REVA_STATUS_STATUS_POS              0 /**< STATUS_STATUS Position */
 #define MXC_F_SEMA_REVA_STATUS_STATUS                  ((uint32_t)(0xFFUL << MXC_F_SEMA_REVA_STATUS_STATUS_POS)) /**< STATUS_STATUS Mask */

/**@} end of group SEMA_REVA_STATUS_Register */

#ifdef __cplusplus
}
#endif

#endif /* _SEMA_REVA_REGS_H_ */
