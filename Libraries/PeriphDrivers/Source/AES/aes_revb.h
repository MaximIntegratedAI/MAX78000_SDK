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

#include "aes.h"

int MXC_AES_RevB_Init (void);
void MXC_AES_RevB_EnableInt (uint32_t interrupt);
void MXC_AES_RevB_DisableInt (uint32_t interrupt);
int MXC_AES_RevB_IsBusy (void);
int MXC_AES_RevB_Shutdown (void);
void MXC_AES_RevB_GenerateKey(void);
void MXC_AES_RevB_SetKeySize(mxc_aes_keys_t key);
mxc_aes_keys_t MXC_AES_RevB_GetKeySize(void);
void MXC_AES_RevB_FlushInputFIFO(void);
void MXC_AES_RevB_FlushOutputFIFO(void);
void MXC_AES_RevB_Start(void);
uint32_t MXC_AES_RevB_GetFlags(void);
void MXC_AES_RevB_ClearFlags(uint32_t flags);
int MXC_AES_RevB_Generic(mxc_aes_req_t* req);
int MXC_AES_RevB_Encrypt(mxc_aes_req_t* req);
int MXC_AES_RevB_Decrypt(mxc_aes_req_t* req);
int MXC_AES_RevB_TXDMAConfig(void *src_addr, int len);
int MXC_AES_RevB_RXDMAConfig(void *dest_addr, int len);
int MXC_AES_RevB_GenericAsync(mxc_aes_req_t* req, uint8_t enc);
int MXC_AES_RevB_EncryptAsync(mxc_aes_req_t* req);
int MXC_AES_RevB_DecryptAsync(mxc_aes_req_t* req);
void MXC_AES_RevB_DMACallback (int ch, int error);