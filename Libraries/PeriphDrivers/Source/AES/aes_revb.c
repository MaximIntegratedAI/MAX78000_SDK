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

#include <stdlib.h>
#include <string.h>
#include "mxc_sys.h"
#include "mxc_device.h"
#include "mxc_errors.h"
#include "mxc_assert.h"
#include "mxc_lock.h"

#include "dma.h"
#include "aes_regs.h"
#include "aes_key_regs.h"
#include "trng.h"
#include "aes_revb.h"

/* **** Variable Declaration **** */
typedef struct {
  uint8_t enc;
  uint8_t channelRX;
  uint8_t channelTX;
  uint32_t remain;
  uint32_t* inputText;
  uint32_t* outputText;
} mxc_aes_dma_req_t;

static mxc_aes_dma_req_t dma_state;

static void memcpy32r(uint32_t * dst, const uint32_t * src, unsigned int len)
{
  uint32_t * dstr = dst + (len/4) - 1;
  uint32_t temp;
  while (len) {
    temp = *src;
    *dstr = ((temp & 0xff)<<24)|((temp & 0xff00)<<8)|((temp & 0xff0000)>>8)|((temp & 0xff000000)>>24);
    dstr--;
    src++;
    len -= 4;
  }
}

int MXC_AES_RevB_Init(void)
{
  MXC_AES->ctrl = 0x00;

  MXC_TRNG_GenerateKey();

  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_AES->ctrl |= MXC_F_AES_CTRL_EN;

  return E_NO_ERROR;
}

int MXC_AES_RevB_Shutdown(void)
{
  MXC_AES_RevB_FlushInputFIFO();
  MXC_AES_RevB_FlushOutputFIFO();

  while (MXC_AES_IsBusy() != E_NO_ERROR) ;

  MXC_AES->ctrl = 0x00;

  return E_NO_ERROR;
}

int MXC_AES_RevB_IsBusy(void)
{
  if (MXC_AES->status & MXC_F_AES_STATUS_BUSY) {
    return E_BUSY;
  }

  return E_NO_ERROR;
}

void MXC_AES_RevB_SetKeySize(mxc_aes_keys_t key)
{
  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_AES->ctrl |= key;
}

mxc_aes_keys_t MXC_AES_RevB_GetKeySize(void)
{
  return (MXC_AES->ctrl & MXC_F_AES_CTRL_KEY_SIZE);
}

void MXC_AES_RevB_FlushInputFIFO(void)
{
  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_AES->ctrl |= MXC_F_AES_CTRL_INPUT_FLUSH;
}

void MXC_AES_RevB_FlushOutputFIFO(void)
{
  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_AES->ctrl |= MXC_F_AES_CTRL_OUTPUT_FLUSH;
}

void MXC_AES_RevB_Start(void)
{
  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_AES->ctrl |= MXC_F_AES_CTRL_START;
}

void MXC_AES_RevB_EnableInt(uint32_t interrupt)
{
  MXC_AES->inten |= (interrupt & (MXC_F_AES_INTEN_DONE | MXC_F_AES_INTEN_KEY_CHANGE | \
                                  MXC_F_AES_INTEN_KEY_ZERO | MXC_F_AES_INTEN_OV));
}

void MXC_AES_RevB_DisableInt(uint32_t interrupt)
{
  MXC_AES->inten &= ~(interrupt & (MXC_F_AES_INTEN_DONE | MXC_F_AES_INTEN_KEY_CHANGE | \
                                   MXC_F_AES_INTEN_KEY_ZERO | MXC_F_AES_INTEN_OV));
}

uint32_t MXC_AES_RevB_GetFlags(void)
{
  return MXC_AES->intfl;
}

void MXC_AES_RevB_ClearFlags(uint32_t flags)
{
  MXC_AES->intfl = (flags & (MXC_F_AES_INTFL_DONE | MXC_F_AES_INTFL_KEY_CHANGE | \
                             MXC_F_AES_INTFL_KEY_ZERO | MXC_F_AES_INTFL_OV));
}

int MXC_AES_RevB_Generic(mxc_aes_req_t* req)
{
  int i;
  int remain = req->length;

  if (req == NULL) {
    return E_NULL_PTR;
  }

  if (req->inputData == NULL || req->resultData == NULL) {
    return E_NULL_PTR;
  }

  if (req->length == 0) {
    return E_BAD_PARAM;
  }

  MXC_AES_RevB_FlushInputFIFO();
  MXC_AES_RevB_FlushOutputFIFO();

  MXC_AES_RevB_SetKeySize(req->keySize);

  while (MXC_AES_IsBusy() != E_NO_ERROR);

  MXC_SETFIELD(MXC_AES->ctrl, MXC_F_AES_CTRL_TYPE, req->encryption << MXC_F_AES_CTRL_TYPE_POS);

  while (remain / 4) {
    for (i = 0; i < 4; i++) {
      MXC_AES->fifo = * (req->inputData++);
    }

    while (!(MXC_AES->intfl & MXC_F_AES_INTFL_DONE));

    MXC_AES->intfl |= MXC_F_AES_INTFL_DONE;

    for (i = 0; i < 4; i++) {
      * (req->resultData++) = MXC_AES->fifo;
    }

    remain -= 4;
  }

  if (remain % 4) {
    for (i = 0; i < remain; i++) {
      MXC_AES->fifo = * (req->inputData++);
    }

    MXC_AES_RevB_Start();

    while (!(MXC_AES->intfl & MXC_F_AES_INTFL_DONE));

    MXC_AES->intfl |= MXC_F_AES_INTFL_DONE;

    for (i = 0; i < 4; i++) {
      * (req->resultData++) = MXC_AES->fifo;
    }
  }

  return E_NO_ERROR;
}

int MXC_AES_RevB_Encrypt(mxc_aes_req_t* req)
{
  return MXC_AES_RevB_Generic(req);
}

int MXC_AES_RevB_Decrypt(mxc_aes_req_t* req)
{
  return MXC_AES_Generic(req);
}

int MXC_AES_RevB_TXDMAConfig(void* src_addr, int len)
{
  uint8_t channel;
  mxc_dma_config_t config;
  mxc_dma_srcdst_t srcdst;

  if (src_addr == NULL) {
    return E_NULL_PTR;
  }

  if (len == 0) {
    return E_BAD_PARAM;
  }

  MXC_DMA_Init();

  channel = MXC_DMA_AcquireChannel();
  dma_state.channelTX = channel;

  config.reqsel = MXC_DMA_REQUEST_AESTX;

  config.ch = channel;

  config.srcwd = MXC_DMA_WIDTH_WORD;
  config.dstwd = MXC_DMA_WIDTH_WORD;

  config.srcinc_en = 1;
  config.dstinc_en = 0;

  srcdst.ch = channel;
  srcdst.source = src_addr;

  if (dma_state.enc == 1) {
    srcdst.len = 4;
  } else if (len > 4) {
    srcdst.len = 4;
  } else {
    srcdst.len = len;
  }

  MXC_DMA_ConfigChannel(config, srcdst);
  MXC_DMA_SetCallback(channel, MXC_AES_RevB_DMACallback);

  MXC_DMA_EnableInt(channel);
  MXC_DMA_Start(channel);
  MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

  return E_NO_ERROR;
}

int MXC_AES_RevB_RXDMAConfig(void* dest_addr, int len)
{
  if (dest_addr == NULL) {
    return E_NULL_PTR;
  }

  if (len == 0) {
    return E_BAD_PARAM;
  }

  uint8_t channel;
  mxc_dma_config_t config;
  mxc_dma_srcdst_t srcdst;

  MXC_DMA_Init();

  channel = MXC_DMA_AcquireChannel();
  dma_state.channelRX = channel;

  config.reqsel = MXC_DMA_REQUEST_AESRX;

  config.ch = channel;

  config.srcwd = MXC_DMA_WIDTH_WORD;
  config.dstwd = MXC_DMA_WIDTH_WORD;

  config.srcinc_en = 0;
  config.dstinc_en = 1;

  srcdst.ch = channel;
  srcdst.dest = dest_addr;

  if (dma_state.enc == 0) {
    srcdst.len = 4;
  } else if (len > 4) {
    srcdst.len = 4;
  } else {
    srcdst.len = len;
  }

  MXC_DMA_ConfigChannel(config, srcdst);
  MXC_DMA_SetCallback(channel, MXC_AES_RevB_DMACallback);

  MXC_DMA_EnableInt(channel);
  MXC_DMA_Start(channel);
  MXC_DMA->ch[channel].ctrl |= MXC_F_DMA_CTRL_CTZ_IE;

  return E_NO_ERROR;
}

int MXC_AES_RevB_GenericAsync(mxc_aes_req_t* req, uint8_t enc)
{
  if (req == NULL) {
    return E_NULL_PTR;
  }

  if (req->inputData == NULL || req->resultData == NULL) {
    return E_NULL_PTR;
  }

  if (req->length == 0) {
    return E_BAD_PARAM;
  }

  MXC_AES_FlushInputFIFO();
  MXC_AES_FlushOutputFIFO();

  MXC_AES_SetKeySize(req->keySize);

  MXC_AES_IsBusy();
  MXC_SETFIELD(MXC_AES->ctrl, MXC_F_AES_CTRL_TYPE, req->encryption << MXC_F_AES_CTRL_TYPE_POS);

  dma_state.enc = enc;
  dma_state.remain = req->length;
  dma_state.inputText = req->inputData;
  dma_state.outputText = req->resultData;

  MXC_AES->ctrl |= MXC_F_AES_CTRL_DMA_RX_EN;              /*Enable AES DMA */
  MXC_AES->ctrl |= MXC_F_AES_CTRL_DMA_TX_EN;              /*Enable AES DMA */

  if (MXC_AES_RevB_TXDMAConfig(dma_state.inputText, dma_state.remain) != E_NO_ERROR) {
    return E_BAD_PARAM;
  }

  return E_NO_ERROR;
}

int MXC_AES_RevB_EncryptAsync(mxc_aes_req_t* req)
{
  return MXC_AES_RevB_GenericAsync(req, 0);
}

int MXC_AES_RevB_DecryptAsync(mxc_aes_req_t* req)
{
  return MXC_AES_RevB_GenericAsync(req, 1);
}

void MXC_AES_RevB_DMACallback(int ch, int error)
{
  if (error != E_NO_ERROR) {

  } else {
    if (dma_state.channelTX == ch) {
      MXC_DMA_ReleaseChannel(dma_state.channelTX);

      if (dma_state.remain < 4) {
        MXC_AES_RevB_Start();
      }

      MXC_AES_RevB_RXDMAConfig(dma_state.outputText, dma_state.remain);
    } else if (dma_state.channelRX == ch) {
      if (dma_state.remain > 4) {
        dma_state.remain -= 4;
      } else if (dma_state.remain > 0) {
        dma_state.remain = 0;
      }

      MXC_DMA_ReleaseChannel(dma_state.channelRX);

      if (dma_state.remain > 0) {
        MXC_AES_RevB_TXDMAConfig(dma_state.inputText, dma_state.remain);
      }
    }
  }
}

void MXC_AES_RevB_SetExtKey(const void* key, mxc_aes_keys_t len)
{
  int numBytes;

  if(len == MXC_AES_128BITS) {
    numBytes = 16;
  } else if (len == MXC_AES_192BITS) {
    numBytes = 24;
  } else {
    numBytes = 32;
  }

  /* TODO: Figure out if this is the correct byte ordering */
  memcpy32r((void*)&(MXC_AESKEY->aes_key0), key, numBytes);
}
