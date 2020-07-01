/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
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
******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "camera.h"
#include "sccb.h"
#include "ov7692_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"
#include "utils.h"

#define cambus_writeb(addr, x)      sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x)       sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_RGB565;

static const uint8_t default_regs[][2] = {
    {0x0e, 0x08}, // Sleep mode
    {0x69, 0x52}, // BLC window selection, BLC enable (default is 0x12)
    {0x1e, 0xb3}, // AddLT1F (default 0xb1)
    {0x48, 0x42},
    {0xff, 0x01}, // Select MIPI register bank
    {0xb5, 0x30},
    {0xff, 0x00}, // Select system control register bank
    {0x16, 0x03}, // (default)
    
    {0x62, 0x10}, // (default)
    {0x12, 0x01}, // Select Bayer RAW
    {0x17, 0x65}, // Horizontal Window Start Point Control (LSBs), default is 0x69
    {0x18, 0xa4}, // Horizontal sensor size (default)
    {0x19, 0x0c}, // Vertical Window Start Line Control (default)
    {0x1a, 0xf6}, // Vertical sensor size (default)
    {0x37, 0x04}, // PCLK is double system clock (default is 0x0c)
    {0x3e, 0x20}, // (default)
    {0x81, 0x3f}, // sde_en, uv_adj_en, scale_v_en, scale_h_en, uv_avg_en, cmx_en
    {0xcc, 0x02}, // High 2 bits of horizontal output size (default)
    {0xcd, 0x80}, // Low 8 bits of horizontal output size (default)
    {0xce, 0x01}, // Ninth bit of vertical output size (default)
    {0xcf, 0xe0}, // Low 8 bits of vertical output size (default)
    {0x82, 0x01}, // 01: Raw from CIP (default is 0x00)
    
    {0xc8, 0x02},
    {0xc9, 0x80},
    {0xca, 0x01},
    {0xcb, 0xe0},
    {0xd0, 0x28},
    {0x0e, 0x00}, // Normal mode (not sleep mode)
    {0x70, 0x00},
    {0x71, 0x34},
    {0x74, 0x28},
    {0x75, 0x98},
    {0x76, 0x00},
    {0x77, 0x64},
    {0x78, 0x01},
    {0x79, 0xc2},
    {0x7a, 0x4e},
    {0x7b, 0x1f},
    {0x7c, 0x00},
    {0x11, 0x01}, // CLKRC, Internal clock pre-scalar divide by 2 (default divide by 1)
    {0x20, 0x00}, // Banding filter (default)
    {0x21, 0x57}, // Banding filter (default is 0x44)
    {0x50, 0x4d},
    {0x51, 0x40}, // 60Hz Banding AEC 8 bits (default 0x80)
    {0x4c, 0x7d},
    {0x0e, 0x00},
    {0x80, 0x7f},
    {0x85, 0x00},
    {0x86, 0x00},
    {0x87, 0x00},
    {0x88, 0x00},
    {0x89, 0x2a},
    {0x8a, 0x22},
    {0x8b, 0x20},
    {0xbb, 0xab},
    {0xbc, 0x84},
    {0xbd, 0x27},
    {0xbe, 0x0e},
    {0xbf, 0xb8},
    {0xc0, 0xc5},
    {0xc1, 0x1e},
    {0xb7, 0x05},
    {0xb8, 0x09},
    {0xb9, 0x00},
    {0xba, 0x18},
    {0x5a, 0x1f},
    {0x5b, 0x9f},
    {0x5c, 0x69},
    {0x5d, 0x42},
    {0x24, 0x78}, // AGC/AEC
    {0x25, 0x68}, // AGC/AEC
    {0x26, 0xb3}, // AGC/AEC
    {0xa3, 0x0b},
    {0xa4, 0x15},
    {0xa5, 0x29},
    {0xa6, 0x4a},
    {0xa7, 0x58},
    {0xa8, 0x65},
    {0xa9, 0x70},
    {0xaa, 0x7b},
    {0xab, 0x85},
    {0xac, 0x8e},
    {0xad, 0xa0},
    {0xae, 0xb0},
    {0xaf, 0xcb},
    {0xb0, 0xe1},
    {0xb1, 0xf1},
    {0xb2, 0x14},
    {0x8e, 0x92},
    {0x96, 0xff},
    {0x97, 0x00},
    {0x14, 0x3b}, // AGC value, manual, set banding (default is 0x30)
    {0x0e, 0x00},
    
    {0x0c, 0xd6},
    {0x82, 0x3},
    {0x11, 0xf},  // Set clock prescaler
    {0x12, 0x6},
    {0x61, 0x0},
    {0x64, 0x11},
    {0xc3, 0x80},
    {0x81, 0x3f},
    {0x16, 0x3},
    {0x37, 0xc},
    {0x3e, 0x20},
    {0x5e, 0x0},
    {0xc4, 0x1},
    {0xc5, 0x80},
    {0xc6, 0x1},
    {0xc7, 0x80},
    {0xc8, 0x2},
    {0xc9, 0x80},
    {0xca, 0x1},
    {0xcb, 0xe0},
    {0xcc, 0x0},
    {0xcd, 0x40}, // Default to 64 line width
    {0xce, 0x0},
    {0xcf, 0x40}, // Default to 64 lines high
    {0x1c, 0x7f},
    {0x1d, 0xa2},
    {0xee, 0xee}  // End of register list marker 0xee
};

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;
    
    g_slv_addr = sccb_scan();
    
    if (g_slv_addr == -1) {
        return -1;
    }
    
    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int* id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;
    
    ret |= cambus_readb(PIDH, &id_high);
    ret |= cambus_readb(PIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int get_manufacture_id(int* id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;
    
    ret |= cambus_readb(MIDH, &id_high);
    ret |= cambus_readb(MIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int dump_registers(void)
{
    int ret = 0;
    uint8_t byt = 0;
    uint32_t i;
    uint8_t buf[64] = {0};
    uint8_t* ptr = buf;
    
    for (i = 0; ; i++) {
    
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }
        
        if (i == 256) {
            break;
        }
        
        ret = cambus_readb(i, &byt);
        
        if (ret == 0) {
            ret = sprintf((char*)ptr, " %02X", byt);
            
            if (ret < 0) {
                return ret;
            }
            
            ptr += 3;// XX + space
        }
        else {
            //printf("\nREAD FAILED: reg:%X\n", i);
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
        }
    }
    
    return ret;
}

static int reset(void)
{
    int ret = 0;
    ret |= cambus_writeb(REG12, REG12_RESET);
    utils_delay_ms(250);
    
    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xee); i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }
    
    //ret |= cambus_writeb(REG_CLKRC, 10); //InternalClk = InputClk / (2*(10+1))
    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
#if 0
    ret = cambus_readb(REG_COM2, &reg);
    
    if (ret == 0) {
        if (enable) {
            reg |= COM2_SOFT_SLEEP_MODE;
        }
        else {
            reg &= ~COM2_SOFT_SLEEP_MODE;
        }
        
        // Write back register
        ret |= cambus_writeb(REG_COM2, reg);
    }
    
#endif
    return ret;
}

static int read_reg(uint8_t reg_addr, uint8_t* reg_data)
{
    *reg_data = 0xff;
    
    if (cambus_readb(reg_addr, reg_data) != 0) {
        return -1;
    }
    
    return 0;
}

static int write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return cambus_writeb(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;
    
    g_pixelformat = pixformat;
    
    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        ret |= cambus_writeb(REG12, COLOR_YUV422);
        break;
        
    case PIXFORMAT_RGB444:
        ret |= cambus_writeb(REG12, COLOR_RGB444);
        break;
        
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        ret |= cambus_writeb(REG12, COLOR_RGB565);
        break;
        
    case PIXFORMAT_BAYER:
        ret |= cambus_writeb(REG12, COLOR_BAYER);
        break;
        
    default:
        ret = -1;
        break;
    }
    
    return ret;
}

static int get_pixformat(pixformat_t* pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}


static int set_framesize(int width, int height)
{
    int ret = 0;
    // Default input factor for large images
    uint8_t input_factor_large[] = { 0x02, 0x80, 0x01, 0xe0 };
    uint8_t input_factor_small[] = { 0x01, 0xbf, 0x01, 0xbf };
    uint8_t* input_factor_ptr = input_factor_large;
    
    // Image typically outputs one line short, add a line to account.
    height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_writeb(OH_HIGH, (width >> 8) & 0xff);
    ret |= cambus_writeb(OH_LOW, (width >> 0) & 0xff);
    ret |= cambus_writeb(OV_HIGH, (height >> 8) & 0xff);
    ret |= cambus_writeb(OV_LOW, (height >> 0) & 0xff);
    
    // Check and see if the target resolution is very small
    // that is less than 42 x 42, if so then apply and
    // x and y scaling factor.
    if ((width < 42) || (height < 42)) {
        input_factor_ptr = input_factor_small;
    }
    
    // Apply the appropriate input image factor.
    ret |= cambus_writeb(0xc8, input_factor_ptr[0]);
    ret |= cambus_writeb(0xc9, input_factor_ptr[1]);
    ret |= cambus_writeb(0xca, input_factor_ptr[2]);
    ret |= cambus_writeb(0xcb, input_factor_ptr[3]);
    return ret;
}



static int set_contrast(int level)
{
    int ret = 0;
#if 0
    
    switch (level) {
    case -4:
        level =   0;
        break;
        
    case -3:
        level =  30;
        break;
        
    case -2:
        level =  60;
        break;
        
    case -1:
        level =  90;
        break;
        
    case  0:
        level = 120;
        break;
        
    case  1:
        level = 150;
        break;
        
    case  2:
        level = 180;
        break;
        
    case  3:
        level = 210;
        break;
        
    case  4:
        level = 255;
        break;
        
    default:
        return -1;
    }
    
    ret = cambus_writeb(REG_CONTRAS, level);
#endif
    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;
#if 0
    
    switch (level) {
    case -4:
        level =   0;
        break;
        
    case -3:
        level =  30;
        break;
        
    case -2:
        level =  60;
        break;
        
    case -1:
        level =  90;
        break;
        
    case  0:
        level = 120;
        break;
        
    case  1:
        level = 150;
        break;
        
    case  2:
        level = 180;
        break;
        
    case  3:
        level = 210;
        break;
        
    case  4:
        level = 255;
        break;
        
    default:
        return -1;
    }
    
    // update REG_COM8
    ret = cambus_readb(REG_COM8, &reg);
    reg &= ~COM8_AEC;
    ret = cambus_writeb(REG_COM8, reg);
    
    // update REG_BRIGHT
    level &= 0xFF;
    reg = (level > 127) ? (level & 0x7f) : ((128 - level) | 0x80);
    ret = cambus_writeb(REG_BRIGHT, reg);
#endif
    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;
    
    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
#if 0
    ret = cambus_readb(REG_COM9, &reg);
    
    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    ret |= cambus_writeb(REG_COM9, reg);
#endif
    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;
    
    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;
    
    ret = cambus_readb(REG0C, &reg);
    
    if (enable) {
        reg |= HORIZONTAL_FLIP;
    }
    else {
        reg &= ~HORIZONTAL_FLIP;
    }
    
    ret |= cambus_writeb(REG0C, reg);
    
    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;
    
    if (enable) {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x82);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    }
    else {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x02);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    }
    
    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;
    
    ret = cambus_readb(REG0C, &reg);
    
    if (enable) {
        reg |= VERTICAL_FLIP;
    }
    else {
        reg &= ~VERTICAL_FLIP;
    }
    
    ret |= cambus_writeb(REG0C, reg);
    
    return ret;
}

/******************************** Public Functions ***************************/
int sensor_register(camera_t* camera)
{
    // Initialize sensor structure.
    camera->init                = init;
    camera->get_slave_address   = get_slave_address;
    camera->get_product_id      = get_product_id;
    camera->get_manufacture_id  = get_manufacture_id;
    camera->dump_registers      = dump_registers;
    camera->reset               = reset;
    camera->sleep               = sleep;
    camera->read_reg            = read_reg;
    camera->write_reg           = write_reg;
    camera->set_pixformat       = set_pixformat;
    camera->get_pixformat       = get_pixformat;
    camera->set_framesize       = set_framesize;
    camera->set_contrast        = set_contrast;
    camera->set_brightness      = set_brightness;
    camera->set_saturation      = set_saturation;
    camera->set_gainceiling     = set_gainceiling;
    camera->set_colorbar        = set_colorbar;
    camera->set_hmirror         = set_hmirror;
    camera->set_vflip           = set_vflip;
    camera->set_negateimage     = set_negateimage;
    
    return 0;
}
