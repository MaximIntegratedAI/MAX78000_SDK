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

/**
 * @file    main.c
 * @brief   Parallel camera example with the OV7692.
 *
 * @details This example uses the UART to stream out the image captured from the camera.
 *          The image is prepended with a header that is interpreted by the grab_image.py
 *          python script.  The data from this example through the UART is in a binary format.
 *          Instructions: 1) Load and execute this example. The example will initialize the camera
 *                        and start the repeating binary output of camera frame data.
 *                        2) Run 'sudo grab_image.py /dev/ttyUSB0 115200'
 *                           Substitute the /dev/ttyUSB0 string for the serial port on your system.
 *                           The python program will read in the binary data from this example and
 *                           output a png image.
 */

/***** Includes *****/
#include <stdio.h>
#include <stdint.h>
#include "mxc_device.h"
#include "uart.h"
#include "led.h"
#include "board.h"
#include "mxc_delay.h"
#include "camera.h"
#include "utils.h"
#include "dma.h"

#define IMAGE_XRES  64
#define IMAGE_YRES  64
#define CAMERA_FREQ (10 * 1000 * 1000)

void process_img(void)
{
    uint8_t*   raw;
    uint32_t  imgLen;
    uint32_t  w, h;
    
    // Get the details of the image from the camera driver.
    camera_get_image(&raw, &imgLen, &w, &h);
    // Send the image through the UART to the console.
    // A python program will read from the console and write
    // to an image file.
    utils_send_img_to_pc(raw, imgLen, w, h, camera_get_pixel_format());
}

// *****************************************************************************
int main(void)
{
    int ret = 0;
    int slaveAddress;
    int id;
    int dma_channel;

    // Initialize DMA for camera interface
    MXC_DMA_Init();
    dma_channel = MXC_DMA_AcquireChannel();

    // Initialize the camera driver.
    camera_init(CAMERA_FREQ);
    printf("\n\nCameraIF Example\n");
    
    // Obtain the I2C slave address of the camera.
    slaveAddress = camera_get_slave_address();
    printf("Camera I2C slave address is %02x\n", slaveAddress);
    
    // Obtain the product ID of the camera.
    ret = camera_get_product_id(&id);
    
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }
    
    printf("Camera Product ID is %04x\n", id);
    
    // Obtain the manufactor ID of the camera.
    ret = camera_get_manufacture_id(&id);
    
    if (ret != STATUS_OK) {
        printf("Error returned from reading camera id. Error %d\n", ret);
        return -1;
    }
    
    printf("Camera Manufacture ID is %04x\n", id);
    
    // Setup the camera image dimensions, pixel format and data aquiring details.
    ret = camera_setup(IMAGE_XRES, IMAGE_YRES, PIXFORMAT_RGB888, FIFO_THREE_BYTE, USE_DMA, dma_channel);
    
    if (ret != STATUS_OK) {
        printf("Error returned from setting up camera. Error %d\n", ret);
        return -1;
    }
    
    // Display a human readable banner.  After this banner then send image in binary format.
    printf("Use the pc_utility/grab_image.py script to capture frames from this example.\n");
    printf("Will start sending camera frames in binary format\n");
    printf("In 5 seconds...\n");
    MXC_Delay(SEC(5));
    
    // Start off the first camera image frame.
    camera_start_capture_image();
    
    while (1) {
        // Check if image is aquired.
        if (camera_is_image_rcv()) {
            // Process the image, send it through the UART console.
            process_img();
            MXC_Delay(SEC(1));
            // Prepare for another frame capture.
            camera_start_capture_image();
        }
    }
    
    return ret;
}
