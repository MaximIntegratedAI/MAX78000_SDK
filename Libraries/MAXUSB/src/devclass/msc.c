/*******************************************************************************
 * Copyright (C) 2017 Maxim Integrated Products, Inc., All Rights Reserved.
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

#include "usb.h"
#include "usb_event.h"
#include "enumerate.h"
#include "msc.h"
#include "string.h"

/***** Definitions *****/

/* USB Mass-Storage class status values */
#define CSW_OK                  0
#define CSW_FAILED              1

#define INTERFACE               2

/* USB Mass-storage class requests */
#define MSC_MASS_STORAGE_RESET          0xff
#define MSC_GET_MAX_LUN                 0xfe

#define CBW_LEN                         31
#define CSW_LEN                         13
#define REQSENSE_LEN                    18
#define MODESENSE_LEN                   4
#define INQUIRY_LEN                     36
#define RECVDIAGRES_LEN                 32
#define FORMATCAPACITY_LEN              12
#define CAPACITY_LEN                    8
#define CAPACITY16_LEN                  32

#define FORMATTED_MEDIA                 0x02000000

#define CMD_TEST_UNIT_READY             0x00
#define CMD_REQUEST_SENSE               0x03
#define CMD_INQUIRY                     0x12
#define CMD_MODE_SENSE_6                0x1A
#define CMD_START_STOP                  0x1B
#define CMD_RECEIVE_DIAGNOSTIC_RESULT   0x1C
#define CMD_MEDIUM_REMOVAL              0x1E
#define CMD_READ_FORMAT_CAPACITY        0x23
#define CMD_READ_CAPACITY               0x25
#define CMD_READ_10                     0x28
#define CMD_WRITE_10                    0x2A
#define CMD_MODE_SENSE_10               0x5A
#define CMD_READ_CAPACITY_16            0x9E

#define REQSENSE_OK                     0
#define REQSENSE_CHECK_CONDITION        2

#define CSW_SIGNATURE_IDX               0
#define CSW_TAG_IDX                     4
#define CSW_DATARESIDUE_IDX             8
#define CSW_STATUS_IDX                  12

#define EP_CONTROL                      0

#define CBW_SIGNATURE_IDX               0
#define CBW_TAG_IDX                     4
#define CBW_DATA_TRANSFER_LENGTH_IDX    8
#define CBW_FLAGS_IDX                   12
#define CBW_LUN_IDX                     13
#define CBW_CBLENGTH_IDX                14
#define CBW_CB_IDX                      15

/* Function pointer definition.  Used to call a funtion after a read or write completes. */
typedef void callbackFunc();

/***** File Scope Data *****/
static uint8_t if_num[INTERFACE] = {0xFF,0xFF};   

static MXC_USB_Req_t req[INTERFACE];
static msc_callback_req_t reqCB[INTERFACE];

static uint8_t req_data[INTERFACE][MAX_PACKET_SIZE];
static uint8_t csw_data[INTERFACE][CSW_LEN];

/* Storage for application specific data */
static msc_idstrings_t id_strings[INTERFACE];
static msc_mem_t mem[INTERFACE];

/* Endpoint numbers */
static unsigned int ep_from_host[INTERFACE];
static unsigned int ep_to_host[INTERFACE];

static uint32_t numBlocks[INTERFACE];
static uint32_t blockAddr[INTERFACE];

static int (*chained_func)(MXC_USB_SetupPkt *, void *);
static void *chained_cbdata;

/***** Function Prototypes *****/
static int msc_classReq(MXC_USB_SetupPkt *sud, void *cbdata);
static void msc_write(int ep, int len, void* cbdata, const uint8_t interface);
static void msc_writeComplete(void* cbdata);
static void msc_read(int ep, int len, void (*callback)(void *), void *cbdata, const uint8_t interface);
static void msc_waitForCBW(const uint8_t interface);
static void msc_cbwReceived(void* cbdata);
static void msc_sendCSW(const uint8_t interface);
static void msc_sendRequestSense(const uint8_t interface);
static void msc_sendInquiry(const uint8_t interface);
static void msc_sendModeSense(const uint8_t interface);
static void msc_sendReceiveDiagResult(const uint8_t interface);
static void msc_sendFormatCapacity(const uint8_t interface);
static void msc_sendCapacity(const uint8_t interface);
static void msc_sendCapacity16(const uint8_t interface);
static void msc_readMem(const uint8_t interface);
static void msc_writeMem(const uint8_t interface);
static void msc_writeMemBlock(void* cbdata);

/******************************************************************************/

int msc_init(const MXC_USB_interface_descriptor_t *if_desc, const msc_idstrings_t* ids, const msc_mem_t* funcs)
{
    int err;
    uint8_t interface = if_desc->bInterfaceNumber;
    
    ep_from_host[interface] = 0;
    ep_to_host[interface] = 0;
    
    memcpy(&id_strings[interface], ids, sizeof(msc_idstrings_t));
    memcpy(&mem[interface], funcs, sizeof(msc_mem_t));
    
    /* Prep "disk" memory */
    err = mem[interface].init();
    
    if (!err) 
    {
        enum_query_callback(ENUM_CLASS_REQ, &chained_func, &chained_cbdata); 
    
        /* Handle class-specific SETUP requests */
        err = enum_register_callback(ENUM_CLASS_REQ, msc_classReq, (void *)(&if_num[interface]));

        /* Store interface number */
        if_num[interface] = interface;
    }    
    /* Return zero for success or non-zero for error. */
    return err;
}

/******************************************************************************/

int msc_configure(const msc_cfg_t *cfg, const uint8_t interface)
{
    int err;
    
    /* This driver is limited to handling blocks no bigger than MAX_PACKET_SIZE. */
    if((cfg->out_maxpacket > MAX_PACKET_SIZE) || (cfg->in_maxpacket > MAX_PACKET_SIZE))
    {
        return 1;
    }

    /* Setup endpoint used to receive communication from the host. */
    ep_from_host[interface] = cfg->out_ep;
    if ((err = MXC_USB_ConfigEp(ep_from_host[interface], MAXUSB_EP_TYPE_OUT, cfg->out_maxpacket)) != 0) {
        msc_deconfigure(interface);
        return err;
    }

    /* Setup endpoint used to send communication to the host. */
    ep_to_host[interface] = cfg->in_ep;
    if ((err = MXC_USB_ConfigEp(ep_to_host[interface], MAXUSB_EP_TYPE_IN, cfg->in_maxpacket)) != 0) {
        msc_deconfigure(interface);
        return err;
    }
    
    /* Device has been configured, prep the memory for transactions. */
    return mem[interface].start();
}

/******************************************************************************/

int msc_deconfigure(const uint8_t interface)
{
    /* Release endpoints if they have been configured */
    if (ep_from_host[interface] != 0) {
        MXC_USB_ResetEp(ep_from_host[interface]);
        ep_from_host[interface] = 0;
    }

    if (ep_to_host[interface] != 0) {
        MXC_USB_ResetEp(ep_to_host[interface]);
        ep_to_host[interface] = 0;
    }

    return 0;
}

/******************************************************************************/

static int msc_classReq(MXC_USB_SetupPkt *sud, void *cbdata)
{
    int result = -1;
    uint8_t interface = *((uint8_t*)cbdata);

    if ((sud->bmRequestType & RT_RECIP_IFACE) && (sud->wIndex == if_num[interface])) 
    {
        switch (sud->bRequest) {
            case MSC_GET_MAX_LUN:
                if (sud->bmRequestType & RT_DEV_TO_HOST) {
                    /* Get Max LUN */
                    req_data[interface][0] = 0;
                    msc_write(EP_CONTROL, 1, (void *)msc_waitForCBW, interface);
                    result = 0;
                }
                break;
            case MSC_MASS_STORAGE_RESET:
                /* Reset the control endpoint */
                /* Abort any in-progress USB request */
                MXC_USB_RemoveRequest(&req[interface]);
                /* Prepare for new CBW */
                msc_waitForCBW(interface);
                /* Success, no data stage */
                result = 0;
                break;
            default:
                /* Unexpected message received -- stall */
                break;
        }
    }
    else
    {
	    /* Not for this class, send to chained classes (if any) */
        if (chained_func != NULL)
	        result = chained_func(sud, chained_cbdata);        
    }
    
    return result;
}

/******************************************************************************/

/* Send bytes to the host */
/* cbdata contains a function pointer that will be called once the write completes. */
static void msc_write(int ep, int len, void* cbdata, const uint8_t interface)
{
    reqCB[interface].cbFunc = cbdata;
    reqCB[interface].interfaceNum = interface;

    memset(&req[interface], 0, sizeof(MXC_USB_Req_t));
    req[interface].ep = ep;
    req[interface].data = req_data[interface];
    req[interface].reqlen = len;
    req[interface].callback = msc_writeComplete;
    req[interface].cbdata = (void*)(&reqCB[interface]);
    req[interface].type = MAXUSB_TYPE_TRANS;
    MXC_USB_WriteEndpoint(&req[interface]);
}

/******************************************************************************/

static void msc_writeComplete(void* cbdata)
{
    uint8_t interface = ((msc_callback_req_t*)(cbdata))->interfaceNum;
    void* cb = ((msc_callback_req_t*)(cbdata))->cbFunc;
        
    /* A write to the host has completed.  Check if the correct number of bytes were transferred. */
    if ((!req[interface].error_code) && (req[interface].reqlen == req[interface].actlen)) {
        if(!req[interface].ep) {
            /* ACK the status stage, only for Control EP */
            MXC_USB_Ackstat(req[interface].ep);
        }
        /* Call the requested function (if any) */
        if(cbdata) {
            ((callbackFunc*)cb)(interface);
        }
    } else {
        if(!req[interface].ep) {
            /* Wrote an unexpected number of bytes. */
            MXC_USB_Stall(req[interface].ep);
        }
    }
}

/******************************************************************************/

/* Read bytes from the host */
/* callback contains a function pointer to be called when the read completes. */
static void msc_read(int ep, int len, void (*callback)(void *), void *cbdata, const uint8_t interface)
{
    memset(&req[interface], 0, sizeof(MXC_USB_Req_t));
    req[interface].ep = ep;
    req[interface].data = req_data[interface];
    req[interface].reqlen = len;
    req[interface].callback = callback;
    req[interface].cbdata = (void *)(&if_num[interface]);
    req[interface].type = MAXUSB_TYPE_TRANS;
    MXC_USB_ReadEndpoint(&req[interface]);
}

/******************************************************************************/

static void msc_waitForCBW(const uint8_t interface)
{
    /* Submit a read request to the USB stack. */
    /* Have it call cbwReceived when the read completes. */
    msc_read(ep_from_host[interface], CBW_LEN, msc_cbwReceived, NULL, interface);
}

/******************************************************************************/

static void msc_cbwReceived(void* cbdata)
{
    uint8_t interface = *((uint8_t *)(cbdata));

    /* Verify this is a valid CBW */
    if((!req[interface].error_code) && (req[interface].actlen == CBW_LEN) && 
        (strncmp((const char *)((&req_data[interface][0]) + CBW_SIGNATURE_IDX), "USBC", 4) == 0)) 
    {
        /* Construct the response CSW */
        memset(csw_data, 0, CSW_LEN);
        memcpy((&(csw_data[interface][0]) + CSW_SIGNATURE_IDX), "USBS", 4);
        memcpy((&(csw_data[interface][0]) + CSW_TAG_IDX), (&(req_data[interface][0]) + CBW_TAG_IDX), 4);

        /* Assume a good response for now.  Commands can overwrite this as needed */
        csw_data[interface][CSW_STATUS_IDX] = CSW_OK;

        /* More CBW validation. */
        if((req_data[interface][CBW_CBLENGTH_IDX] & 0x1F) > 16) 
        {
            MXC_USB_Stall(EP_CONTROL);
        }

        /* Act on the command received.  Most commands will process some data
           from the host, or send data to the host.  Once all the required 
           data has been transmitted, a CSW must be sent.  Those commands that
           have no data can send the CSW immediately.  Others will need to rely
           on callbacks to send the CSW after the data is transmitted. */

        switch(req_data[interface][CBW_CB_IDX]) 
        {
            case CMD_TEST_UNIT_READY:
                /* This command only requires a CSW to be sent back. */
                /* Prep the response and send it along. */
                csw_data[interface][CSW_STATUS_IDX] = mem[interface].ready() ? CSW_OK : CSW_FAILED;
                msc_sendCSW(interface);
                break;
            case CMD_REQUEST_SENSE:
                msc_sendRequestSense(interface);
                break;
            case CMD_INQUIRY:
                msc_sendInquiry(interface);
                break;
            case CMD_START_STOP:
                /* This command only requires a CSW to be sent back. */
                /* Prep the response and send it along. */
                if(req_data[interface][19] & 2) {
                    /* STOP */
                    /* Flush any pending transactions and report the results. */
                    csw_data[interface][CSW_STATUS_IDX] = mem[interface].stop() ? CSW_FAILED : CSW_OK;
                } else {
                    /* START */
                    mem[interface].start();
                    csw_data[interface][CSW_STATUS_IDX] = mem[interface].ready() ? CSW_OK : CSW_FAILED;
                }
                msc_sendCSW(interface);
                break;
            case CMD_MODE_SENSE_6:
            case CMD_MODE_SENSE_10:
                msc_sendModeSense(interface);
                break;
            case CMD_MEDIUM_REMOVAL:
                /* This command only requires a CSW to be sent back. */
                msc_sendCSW(interface);
                break;
            case CMD_RECEIVE_DIAGNOSTIC_RESULT:
                msc_sendReceiveDiagResult(interface);
                break;
            case CMD_READ_FORMAT_CAPACITY:
                msc_sendFormatCapacity(interface);
                break;
            case CMD_READ_CAPACITY:
                msc_sendCapacity(interface);
                break;
            case CMD_READ_CAPACITY_16:
                msc_sendCapacity16(interface);
                break;
            case CMD_READ_10:
                /* The read command may consist of multiple blocks.  The readMem function will 
                   handle sending all of the blocks followed by sending the CSW. */
                numBlocks[interface] = req_data[interface][22] << 8 | req_data[interface][23];
                blockAddr[interface] = req_data[interface][17] << 24 | req_data[interface][18] << 16 | req_data[interface][19] << 8 | req_data[interface][20];
                msc_readMem(interface);
                break;
            case CMD_WRITE_10:
                /* The write command may consist of multiple blocks.  The writeMem function will 
                   handle receiving all of the blocks followed by sending the CSW. */
                numBlocks[interface] = req_data[interface][22] << 8 | req_data[interface][23];
                blockAddr[interface] = req_data[interface][17] << 24 | req_data[interface][18] << 16 | req_data[interface][19] << 8 | req_data[interface][20];
                msc_writeMem(interface);
                break;
            default:
                /* This command wasn't recognized.  Send an error CSW. */
                csw_data[interface][CSW_STATUS_IDX] = CSW_FAILED;
                msc_sendCSW(interface);
                break;
        }
    } 
    else 
    {
        /* Something not right about this CBW */
        MXC_USB_Stall(req[interface].ep);
    }
}

/******************************************************************************/

static void msc_sendRequestSense(const uint8_t interface)
{
    /* Prep the response data */
    req_data[interface][0]  = 0x70;     /* Error Code */
    req_data[interface][1]  = 0x00;     /* Segment Number */
    req_data[interface][2]  = mem[interface].ready() ? REQSENSE_OK : REQSENSE_CHECK_CONDITION; /* Sense Key */
    req_data[interface][3]  = 0x00;     /* Info - 4 Bytes */
    req_data[interface][4]  = 0x00;     
    req_data[interface][5]  = 0x00;     
    req_data[interface][6]  = 0x00;     
    req_data[interface][7]  = 0x0A;     /* Additional Sense Length */
    req_data[interface][8]  = 0x00;     /* Command Specific Info - 4 Bytes */
    req_data[interface][9]  = 0x00;     
    req_data[interface][10] = 0x00;     
    req_data[interface][11] = 0x00;     
    req_data[interface][12] = 0x3A;     /* Additional Sense Code */
    req_data[interface][13] = 0x00;     /* Additional Send Qualifier */
    req_data[interface][14] = 0x00;     /* Reserved - 4 Bytes */
    req_data[interface][15] = 0x00;     
    req_data[interface][16] = 0x00;     
    req_data[interface][17] = 0x00;     

    /* Send the data to the host and have the USB stack call
       send CSW once complete. */
    msc_write(ep_to_host[interface], REQSENSE_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendInquiry(const uint8_t interface)
{
    req_data[interface][0]  = 0x00;               // Peripheral Device Type (PDT) - SBC Direct-access device
    req_data[interface][1]  = 0x80;               // Removable Medium Bit is Set
    req_data[interface][2]  = 0x02;               // Version
    req_data[interface][3]  = 0x02;               // Obsolete[7:6],NORMACA[5],HISUP[4],Response Data Format[3:0]
    req_data[interface][4]  = 0x1f;               // Additional Length
    req_data[interface][5]  = 0x73;               // SCCS[7],ACC[6],TPGS[5:4],3PC[3],Reserved[2:1],PROTECT[0]
    req_data[interface][6]  = 0x6d;               // BQUE[7],ENCSERV[6],VS[5],MULTIP[4],MCHNGR[3],Obsolete[2:1],ADDR16[0]
    req_data[interface][7]  = 0x69;               // Obsolete[7:6],WBUS116[5],SYNC[4],LINKED[3],Obsolete[2],CMDQUE[1],VS[0]

    memcpy(&(req_data[interface][8]), id_strings[interface].vendor, VENDOR_ID_MAX_LEN);        /* Copy application specific data in */
    memcpy(&(req_data[interface][16]), id_strings[interface].product, PRODUCT_ID_MAX_LEN);
    memcpy(&(req_data[interface][32]), id_strings[interface].revision, PRODUCT_REV_MAX_LEN);

    msc_write(ep_to_host[interface], INQUIRY_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendCSW(const uint8_t interface)
{
    /* Previously received command has stored its status data in the csw_data array
       Copy that data to the outgoing buffer, send it along, and have the USB stack call
       waitForCBW once all of the data has been transmitted.  This will arm the driver
       for receiving the next command. */
    memcpy(&(req_data[interface][0]), &(csw_data[interface][0]), CSW_LEN);
    msc_write(ep_to_host[interface], CSW_LEN, (void *)msc_waitForCBW, interface);
}

/******************************************************************************/

static void msc_sendModeSense(const uint8_t interface)
{
    /* Prep the response and send it followed by sending the CSW */
    req_data[interface][0] = 0x04;
    req_data[interface][1] = 0x00;
    req_data[interface][2] = (mem[interface].write == NULL) ? 0x90 : 0x10;
    req_data[interface][3] = 0x00;
    msc_write(ep_to_host[interface], MODESENSE_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendReceiveDiagResult(const uint8_t interface)
{
    /* Prep the response and send it followed by sending the CSW */
    memset(&req_data[interface][0], 0, RECVDIAGRES_LEN);
    msc_write(ep_to_host[interface], RECVDIAGRES_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendFormatCapacity(const uint8_t interface)
{
    /* Get the total number of blocks on the "disk". */
    uint32_t size = mem[interface].size();

    /* Prep the response and send it followed by sending the CSW */
    req_data[interface][0] = 0x00;
    req_data[interface][1] = 0x00;
    req_data[interface][2] = 0x00;
    req_data[interface][3] = 0x08;

    req_data[interface][4] = (uint8_t)(size >> 24);
    req_data[interface][5] = (uint8_t)(size >> 16);
    req_data[interface][6] = (uint8_t)(size >> 8);
    req_data[interface][7] = (uint8_t)(size);

    req_data[interface][8]  = (uint8_t)((MAX_PACKET_SIZE | FORMATTED_MEDIA) >> 24);
    req_data[interface][9]  = (uint8_t)((MAX_PACKET_SIZE | FORMATTED_MEDIA) >> 16);
    req_data[interface][10] = (uint8_t)((MAX_PACKET_SIZE | FORMATTED_MEDIA) >> 8);
    req_data[interface][11] = (uint8_t)(MAX_PACKET_SIZE | FORMATTED_MEDIA);

    msc_write(ep_to_host[interface], FORMATCAPACITY_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendCapacity(const uint8_t interface)
{
    /* Get the total number of data blocks on the "disk". */
    uint32_t size = mem[interface].size() - 1;

    /* Prep the response and send it followed by sending the CSW */
    req_data[interface][0] = (uint8_t)(size >> 24);
    req_data[interface][1] = (uint8_t)(size >> 16);
    req_data[interface][2] = (uint8_t)(size >> 8);
    req_data[interface][3] = (uint8_t)(size);

    req_data[interface][4] = (uint8_t)((MAX_PACKET_SIZE) >> 24);
    req_data[interface][5] = (uint8_t)((MAX_PACKET_SIZE) >> 16);
    req_data[interface][6] = (uint8_t)((MAX_PACKET_SIZE) >> 8);
    req_data[interface][7] = (uint8_t)(MAX_PACKET_SIZE);

    msc_write(ep_to_host[interface], CAPACITY_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_sendCapacity16(const uint8_t interface)
{
    /* Get the total number of data blocks on the "disk". */
    uint32_t size = mem[interface].size() - 1;

    /* Prep the response and send it followed by sending the CSW */
    memset(&req_data[interface][0], 0, CAPACITY16_LEN);

    req_data[interface][4] = (uint8_t)(size >> 24);
    req_data[interface][5] = (uint8_t)(size >> 16);
    req_data[interface][6] = (uint8_t)(size >> 8);
    req_data[interface][7] = (uint8_t)(size);

    req_data[interface][8]  = (uint8_t)((MAX_PACKET_SIZE) >> 24);
    req_data[interface][9]  = (uint8_t)((MAX_PACKET_SIZE) >> 16);
    req_data[interface][10] = (uint8_t)((MAX_PACKET_SIZE) >> 8);
    req_data[interface][11] = (uint8_t)(MAX_PACKET_SIZE);

    msc_write(ep_to_host[interface], CAPACITY16_LEN, (void *)msc_sendCSW, interface);
}

/******************************************************************************/

static void msc_readMem(const uint8_t interface)
{
    /* Are there more blocks to send to the host? */
    if(numBlocks[interface]) {
        /* Read from the "disk" and place the results in the outgoing buffer. */
        /* Log any errors in the appropriate location of the CSW response. */
        csw_data[interface][CSW_STATUS_IDX] |= mem[interface].read(blockAddr[interface], req_data[interface]);
        blockAddr[interface]++; 
        numBlocks[interface]--;
        /* Send the data to the host and have the USB stack call the function
           again once the data has been sent. */
        msc_write(ep_to_host[interface], MAX_PACKET_SIZE, (void *)msc_readMem, interface);
    } else {
        /* All the requested data has been sent, it is time to send the CSW. */
        msc_sendCSW(interface);
    }
}

/******************************************************************************/

static void msc_writeMem(const uint8_t interface)
{
    /* Are there more blocks to receive from the host? */
    if(numBlocks[interface]) {
        /* Setup the read from host transaction and have the USB stack call
           writeMemBlock once all of the data has been received. */
        msc_read(ep_from_host[interface], MAX_PACKET_SIZE, msc_writeMemBlock, NULL, interface);
    }
    else {
        /* All the requested data has been received, it is time to send the CSW. */
        msc_sendCSW(interface);
    }
}

/******************************************************************************/

static void msc_writeMemBlock(void* cbdata)
{
    uint8_t interface = *((uint8_t *)(cbdata));

    if (!req[interface].error_code) 
    {
        /* Write the received data to the "disk". */
        csw_data[interface][CSW_STATUS_IDX] |= mem[interface].write(blockAddr[interface], req_data[interface]);
        blockAddr[interface]++;
        numBlocks[interface]--;

        /* Go receive the next block of data (in any) */
        msc_writeMem(interface);
    }
    else 
    {
        /* USB error occurred while writing. Fail the command. */
        csw_data[interface][CSW_STATUS_IDX] = CSW_FAILED;
        msc_sendCSW(interface);
    }
}
