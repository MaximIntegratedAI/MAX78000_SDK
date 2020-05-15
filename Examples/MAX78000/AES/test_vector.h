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
******************************************************************************/


/***** Definitions *****/

/* NIST Monte Carlo test CAVS 11.1
 *
 * Start with PLAINTEXT -> AES128(KEY) -> CIPHERTEXT
 * Repeat 999 more times, feeding previous CIPHERTEXT in as next PLAINTEXT
 * Verify last CIPHERTEXT against expected result shown above
 *
 */

char iv[AES_DATA_LEN] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
  };
/*
 * 128-bit Encrypt vector
 *
 * KEY = 13 9a 35 42 2f 1d 61 de 3c 91 78 7f e0 50 7a fd
 * PLAINTEXT = b9 14 5a 76 8b 7d c4 89 a0 96 b5 46 f4 3b 23 1f
 * CIPHERTEXT = d7c3ffac9031238650901e157364c386
 *
 */


const char nist_enc_128_key[MXC_AES_KEY_128_LEN] = {
    0x13, 0x9a, 0x35, 0x42,
    0x2f, 0x1d, 0x61, 0xde,
    0x3c, 0x91, 0x78, 0x7f,
    0xe0, 0x50, 0x7a, 0xfd
};

const char nist_enc_128_pt[AES_DATA_LEN] = {
    0xb9, 0x14, 0x5a, 0x76,
    0x8b, 0x7d, 0xc4, 0x89,
    0xa0, 0x96, 0xb5, 0x46,
    0xf4, 0x3b, 0x23, 0x1f
};


const char nist_enc_128_ct[AES_DATA_LEN] = {
    0xd7, 0xc3, 0xff, 0xac,
    0x90, 0x31, 0x23, 0x86,
    0x50, 0x90, 0x1e, 0x15,
    0x73, 0x64, 0xc3, 0x86
};


/*
 * 128-bit Decrypt vector
 *
 * KEY = 0c60e7bf20ada9baa9e1ddf0d1540726
 * CIPHERTEXT = b08a29b11a500ea3aca42c36675b9785
 * PLAINTEXT = b613b87085fed1bb87f07a574e6d2879
 *
 */

const char nist_dec_128_key[MXC_AES_KEY_128_LEN] = {
    0x0c, 0x60, 0xe7, 0xbf,
    0x20, 0xad, 0xa9, 0xba,
    0xa9, 0xe1, 0xdd, 0xf0,
    0xd1, 0x54, 0x07, 0x26
};


const char nist_dec_128_ct[AES_DATA_LEN] = {
    0xb0, 0x8a, 0x29, 0xb1,
    0x1a, 0x50, 0x0e, 0xa3,
    0xac, 0xa4, 0x2c, 0x36,
    0x67, 0x5b, 0x97, 0x85
};


const char nist_dec_128_pt[AES_DATA_LEN] = {
    0xb6, 0x13, 0xb8, 0x70,
    0x85, 0xfe, 0xd1, 0xbb,
    0x87, 0xf0, 0x7a, 0x57,
    0x4e, 0x6d, 0x28, 0x79
};
/*
 * 192-bit Encrypt vector
 *
 * KEY = b9a63e09e1dfc42e93a90d9bad739e5967aef672eedd5da9
 * PLAINTEXT = 85a1f7a58167b389cddc8a9ff175ee26
 * CIPHERTEXT = ee83d85279e022d2048031abeefbc4a4
 *
 */

const char nist_enc_192_key[MXC_AES_KEY_192_LEN] = {
    0xb9, 0xa6, 0x3e, 0x09,
    0xe1, 0xdf, 0xc4, 0x2e,
    0x93, 0xa9, 0x0d, 0x9b,
    0xad, 0x73, 0x9e, 0x59,
    0x67, 0xae, 0xf6, 0x72,
    0xee, 0xdd, 0x5d, 0xa9
};


const char nist_enc_192_pt[AES_DATA_LEN] = {
    0x85, 0xa1, 0xf7, 0xa5,
    0x81, 0x67, 0xb3, 0x89,
    0xcd, 0xdc, 0x8a, 0x9f,
    0xf1, 0x75, 0xee, 0x26
};


const char nist_enc_192_ct[AES_DATA_LEN] = {
    0xee, 0x83, 0xd8, 0x52,
    0x79, 0xe0, 0x22, 0xd2,
    0x04, 0x80, 0x31, 0xab,
    0xee, 0xfb, 0xc4, 0xa4
};

/*
 * 192-bit Decrypt vector
 *
 * KEY = 4b97585701c03fbebdfa8555024f589f1482c58a00fdd9fd
 * CIPHERTEXT = d0bd0e02ded155e4516be83f42d347a4
 * PLAINTEXT = c7b6581ccc88f7fc26d15d2731e7251b
 *
 */



const char nist_dec_192_key[MXC_AES_KEY_192_LEN] = {
    0x4b, 0x97, 0x58, 0x57,
    0x01, 0xc0, 0x3f, 0xbe,
    0xbd, 0xfa, 0x85, 0x55,
    0x02, 0x4f, 0x58, 0x9f,
    0x14, 0x82, 0xc5, 0x8a,
    0x00, 0xfd, 0xd9, 0xfd
};


const char nist_dec_192_ct[AES_DATA_LEN] = {
    0xd0, 0xbd, 0x0e, 0x02,
    0xde, 0xd1, 0x55, 0xe4,
    0x51, 0x6b, 0xe8, 0x3f,
    0x42, 0xd3, 0x47, 0xa4
};



const char nist_dec_192_pt[AES_DATA_LEN] = {
    0xc7, 0xb6, 0x58, 0x1c,
    0xcc, 0x88, 0xf7, 0xfc,
    0x26, 0xd1, 0x5d, 0x27,
    0x31, 0xe7, 0x25, 0x1b
};


/*
 * 256-bit Encrypt vector
 *
 * KEY = f9e8389f5b80712e3886cc1fa2d28a3b8c9cd88a2d4a54c6aa86ce0fef944be0
 * PLAINTEXT = b379777f9050e2a818f2940cbbd9aba4
 * CIPHERTEXT = 6893ebaf0a1fccc704326529fdfb60db
 *
 */



const char nist_enc_256_key[MXC_AES_KEY_256_LEN] = {
    0xf9, 0xe8, 0x38, 0x9f,
    0x5b, 0x80, 0x71, 0x2e,
    0x38, 0x86, 0xcc, 0x1f,
    0xa2, 0xd2, 0x8a, 0x3b,
    0x8c, 0x9c, 0xd8, 0x8a,
    0x2d, 0x4a, 0x54, 0xc6,
    0xaa, 0x86, 0xce, 0x0f,
    0xef, 0x94, 0x4b, 0xe0
};

const char nist_enc_256_pt[AES_DATA_LEN] = {
    0xb3, 0x79, 0x77, 0x7f,
    0x90, 0x50, 0xe2, 0xa8,
    0x18, 0xf2, 0x94, 0x0c,
    0xbb, 0xd9, 0xab, 0xa4
};

const char nist_enc_256_ct[AES_DATA_LEN] = {
    0x68, 0x93, 0xeb, 0xaf,
    0x0a, 0x1f, 0xcc, 0xc7,
    0x04, 0x32, 0x65, 0x29,
    0xfd, 0xfb, 0x60, 0xdb
};
/*
 * 256-bit Decrypt vector
 *
 * KEY = 2b09ba39b834062b9e93f48373b8dd018dedf1e5ba1b8af831ebbacbc92a2643
 * CIPHERTEXT = 89649bd0115f30bd878567610223a59d
 * PLAINTEXT = 1f9b9b213f1884fa98b62dd6639fd33b
 *
 */


const char nist_dec_256_key[MXC_AES_KEY_256_LEN] = {
    0x2b, 0x09, 0xba, 0x39,
    0xb8, 0x34, 0x06, 0x2b,
    0x9e, 0x93, 0xf4, 0x83,
    0x73, 0xb8, 0xdd, 0x01,
    0x8d, 0xed, 0xf1, 0xe5,
    0xba, 0x1b, 0x8a, 0xf8,
    0x31, 0xeb, 0xba, 0xcb,
    0xc9, 0x2a, 0x26, 0x43
};

const char nist_dec_256_ct[AES_DATA_LEN] = {
    0x89, 0x64, 0x9b, 0xd0,
    0x11, 0x5f, 0x30, 0xbd,
    0x87, 0x85, 0x67, 0x61,
    0x02, 0x23, 0xa5, 0x9d
};

const char nist_dec_256_pt[AES_DATA_LEN] = {
    0x1f, 0x9b, 0x9b, 0x21,
    0x3f, 0x18, 0x84, 0xfa,
    0x98, 0xb6, 0x2d, 0xd6,
    0x63, 0x9f, 0xd3, 0x3b
};
