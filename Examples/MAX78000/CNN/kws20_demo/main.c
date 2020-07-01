/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
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
*******************************************************************************/

/**
 * @file    main.c
 * @brief   Main for KWS20
 * @details 
 *         
 *          
 */

/* **** Includes **** */
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "mxc_sys.h"
#include "bbfc_regs.h"
#include "fcr_regs.h"
#include "icc.h"
#include "mxc_device.h"
#include "mxc_delay.h"
#include "nvic_table.h"
#include "i2s_regs.h"
#include "board.h"
#include "i2s.h"
#include "tmr.h"
#include "dma.h"
#include "led.h"
#include "pb.h"
#include "cnn.h"
#include "bitmap.h"
#include "tft.h"
#include "mxc.h"

#define VERSION   "1.0.3 (6/29/20)"
/* **** Definitions **** */

/* Enable/Disable Features */
#define ENABLE_TFT						 // enables TFT display
#define ENABLE_PRINT_ENVELOPE            // enables printing average waveform envelope for samples
//#define ENABLE_CLASSIFICATION_DISPLAY	 // enables printing classification result
#define ENABLE_SILENCE_DETECTION		 // Starts collecting only after avg > THRESHOLD_HIGH, otherwise starts from first sample
#undef EIGHT_BIT_SAMPLES			     // samples from Mic or Test vectors are eight bit, otherwise 16-bit
#define ENABLE_MIC_PROCESSING			 // enables capturing Mic, otherwise a header file Test vector is used as sample data

#ifndef ENABLE_MIC_PROCESSING
#include "kws_five.h"
#else
#undef ENABLE_PRINT_ENVELOPE  // print is slow with live mic data
#endif

/*-----------------------------*/
/* keep following unchanged */
#define SAMPLE_SIZE 		16384 	// size of input vector for CNN, keep it multiple of 128
#define CHUNK				128  	// number of data points to read at a time and average for threshold, keep multiple of 128
#define TRANSPOSE_WIDTH		128 	// width of 2d data model to be used for transpose
#define NUM_OUTPUTS 		21		// number of classes
#define I2S_RX_BUFFER_SIZE	64		// I2S buffer size
#define TFT_BUFF_SIZE		50		// TFT buffer size
/*-----------------------------*/

/* Adjustables */
#ifdef ENABLE_MIC_PROCESSING
#define SAMPLE_SCALE_FACTOR    		4		// multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH				350  	// voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW				100  	// voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD 	20 		// [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE				30*CHUNK// how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD   		49 		// min probability (0-100) to accept an inference
#else
#define SAMPLE_SCALE_FACTOR    		1		// multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH				130  	// voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW				70  	// voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD 	20 		// [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE				30*CHUNK// how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD   		49 		// min probability (0-100) to accept an inference
#endif

/* **** Globals **** */
extern uint32_t cnn_time;
static int32_t ml_data[NUM_OUTPUTS];
static q15_t ml_softmax[NUM_OUTPUTS];
uint8_t pAI85Buffer[SAMPLE_SIZE];
uint8_t pPreambleCircBuffer[PREAMBLE_SIZE];
int16_t Max, Min;
uint16_t thresholdHigh = THRESHOLD_HIGH;
uint16_t thresholdLow = THRESHOLD_LOW;

volatile uint8_t i2s_flag = 0;
int32_t i2s_rx_buffer[I2S_RX_BUFFER_SIZE];

/* **** Constants **** */
typedef enum _mic_processing_state {
	STOP = 0,     /* No processing  */
	SILENCE = 1,  /* Threshold not detected yet  */
	KEYWORD = 2   /* Threshold has been detected, gathering keyword samples */
} mic_processing_state;

/* Set of detected words */
const char keywords[NUM_OUTPUTS][10] = { "UP", "DOWN", "LEFT", "RIGHT", "STOP",
		"GO", "YES", "NO", "ON", "OFF", "ONE", "TWO", "THREE", "FOUR", "FIVE",
		"SIX", "SEVEN", "EIGHT", "NINE", "ZERO", "Unknown" };

#ifndef ENABLE_MIC_PROCESSING

#ifndef EIGHT_BIT_SAMPLES
const int16_t voiceVector[] = KWS20_TEST_VECTOR;
#else
const int8_t voiceVector[] = KWS20_TEST_VECTOR;
#endif

#else
void i2s_isr(void)
{
    i2s_flag = 1;
    /* Clear I2S interrupt flag */
    MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_THD_CH0);
}
#endif


/* **** Functions Prototypes **** */
int8_t MicReader(int16_t *sample);
uint8_t MicReadChunk(uint8_t *pBuff, uint16_t * avg);
uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize,
		uint16_t outSize, uint16_t width);
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data,
		int16_t *out_class, double *out_prob);
void I2SInit();
void HPF_init(void);
int16_t HPF(int16_t input);
#ifdef ENABLE_TFT
void TFT_Intro(void);
void TFT_Print(char *str, int x, int y, int font);
void TFT_End(uint16_t words);
#endif

/* **************************************************************************** */

int main(void)
{
    uint32_t sampleCounter = 0;
    mxc_tmr_unit_t units;

	uint8_t pChunkBuff[CHUNK];

	uint16_t avg = 0;
	uint16_t preambleCounter = 0;
	uint16_t ai85Counter = 0;
	uint16_t wordCounter = 0;

	uint16_t avgSilenceCounter = 0;

	mic_processing_state procState = STOP;

	/* Enable cache */
	MXC_ICC_Enable(MXC_ICC0);

	/* Switch to 100 MHz clock */
	MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
	SystemCoreClockUpdate();

	/* Reset all domains, restore power to CNN */
	MXC_BBFC->reg3 = 0xf; // Reset
	MXC_BBFC->reg1 = 0xf; // Mask
	MXC_BBFC->reg0 = 0xf; // Power
	MXC_BBFC->reg2 = 0x0; // Iso
	MXC_BBFC->reg3 = 0x0; // Reset

	MXC_GCR->pclkdiv &= ~(MXC_F_GCR_PCLKDIV_CNNCLKDIV
			| MXC_F_GCR_PCLKDIV_CNNCLKSEL);
	MXC_GCR->pclkdiv |= MXC_S_GCR_PCLKDIV_CNNCLKDIV_DIV1; // CNN clock: 100 MHz div 2
	MXC_SYS_ClockEnable(MXC_SYS_PERIPH_CLOCK_CNN); // Enable CNN clock

	/* Configure P2.5, turn on the CNN Boost */
	mxc_gpio_cfg_t gpio_out;
	gpio_out.port = MXC_GPIO2;
	gpio_out.mask = MXC_GPIO_PIN_5;
	gpio_out.pad = MXC_GPIO_PAD_NONE;
	gpio_out.func = MXC_GPIO_FUNC_OUT;
	MXC_GPIO_Config(&gpio_out);
	MXC_GPIO_OutSet(gpio_out.port, gpio_out.mask);

	printf("Maxim Integrated \nKeyword Spotting Demo\nVer. %s \n",VERSION);
	printf("\n***** Init *****\n");
	memset(pAI85Buffer, 0x0, sizeof(pAI85Buffer));
	memset(pPreambleCircBuffer, 0x0, sizeof(pPreambleCircBuffer));

	printf("pChunkBuff: %d\n", sizeof(pChunkBuff));
	printf("pPreambleCircBuffer: %d\n", sizeof(pPreambleCircBuffer));
	printf("pAI85Buffer: %d\n", sizeof(pAI85Buffer));

	/* load kernels */
	printf("\n*** CNN Kernel load ***\n");
	if (!cnn_load_kernel()) {
		printf("ERROR: Loading Kernel! \n");
		fail();
	}
	/* switch to silence state*/
	procState = SILENCE;

#ifdef ENABLE_MIC_PROCESSING
	/* initialize I2S interface to Mic */
	I2SInit();
#endif

#ifdef ENABLE_TFT

	/* TFT reset signal */
    mxc_gpio_cfg_t tft_reset_pin = {MXC_GPIO0, MXC_GPIO_PIN_19, MXC_GPIO_FUNC_OUT, MXC_GPIO_PAD_NONE, MXC_GPIO_VSSEL_VDDIOH};
	MXC_Delay(500000);
	printf("\n*** Init TFT ***\n");
    MXC_TFT_Init(MXC_SPI0, 1, &tft_reset_pin, NULL);
	MXC_TFT_ClearScreen();

	MXC_TFT_ShowImage(0, 0, img_1_bmp);
	MXC_Delay(1000000);

	MXC_TFT_SetPalette(logo_white_bg_darkgrey_bmp);
	MXC_TFT_SetBackGroundColor(4);

	printf("Waiting for PB1 press\n");
	TFT_Intro();
    printf("if RED LED is not on, disconnect PICO SWD and powercycle!\n");
#endif

	printf("\n*** READY ***\n");
    /* Read samples */
	while (1) {
#ifndef ENABLE_MIC_PROCESSING
		/* end of test vectors */
		if(sampleCounter>= sizeof(voiceVector) / sizeof(voiceVector[0]))
		{
			printf("End of test Vector\n");
			break;
		}
#endif

		/* Read from Mic driver to get CHUNK worth of samples, otherwise next sample*/
		if (MicReadChunk(pChunkBuff, &avg) == 0) {
			continue;
		}

		sampleCounter += CHUNK;

#ifdef ENABLE_SILENCE_DETECTION       // disable to start collecting data immediately.

		/* copy the preamble data*/
		/* add the new chunk to the end of circular buffer*/
		memcpy(&pPreambleCircBuffer[preambleCounter], pChunkBuff,
				sizeof(uint8_t) * CHUNK);

		/* increment circular buffer pointer*/
		preambleCounter = (preambleCounter + CHUNK) % (PREAMBLE_SIZE);

		/* Display average envelope as a bar */
#ifdef ENABLE_PRINT_ENVELOPE
		printf("%.6d|",sampleCounter);
		for (int i = 0; i < avg / 10; i++)
			printf("=");
		if (avg >= thresholdHigh)
			printf("*");
		printf("[%d]\n",avg);
#endif

		/* if we have not detected voice, check the average*/
		if (procState == SILENCE) {

			/* compute average, proceed if greater than threshold */
			if (avg >= thresholdHigh) {
				/* switch to keyword data collection*/
				procState = KEYWORD;
				printf("%.6d Word starts from index: %d, avg:%d > %d \n",
						sampleCounter, sampleCounter - PREAMBLE_SIZE - CHUNK,
						avg, thresholdHigh);

				/* reorder circular buffer according to time at the beginning of pAI85Buffer */
				if (preambleCounter == 0) {
					/* copy latest samples afterwards */
					if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer,
					PREAMBLE_SIZE, SAMPLE_SIZE, TRANSPOSE_WIDTH))
						printf("ERROR: Transpose ended early \n");
				} else {
					/* copy oldest samples to the beginning*/
					if (AddTranspose(&pPreambleCircBuffer[preambleCounter],
							pAI85Buffer, PREAMBLE_SIZE - preambleCounter,
							SAMPLE_SIZE, TRANSPOSE_WIDTH))
						printf("ERROR: Transpose ended early \n");

					/* copy latest samples afterwards */
					if (AddTranspose(&pPreambleCircBuffer[0], pAI85Buffer,
							preambleCounter, SAMPLE_SIZE, TRANSPOSE_WIDTH))
						printf("ERROR: Transpose ended early \n");

				}
				
				/* preamble is copied and state is changed, start adding keyword samples next run */
				ai85Counter += PREAMBLE_SIZE;
				continue;
				
			}
		}
		/* if it is in data collection, add samples to buffer*/
		else if (procState == KEYWORD)
#endif  //#ifdef ENABLE_SILENCE_DETECTION

		{
			uint8_t ret=0;

			/* add sample, rearrange buffer */
			ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK, SAMPLE_SIZE,
			TRANSPOSE_WIDTH);

			/* increment number of stored samples */
			ai85Counter += CHUNK;
			
			/* if there is silence after at least 1/3 of samples passed, increment number of times back to back silence to find end of keyword */
			if ((avg < thresholdLow) && (ai85Counter >= SAMPLE_SIZE / 3)) {
				avgSilenceCounter ++;
			} else
				avgSilenceCounter = 0;


			/* if this is the last sample and there are not enough samples to
			 * feed to CNN, or if it is long silence after keyword,  append with zero (for reading file)
			 */
#ifndef ENABLE_MIC_PROCESSING
			if (((ai85Counter < SAMPLE_SIZE) && (sampleCounter>= sizeof(voiceVector)/sizeof(voiceVector[0])-1)) || (avgSilenceCounter > SILENCE_COUNTER_THRESHOLD))
#else
			if (avgSilenceCounter > SILENCE_COUNTER_THRESHOLD)
#endif
			{
				memset(pChunkBuff,0,CHUNK);
				printf("%.6d: Word ends, Appends %d zeros \n", sampleCounter,
				SAMPLE_SIZE - ai85Counter);
				ret = 0;
				while (!ret) {
					ret = AddTranspose(pChunkBuff, pAI85Buffer, CHUNK,
					SAMPLE_SIZE, TRANSPOSE_WIDTH);
					ai85Counter += CHUNK;
				}
			}

			/* if enough samples are collected, start CNN */
			if (ai85Counter >= SAMPLE_SIZE) {
				int16_t out_class = -1;
				double probability = 0;

				/* reset counters */
				ai85Counter = 0;
				avgSilenceCounter = 0;

				/* new word */
				wordCounter++;

				/* change state to silence */
				procState = SILENCE;

				/* sanity check, last transpose should have returned 1, as enough samples should have already been added */
				if (ret != 1) {
					printf("ERROR: Transpose incomplete!\n");
					fail();
				}

				//----------------------------------  : invoke AI85 CNN
				printf("%.6d: Starts CNN: %d\n", sampleCounter, wordCounter);

				/* load to CNN */
				if (!cnn_load_data(pAI85Buffer)) {
					printf("ERROR: Loading data to CNN! \n");
					fail();
				}

				/* Start timer */
				MXC_TMR_SW_Start(MXC_TMR0);

				/* Start CNN */
				if (!cnn_start()) {
					printf("ERROR: Starting CNN! \n");
					fail();
				}

				/* Wait for CNN  to complete */
				cnn_wait();

				/* read data */
				cnn_unload((uint32_t *)ml_data);

				/* Get time */
				MXC_TMR_GetTime(MXC_TMR0, cnn_time, &cnn_time, &units);
				printf("%.6d: Completes CNN: %d\n", sampleCounter, wordCounter);

				switch (units) {
					case TMR_UNIT_NANOSEC:
					cnn_time /= 1000;
					break;
					case TMR_UNIT_MILLISEC:
					cnn_time *= 1000;
					break;
					case TMR_UNIT_SEC:
					cnn_time *= 1000000;
					break;
					default:
						break;
				}
				printf("CNN Time: %d us\n", cnn_time);

				/* run softmax */
				softmax_q17p14_q15((const q31_t*) ml_data, NUM_OUTPUTS,
						ml_softmax);

#ifdef ENABLE_CLASSIFICATION_DISPLAY
				printf("\nClassification results:\n");
				for (int i = 0; i < NUM_OUTPUTS; i++) {
				    int digs = (1000 * ml_softmax[i] + 0x4000) >> 15;
				    int tens = digs % 10;
				    digs = digs/10;

					printf("[%+.7d] -> Class %.2d %8s: %d.%d%%\n", ml_data[i],
							i, keywords[i], digs, tens);
				}
#endif
				/* find detected class with max probability */
				ret = check_inference(ml_softmax, ml_data, &out_class, &probability);

				printf("----------------------------------------- \n");
				if (!ret)
					printf("LOW CONFIDENCE!: ");

				printf("Detected word: %s (%0.1f%%)", keywords[out_class],
						probability);

				printf("\n----------------------------------------- \n");

				Max = 0;
				Min = 0;
				//------------------------------------------------------------
			}
		}

		/* Stop demo if PB1 is pushed */
		if (PB_Get(0)) {
        	printf("Stop! \r\n");

        	procState = STOP;

        	break;
        }
    }

	/* Turn off LED2 (Red) */
	LED_Off(LED2);
	printf("Total Samples:%d, Total Words: %d \n", sampleCounter, wordCounter);

#ifdef ENABLE_TFT
	TFT_End(wordCounter);
#endif
	while (1)
		;
}

/* **************************************************************************** */

#ifdef ENABLE_MIC_PROCESSING
void I2SInit()
{
	mxc_i2s_req_t req;
	int32_t err;

    printf("\n*** I2S & Mic Init ***\n");
    /* Initialize High Pass Filter */
    HPF_init();
    /* Initialize I2S RX buffer */
    memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
    /* Configure I2S interface parameters */
    req.wordSize    = MXC_I2S_DATASIZE_WORD;
    req.sampleSize  = MXC_I2S_SAMPLESIZE_THIRTYTWO;
    req.justify     = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity	= MXC_I2S_POL_NORMAL;
    req.channelMode	= MXC_I2S_INTERNAL_SCK_WS_0;
    /* Get only left channel data from on-board microphone. Right channel samples are zeros */
    req.stereoMode	= MXC_I2S_MONO_LEFT_CH;
    req.bitOrder	= MXC_I2S_MSB_FIRST;
    /* I2S clock = PT freq / (2*(req.clkdiv + 1)) */
    /* I2S sample rate = I2S clock/64 = 16kHz */
    req.clkdiv      = 5;
    req.rawData     = NULL;
    req.txData      = NULL;
    req.rxData      = i2s_rx_buffer;
    req.length      = I2S_RX_BUFFER_SIZE;


    if((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
        printf("\nError in I2S_Init: %d\n", err);
		while (1)
			;
    }

    /* Set I2S RX FIFO threshold to generate interrupt */
    MXC_I2S_SetRXThreshold(4);
    NVIC_SetVector(I2S_IRQn, i2s_isr);
    NVIC_EnableIRQ(I2S_IRQn);
    /* Enable RX FIFO Threshold Interrupt */
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
    MXC_I2S_RXEnable();
    __enable_irq();
}
#endif

/* **************************************************************************** */
uint8_t check_inference(q15_t *ml_soft, int32_t *ml_data,
		int16_t *out_class, double *out_prob) {

	char buff[TFT_BUFF_SIZE];
	int32_t temp[NUM_OUTPUTS];
	q15_t max = 0;    // soft_max output is 0->32767
	int32_t max_ml = 1<<31;  // ml before going to soft_max
	int16_t max_index = -1;

	memcpy(temp, ml_data, sizeof(int32_t) * NUM_OUTPUTS);

	/* find the top 5 classes */
	for (int top = 0; top < 5; top++) {
		/* find the class with highest */
		for (int i = 0; i < NUM_OUTPUTS; i++) {
			if ((int32_t) temp[i] > max_ml) {
				max_ml = (int32_t) temp[i];

			max = ml_soft[i];
			max_index = i;
		}
		}

		/* print top 1 separately */
		if (top == 0) {

			*out_class = max_index;
			*out_prob = 100.0 * max / 32768.0;
#ifndef ENABLE_TFT
			break;
		}

#else

		    MXC_TFT_ClearScreen();
		    memset(buff,32,TFT_BUFF_SIZE);
		    sprintf(buff, "%s (%0.1f%%)                  ", keywords[max_index],
					(double) 100.0 * max / 32768.0);
			TFT_Print(buff, 20, 30, urw_gothic_13_white_bg_grey);
			sprintf(buff, "__________________________    ");
			TFT_Print(buff, 1, 50, urw_gothic_12_white_bg_grey);

			sprintf(buff, "Top classes:                  ");
			TFT_Print(buff, 1, 80, urw_gothic_12_white_bg_grey);
		} else {
			sprintf(buff, "%s (%0.1f%%)                  ", keywords[max_index],
					(double) 100.0 * max / 32768.0);
			TFT_Print(buff, 20, 80 + 20 * top, urw_gothic_12_white_bg_grey);
		}

		/* reset for next top */
		temp[max_index] = 1 << 31;
		max_ml = 1 << 31;
		max_index = -1;

		if (top == 4)
		{
			sprintf(buff, "Sample Min: %d     Max: %d              ", Min, Max);
			TFT_Print(buff, 20, 200, urw_gothic_12_white_bg_grey);
		}
#endif
	}

    printf("Min: %d,   Max: %d \n", Min, Max);

	/* check if probability is low */
	if (*out_prob > INFERENCE_THRESHOLD)
		return 1;
	else
		return 0;
}

/* **************************************************************************** */

uint8_t AddTranspose(uint8_t *pIn, uint8_t *pOut, uint16_t inSize,
		uint16_t outSize, uint16_t width) {
	/* Data order in Ai85 memory (transpose is included):
	input(series of 8 bit samples): (0,0) ...  (0,127)  (1,0) ... (1,127) ...... (127,0)...(127,127)    16384 samples
	output (32bit word): 16K samples in a buffer. Later, each 1K goes to a seperate CNN memory group
	0x0000:
		(0,3)(0,2)(0,1)(0,0)
		(0,67)(0,66)(0,65)(0,64)
		(1,3)(1,2)(1,1)(1,0)
		(1,67)(1,66)(1,65)(1,64)
		....
		(127,67)(127,66)(127,65)(127,64)
	0x0400:
		(0,7)(0,6)(0,5)(0,4)
		(0,71)(0,70)(0,69)(0,68)
		....
		(127,71)(127,70)(127,69)(127,68)
	...
	0x3C00:
		(0,63)(0,62)(0,61)(0,60)
		(0,127)(0,126)(0,125)(0,124)
		....
		(127,127)(127,126)(127,125)(127,124)
	*/

	static uint16_t row = 0, col = 0, total = 0;
	uint16_t  secondHalf = 0, wordRow = 0, byteInWord = 0, group = 0, index = 0;
	for (int i = 0; i < inSize; i++) {
		/* is it above 63? */
		if (col >= (width >> 1)) {
			secondHalf = 1; // odd word rows
		} else {
			secondHalf = 0; // even word rows
		}

		/* which group (0-15) it should be */
		group = (col % (width >> 1)) / 4;

		/* which word row (0-255) within the group */
		wordRow = secondHalf + (row << 1);

		/* which byte (0-3) in the word */
		byteInWord = col % 4;

		/* find output index */
		index = 1024 * group + 4 * wordRow + byteInWord;

		/* place sample in correct output location */
		pOut[index] = pIn[i];

		total++;

		/* increment row and col index */
		col++;
		if (col >= width) {
			col = 0;
			row++;
		}
	}

	if (total >= outSize) {
		/* sanity check */
		if (row != width)
			printf("ERROR: Rearranging!\n");

		total = 0;
		row = 0;
		col = 0;
		return 1;
	} else
		return 0;

}
/* **************************************************************************** */
#ifndef ENABLE_MIC_PROCESSING
uint8_t MicReadChunk(uint8_t *pBuff, uint16_t * avg)
{
	static uint16_t chunkCount = 0;
	static uint16_t sum = 0;

	int16_t sample = 0;
	int16_t temp = 0;
	uint8_t ret = 0;

	/* Read one sample from mic emulated by test vector and add to buffer*/
	ret = MicReader(&sample);

	/* sample not ready */
	if (!ret)
	{
		*avg = 0;
		return 0;
	}
	/* Turn on LED2 (Red) */
	LED_On(LED2);

	/* absolute for averaging */
	if (sample >= 0)
		sum += sample;
	else
		sum -= sample;

	/* convert to 8 bit unsigned */
#ifndef EIGHT_BIT_SAMPLES
	pBuff[chunkCount++] = (uint8_t)((sample)*SAMPLE_SCALE_FACTOR/256);
#else
	pBuff[chunkCount++] = (uint8_t)((sample)*SAMPLE_SCALE_FACTOR/256);
#endif

	temp=(int8_t)pBuff[chunkCount-1];

	/* record max and min */
	if (temp > Max)
		Max = temp;
	if (temp < Min)
		Min = temp;

	/* if not enough samples, return 0 */
	if (chunkCount < CHUNK)
	{
		*avg = 0;
		return 0;
	}
	
#ifdef EIGHT_BIT_SAMPLES
	/* scale up sum in 8-bit case to work with same threshold as 16bit */
	sum = sum * 256;
#endif

	/* enough samples are collected, calculate average and return 1 */
	*avg = ((uint16_t)(sum / CHUNK));

	chunkCount = 0;
	sum = 0;
	return 1;
}
/* **************************************************************************** */
int8_t MicReader(int16_t *sample)
{
	static uint32_t micSampleCount = 0;

	int16_t temp;

	/* reads from Test Vector file and return one sample */
	temp = voiceVector[(micSampleCount++) % KWS20_TEST_VECTOR_SIZE];
	*sample = temp;
	return (1);
}
#else
/* **************************************************************************** */
uint8_t MicReadChunk(uint8_t *pBuff, uint16_t * avg)
{
	static uint16_t chunkCount = 0;
	static uint16_t sum = 0;

	static uint32_t index = 0;

	int32_t sample = 0;
	int16_t temp = 0;
	uint32_t rx_size = 0;

	/* sample not ready */
	if (!i2s_flag) {
		*avg = 0;
		return 0;
	}

	/* Clear flag */
	i2s_flag = 0;
	/* Read number of samples in I2S RX FIFO */
	rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;
//	printf("%d ", rx_size);

	/* read until fifo is empty or enough samples are collected */
	while ((rx_size--) && (chunkCount < CHUNK)) {
		/* Read microphone sample from I2S FIFO */
		sample = (int32_t)MXC_I2S->fifoch0;
		/* The actual value is 18 MSB of 32-bit word */
		temp = sample >> 14;

		/* Remove DC from microphone signal */
		sample = HPF((int16_t)temp); // filter needs about 1K sample to converge

		/* Discard first 10k samples due to microphone charging cap effect */
		if (index++ < 10000)
			continue;

		/* Turn on LED2 (Red) */
		LED_On(LED2);

		/* absolute for averaging */
		if (sample >= 0)
			sum += sample;
		else
			sum -= sample;

		/* Convert to 8 bit unsigned */
		pBuff[chunkCount] = (uint8_t)((sample)*SAMPLE_SCALE_FACTOR/256);

		temp=(int8_t)pBuff[chunkCount];

		chunkCount++;

		/* record max and min */
		if (temp > Max)
			Max = temp;
		if (temp < Min)
			Min = temp;

	}

	/* if not enough samples, return 0 */
	if (chunkCount < CHUNK) {
		*avg = 0;
		return 0;
	}

	/* enough samples are collected, calculate average and return 1 */
	*avg = ((uint16_t)(sum / CHUNK));

	chunkCount = 0;
	sum = 0;
	return 1;
}

static int16_t	x0, x1, Coeff;
static int32_t	y0, y1;

/************************************************************************************/
void HPF_init(void) {
 	Coeff = 32604; //0.995
	x0 = 0;
	y0 = 0;
	y1 = y0;
	x1 = x0;
}

/************************************************************************************/
int16_t HPF(int16_t input) {
	int16_t Acc, output;
	int32_t tmp;

	/* a 1st order IIR high pass filter (100 Hz cutoff frequency)  */
	/* y(n)=x(n)-x(n-1)+A*y(n-1) and A =.995*2^15 */

	x0=input;

	tmp = (Coeff * y1);
	Acc = (int16_t)((tmp + (1 << 14)) >> 15);
	y0 = x0 - x1 + Acc;

	/* Clipping */
	if(y0 > 32767)
		y0 = 32767;
	if(y0 < -32768)
		y0 = -32768;

	/* Update filter state */
	y1 = y0;
	x1 = x0;

	output = (int16_t)y0;

	return	(output);
}

#endif
/************************************************************************************/
#ifdef ENABLE_TFT

void TFT_Intro(void) {

	char buff[TFT_BUFF_SIZE];
	memset(buff,32,TFT_BUFF_SIZE);
	sprintf(buff, "MAXIM INTEGRATED             ");
	TFT_Print(buff, 55, 10, urw_gothic_13_white_bg_grey);

	sprintf(buff, "Keyword Spotting Demo        ");
	TFT_Print(buff, 35, 40, urw_gothic_12_white_bg_grey);

	sprintf(buff, "Ver. %s                      ", VERSION);
	TFT_Print(buff, 70, 70, urw_gothic_12_white_bg_grey);

	/*             01234567890123456789012345678"*/
	sprintf(buff, "Following keywords can be    ");
	TFT_Print(buff, 5, 110, urw_gothic_12_white_bg_grey);

	sprintf(buff, "detected:                    ");
	TFT_Print(buff, 5, 135, urw_gothic_12_white_bg_grey);

	sprintf(buff, "0...9, up, down, left, right ");
	TFT_Print(buff, 35, 160, urw_gothic_12_white_bg_grey);

	sprintf(buff, "stop, go, yes, no, on, off   ");
	TFT_Print(buff, 35, 185, urw_gothic_12_white_bg_grey);

	sprintf(buff, "PRESS PB1 TO START!          ");
	TFT_Print(buff, 55, 210, urw_gothic_13_white_bg_grey);

	while (!PB_Get(0));

	MXC_TFT_ClearScreen();
	sprintf(buff, "Wait for RED LED to turn on   ");
	TFT_Print(buff, 20, 20, urw_gothic_12_white_bg_grey);
	sprintf(buff, "and start saying keywords...  ");
	TFT_Print(buff, 20, 50, urw_gothic_12_white_bg_grey);

	sprintf(buff, "If RED LED didn't turn on in  ");
	TFT_Print(buff, 20, 110, urw_gothic_12_white_bg_grey);

	sprintf(buff, "2 sec, disconnect SWD and       ");
	TFT_Print(buff, 20, 140, urw_gothic_12_white_bg_grey);

	sprintf(buff, "power cycle.                    ");
	TFT_Print(buff, 20, 170, urw_gothic_12_white_bg_grey);
}

/* **************************************************************************** */
void TFT_Print(char *str, int x, int y, int font) {
	// fonts id
	text_t text;
	text.data = str;
	text.len = 36;

	MXC_TFT_PrintFont(x, y, font, &text, NULL);
}

/* **************************************************************************** */
void TFT_End(uint16_t words) {
	char buff[TFT_BUFF_SIZE];
	memset(buff,32,TFT_BUFF_SIZE);
	MXC_TFT_ClearScreen();
	sprintf(buff, "Demo Stopped!                    ");
	TFT_Print(buff, 70, 30, urw_gothic_13_white_bg_grey);

	sprintf(buff, "Number of words: %d              ", words);
	TFT_Print(buff, 10, 60, urw_gothic_12_white_bg_grey);

	sprintf(buff, "PRESS RESET TO TRY AGAIN!        ");
	TFT_Print(buff, 20, 180, urw_gothic_12_white_bg_grey);

}
#endif
