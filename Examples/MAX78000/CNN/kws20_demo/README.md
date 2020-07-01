---
typora-root-url: Resources
---

# MAX78000 Keyword Spotting Demo



## Overview

The Keyword Spotting Demo software demonstrates recognition of a number of keywords using MAX78000 EVKIT.  

The KWS20 demo software utilizes 2nd version of Google speech commands dataset which consists of 35 keywords and more than 100K utterances:

https://storage.cloud.google.com/download.tensorflow.org/data/speech_commands_v0.02.tar.gz



The following 20 keyword subset from the complete dataset is used for this demo:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'yes', 'no', 'on', 'off', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'zero**']

Rest of keywords and unrecognized words fall into "**Unknown**" category.



## Keyword Spotting Demo Software

### Building firmware:

Navigate directory where KWS20 demo software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/kws20_demo
$ make
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make distclean
```

### Load firmware image to target:

Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1).

Connect PICO adapter to JH5 SWD header. 

Load firmware image using Openocd. **Make sure to remove PICO adapter once firmware is loaded.**

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### Jumper setting:

Make sure to install jumper at JP20-CLK (INT position) as shown bellow:

<img src="/I2S_jumper.png" style="zoom:25%;" />

Note: On board external oscillator Y3 is used to generate I2S clock. The I2S sample rate is 16kHz to match speech samples of the dataset.

### Operations:

After power-cycle,  if the TFT display is blank, or not shown properly as below, please press RESET (SW5).

The TFT display shows that it is ready. Press PB1 to start:

<img src="/20200604_142849.jpg" style="zoom: 25%;" />



Once RED LED2 turns on, the initialization is complete and it is ready to accept keywords. If PICO adapter is still connected to SWD, disconnect it and power cycle.

Following words can be detected:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'yes', 'no', 'on', 'off', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'zero**']

 The MAX78000 EVKIT firmware recognizes keywords and reports result and confidence level.

The microphone (U15) is located between JH4 and JH5 headers.



<img src="/20200604_142536_1.jpg" style="zoom:25%;" />



### Using Debug Terminal

Debug terminal shows more information on status and detected words. 

The USB cable connected to CN1 (USB/PWR) provides power and serial communication to MAX78000 EVKIT.

To configure PC terminal program select correct COM port and settings as follow:

![](/Terminal2.png)

After turning on power switch (SW1) or pressing reset button (SW5)  following message will appear in terminal window:

![](/Terminal1.png)



Terminal display after detecting words:

![](/Terminal3.png)



The software components of KWS20 demo are shown in diagram below:

![](/Diagram.png)



## CNN Model

The KWS20 Convolutional Neural Network (CNN) model consists of two connected networks: **Conv1D** and **Conv2D**. The **1D** CNN with 4 layers is used to extract speech parameters and second **2D** CNN with 5 layers and one fully connected layer to recognize keyword from 20 words dictionary used for training.

```python
class AI85KWS20Net(nn.Module):
    """
    Compound KWS20 Audio net, starting with Conv1Ds with kernel_size=1
    and then switching to Conv2Ds
    """
    # num_classes = n keywords + 1 unknown
    def __init__(
            self,
            num_classes=21,
            num_channels=128,
            dimensions=(128, 1),  # pylint: disable=unused-argument
            fc_inputs=7,
            bias=False
    ):
        super(AI85KWS20Net, self).__init__()

        self.voice_conv1 = ai8x.FusedConv1dReLU(num_channels, 100, 1, stride=1, padding=0,
                                               bias=bias)

        self.voice_conv2 = ai8x.FusedConv1dReLU(100, 100, 1, stride=1, padding=0,
                                               bias=bias)

        self.voice_conv3 = ai8x.FusedConv1dReLU(100, 50, 1, stride=1, padding=0,
                                               bias=bias)

        self.voice_conv4 = ai8x.FusedConv1dReLU(50, 16, 1, stride=1, padding=0,
                                               bias=bias)

        self.kws_conv1 = ai8x.FusedConv2dReLU(16, 32, 3, stride=1, padding=1,
                                              bias=bias)

        self.kws_conv2 = ai8x.FusedConv2dReLU(32, 64, 3, stride=1, padding=1,
                                              bias=bias)

        self.kws_conv3 = ai8x.FusedConv2dReLU(64, 64, 3, stride=1,
                                              padding=1, bias=bias)

        self.kws_conv4 = ai8x.FusedConv2dReLU(64, 30, 3, stride=1,
                                              padding=1, bias=bias)

        self.kws_conv5 = ai8x.FusedConv2dReLU(30, fc_inputs, 3, stride=1,
                                              padding=1, bias=bias)

        self.fc = ai8x.Linear(fc_inputs*128, num_classes, bias=bias)

        for m in self.modules():
            if isinstance(m, nn.Conv2d):
                nn.init.kaiming_normal_(m.weight, mode='fan_out', nonlinearity='relu')

    def forward(self, x):  # pylint: disable=arguments-differ
        # Run CNN
        x = self.voice_conv1(x)
        x = self.voice_conv2(x)
        x = self.voice_conv3(x)
        x = self.voice_conv4(x)
        x = x.view(x.shape[0], x.shape[1], 16, -1)
        x = self.kws_conv1(x)
        x = self.kws_conv2(x)
        x = self.kws_conv3(x)
        x = self.kws_conv4(x)
        x = self.kws_conv5(x)
        x = x.view(x.size(0), -1)
        x = self.fc(x)

        return x

```

The CNN input is 128x128=16384 8-bit signed speech samples.

## Network Training

To invoke network training execute the script:

```bash
(ai8x-training) $ ./train_kws20.sh
```

If this is the first time, and the dataset does not exist locally, the scrip will automatically download Google speech commands dataset (1 second keyword .wav files , sampled at 16KHz, 16-bit) into /data/KWS/raw, and process it to make appropriate training, test and validation dataset integrated in /data/KWS/process/dataset.pt. The processing step expands training dataset by using augmentation techniques like adding white noise, random time shift and stretch to improve training results. In addition, each 16000 sample word example is padded with zeros to make it 128x128=16384 speech samples. The augmentation process triples the size of dataset and could take 30min to complete.

Details of network training methodology are described in [AI8X Model Training and Quantization](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)

After training unquantized network can be evaluated by executing script:

```bash
(ai8x-training) $ ./evaluate_kws20.sh
```



## Network Quantization

The CNN weights generated during training need to be quantized:

```bash
(ai8x-synthesis) $ ./quantize_kws20.sh
```



The weight scale parameter 0.97 is selected to achieve best performance for 8-bit quantization.

![](/CalibrationScale.png)



Details of quantization are described in [AI8X Model Training and Quantization](https://github.com/MaximIntegratedAI/ai8x-synthesis/blob/master/README.md)

Effect of quantization is shown in confusion matrix below.

​    [   up       down     left     right    stop      go         yes     no       on       off         one       two     three    four     five       six     seven   eight    nine   zero  unknown]

![](/image2020-6-2_13-44-21.png)



## Network Synthesis

The network synthesis script generates a pass/fail C example code which includes necessary functions to initialize MAX78000 CNN accelerator, to load quantized CNN weights and input samples and to unload classification results. A sample input with the expected result is part of this automatically generated code to verify.  Following script generates all example projects including ai85-kws20:

```bash
(ai8x-synthesis) $ ./gen-demos-max78000.sh
```

The ai85-kws20 bare-bone C code is partially used in KWS20 Demo. In particular, CNN initialization, weights (kernels) and helper functions to load/unload weights and samples are ported from ai85-kws20 to KWS20 Demo.



## KWS20 Demo Code

KWS20 demo works in two modes:  Using microphone (real-time), or offline processing:

```c
#define ENABLE_MIC_PROCESSINGc
```

### Microphone Mode

In this mode, EVKIT I2S Mic is initialized to operate at 16KHz 32-bit samples.  In the main loop, I2S buffer is checked and sampled are stored into  **pChunkBuff** buffer.  

### Offline Mode

if **ENABLE_MIC_PROCESSING** is not defined, a header file containing the 16-bit samples (e.g. **kws_five.h**) should be included in the project to be used as the input . To create a header file from a wav file, use included utilities to record a wav file and convert it to header file. 

```bash
# record 3sec of 16-bit 16KHz sampled wav file 
$ python VoiceRecorder.py -d 3 -o voicefile.wav
# convert to header
$ python RealtimeAudio.py -i voicefile.wav -o voicefile.h
```

### KWS20 Demo Firmware Structure

Following figure shows the processing in KWS20 Demo firmware:

![](/KWS_Demo_flowchart.png)

Collected samples from mic/file are 18/16 bit signed and are converted to 8 bit signed to feed into CNN. If Microphone mode, a high pass filter is used to filter out the DC level in captured samples. Scaled samples are stored in **pPreambleCircBuffer** circular buffer in chunks of 128 samples (bytes). 

Following parameters in the firmware can be tuned:

```c
#define SAMPLE_SCALE_FACTOR    		4		// multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HGIH				350  	// voice detection threshold to find beginning of a keyword
#define THRESHOLD_LOW				100  	// voice detection threshold to find end of a keyword
#define SILENCE_COUNTER_THRESHOLD 	20 		// [>20] number of back to back CHUNK periods with avg < THRESHOLD_LOW to declare the end of a word
#define PREAMBLE_SIZE				30*CHUNK// how many samples before beginning of a keyword to include
#define INFERENCE_THRESHOLD   		49 		// min probability (0-100) to accept an inference
```

When the average absolute values of samples during last 128 number of samples goes above a threshold, the beginning of a word is marked. 

The end of a word is signaled when the **SILENCE_COUNTER_THRESHOLD** back to back chunks of samples with average absolute threshold lower than **THRESHOLD_LOW** is observed. 

The CNN requires 1sec worth of samples (128*128) to start processing. This window starts at **PREAMBLE_SIZE** samples prior to the beginning of the word, and ends after 16384 samples. If the end of a word is determined earlier, the pAI85Buffer sample buffer is padded with zeros.

The CNN related API functions are in **cnn.c**. They are used to load weights and data, start CNN, wait for CNN to complete processing and unload the result. 

If a new network is developed and synthesized, the new weight file and related API functions are needed to be ported from automatically generated ai85-kws20 example project. Furthermore, if the input layer or organization of 128x128 sample sets in the trained network is changed, **AddTranspose()** function should be changed to reflect the new sample data arrangement in CNN memory.