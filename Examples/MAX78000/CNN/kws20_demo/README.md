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

Load firmware image using Openocd:

```bash
./openocd -f tcl/interface/cmsis-dap.cfg -f tcl/target/max78000.cfg -c "program build/MAX78000.elf verify reset exit"
```

### Jumper setting:

Make sure to install jumper at JP20-CLK (INT position) as shown bellow:

![](/I2S_jumper.png)

Note: On board external oscillator Y3 is used to generate I2S clock. The I2S sample rate is 16kHz to match speech samples of the dataset.

### Operations:

The USB cable connected to CN1 (USB/PWR) provides power and serial communication to MAX78000 EVKIT.

To configure PC terminal program select correct COM port and settings as follow:

![](/Terminal2.png)

After turning on power switch (SW1) or pressing reset button (SW5)  following message will appear in terminal window:



![](/Terminal1.png)



Once red LED2 turns on, the initialization is complete and it is ready to accept keywords.

The microphone (U15) is located between JH4 and JH5 connecters.

The user can say keywords from the list:

 ['**up', 'down', 'left', 'right', 'stop', 'go', 'yes', 'no', 'on', 'off', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight', 'nine', 'zero**']

 The MAX78000 EVKIT firmware recognizes keywords and reports result and confidence level in PC terminal window.

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
(ai8x-training) $ ./go_kws20.sh
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

Confusion matrix (unquantized):

```
[ 1944     3    22     2    32     7     2     1    12   118     1     9     0     4    14     3     5     2     1     0    77]
[   49  1782     4     2    22    67     7    69     6     4     9    10     8     2     4     5    16     2    14     4   104]
[   50    13  1830    41     7     3    55    21     2     2     6     2     3     3     6     8     3     4    11     1    92]
[   51     1    73  1834     1     5     4     3     2     0    20     2    12     4    62     4     1     7    37     1    75]
[   75    30     4     1  2004    34     2     9     8    30     2     9     0    12     6     6    10     0     0     4    52]
[   66    93     5     8    13  1756     7   105     5    13     4    40     7    32     2     3     9     3     4    13   104]
[   51     8    93     1     3     4  2085    10     0     3     4     2     2     6     0    16     2     8     4     8    48]
[   43    84    30     5     3   152    14  1831     3     1     5     8     2     1     6     1     1     1    34    27    61]
[   69     8     2     4     2     7     0     1  1825    72    27     3    10    28    80     3     2     3     7     0    58]
[  183     6     6     2    19    10     2     4   107  1711     2     3     1    51    15     1     3     1     2     0    49]
[   51    10    12    42     5     4     4     7    42     2  1883     3     7    28    23     2     5     3    26     2    77]
[   54    14     3     3     6    50     3     6     3     4     2  1964    50    13     1    14     9    15     1    34    55]
[   39     1     3    14     3     5     0     3     8     0     3    91  1902     2     3    15     5    49     9     6    68]
[   30     3     4     4     8    47     0     6    33    55    39    11     6  1758    12     1     4     5     0     2    69]
[   76     5     5    74    18     2     0     0    87    17    30     0     8     3  2008     2     5    13    22     4    69]
[   43     1    21     2    14     3    26     2     3     1     0    31    17     3     1  2023    10    44     0     3    35]
[   41    21     7     2    45    21     2     0    12     2     2    21     8     6     5    23  2023     4     0     4   106]
[   42     2     2    30     0     6     4     1     3     2     1    27    70     4     7    36     1  1903     1     1    38]
[   62    16    13    72     4     2     2    56    20     1    26     2    18     1    37     0     1     1  1884     4    73]
[   39    15    11     2     9    22    22    24     0     2     3    45     8     1     0    14     9     9     1  2090    68]
[  521   358   324   157   170   329   143   242   147   149   208   140   973   582   234   103   240   113   313   315 10754]
```

Confusion matrix (quantized with scale=0.97):

```
[ 1929     1    18     0    38     7     2     2    11   135     2     5     1     6    15     2     3     3     0     0    79]
[   34  1780     7     5    25    84     7    60    10     6    11    12     8     4     2     3    12     1    13     6   100]
[   49     8  1819    42     6     2    69    24     1     3     7     1     2     7     3    14     2     5     8     4    87]
[   43     2    73  1840     1     5     3     3     2     0    19     1    11     7    75     5     1    12    30     0    66]
[   61    29     7     2  2013    24     1     6     8    46     2     6     0    16     5     7     9     2     0     4    50]
[   54    96     5    15    15  1749     8   101    10    13     7    36    10    40     2     3     6     4     4     9   105]
[   31     7    92     0     5     3  2095     9     1     3     4     1     1     5     2    22     1    10     4    13    49]
[   45    78    37     5     8   183    17  1780     7     2     6    12     3     0     4     0     0     4    33    22    67]
[   51     6     4     5     5     4     0     0  1852    89    24     3     8    28    65     2     1     7     5     1    51]
[  186     8     6     2    18     7     0     3   123  1711     5     1     0    41     7     1     2     6     2     2    47]
[   41    14    20    58     5     3     6    10    49     8  1866     2     4    38    25     1     1     1    24     4    58]
[   52     8     6     4     6    56     3     6     4     6     1  1933    46    16     0    22     8    34     0    31    62]
[   36     1     1    22     2     6     4     1    10     0     2    75  1884     4     3    25     9    68     9     5    62]
[   32     3     4     3     6    43     0     4    45    63    35    10     7  1761    10     1     4     4     0     1    61]
[   74     4     7    66    27     3     0     1   100    23    25     0    12     4  1995     2     3    17    15     2    68]
[   32     2    22     3    13     2    20     1     0     0     1    26    15     5     3  2023    16    57     1     4    37]
[   52    18     5     1    42    23     2     0    11     3     1    25     5     7     1    25  2004     8     1     3   118]
[   21     0     3    28     1     5     3     2     2     4     1    18    55     3     8    40     1  1949     3     3    31]
[   50    21    18    90     7     7     2    68    18     1    29     1    19     1    61     1     1     0  1821     2    77]
[   34    16    10     1     7    22    27    32     0     2     3    53     5     7     1    18    10    10     3  2062    71]
[  493   345   337   206   198   298   155   262   194   200   243   118   939   634   272   142   220   166   294   290 10509]
```



## Network Synthesis

The network synthesis script generates a pass/fail C example code which includes necessary functions to initialize MAX78000 CNN accelerator, to load quantized CNN weights and input samples and to unload classification results. A sample input with the expected result is part of this automatically generated code to verify.  Following script generates all example projects including ai85-kws20:

```bash
(ai8x-synthesis) $ ./gen-demos-ai85.sh
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