---
typora-root-url: Resources

---

# MAX78000 Rock Paper Scissors Game



Description
-----------

This is a classic rock-paper-scissor game demo that user can play against the computer via the camera module. The model trained in this demo is used to classify images of "rock", "paper" and "scissor" hand gestures. The input size is 64x64 pixels RGB which is 3x64x64 in CHW format.

The example supports live capture from camera module and displays the result on the TFT LCD. The code also uses a sampledata header (sampledata.h) file to test a pre-defined input sample.

### Building Firmware:

Navigate directory where **rps-demo** software is located and build the project:

```bash
$ cd /Examples/MAX78000/CNN/rps-demo
$ make
```

If this is the first time after installing tools, or peripheral files have been updated, first clean drivers before rebuilding the project: 

```bash
$ make distclean
```

To compile code for MAX78000 EVKIT enable **BOARD=EvKit_V1** in Makefile:

```bash
# Specify the board used
ifeq "$(BOARD)" ""
BOARD=EvKit_V1
#BOARD=FTHR_RevA
endif
```

### Load firmware image to MAX78000 EVKIT

Connect USB cable to CN1 (USB/PWR) and turn ON power switch (SW1).

Connect PICO adapter to JH5 SWD header.

Load the firmware image using OpenOCD. If you are using Windows, perform this step in a MinGW shell.

```bash
openocd -s $MAXIM_PATH/Tools/OpenOCD/scripts -f interface/cmsis-dap.cfg -f target/max78000.cfg -c "program build/MAX78000.elf reset exit"
```

### MAX78000 EVKIT operations

*   If using camera and TFT LCD, place OVM7692 camera module on 'J4 Camera' header. Place TFT display on the display header.
*   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
*   Place jumper P0\_0 and P0\_1 on UART\_0\_EN header JH1.
*   Open a serial port application on the PC and connect to Ev-Kit's console UART at 115200, 8-N-1 configuration.

This demo is operated in two modes: Real-time data using Camera module or using sample image header file in offline mode.

In either mode, pushbutton trigger PB1(SW2) is used to capture and load an image into CNN engine. User is prompted to press PB1 to load an image


### Camera Mode 

To operate in this mode, comment out "#define USE\_SAMPLEDATA", defined in main.c.

This mode uses OVM7692 camera module to capture an image in RGB888 format. Since the model is trained using 64x64 pixel image, the PCIF peripheral captures 64x64 pixel image and displays it on LCD.

The data received from camera interface is an unsigned data and should be converted to signed data before feeding to the CNN network.

### Offline Mode

To operate in this mode, uncomment "#define USE\_SAMPLEDATA", defined in main.c. 

This mode uses a header file "sampledata.h" containing RGB image data and it should be included in the project to use it as an input to the cnn network. 

To create your own header file follow these steps:

1.  Navigate to Utility directory. $ cd Utility
2.  Download rock, paper or scissor hand gesture image in this directory.
3.  Open 'rgb.py' file and change the name of the image file on line 8. im = (Image.open('./image\_filename.format')). Save the cahanges.
4.  Now generate a header file using this command: python3 rgb.py
5.  Use this header file in your main.c

Terminal output
---------------

The Console UART of the device will output these messages:

```
Waiting...

*** CNN Inference Test ***

*** CNN Test ***
Init LCD.
Init Camera.
********** Press PB1 to capture an image **********

Capture a camera frame 1
Show camera frame on LCD.
Time for CNN: 2664 us

Classification results:
[-326446] -> Class 0    Paper: 0.0%
[ 314820] -> Class 1     Rock: 100.0%
[-228949] -> Class 2  Scissor: 0.0%

User choose: Rock 
Computer choose: Scissor 

USER WINS!!!
```

### References

https://github.com/MaximIntegratedAI/MaximAI_Documentation