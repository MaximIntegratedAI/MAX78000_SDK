## Description

Two pulse trains are configured in different modes.  

The first is set to generate a repeating bit pattern (default is 0b10110) at a user configurable bit rate (default 2bps). The pattern and bit rate can be changed with the "CONT\_WV\_PATTERN" and "CONT\_WV\_BPS" defines respectively.

The second is set to generate a square wave at a frequency defined by "SQ\_WV\_HZ".

On the Standard EV Kit, the continuous bit pattern and square wave signals are output to the P0.18 and P0.19 header pins respectively. If the connections described below are made, the signals will be visible on LEDs 1 and 2.

On the Featherboard, only one signal can be output at a time to the P0.19 header pin. The signal is selected by setting "FTHR\_OUT\_SQ" for the square wave or "FTHR\_OUT\_CONT" for the continuous bit pattern. The signal cannot be connected to the LED on the Featherboard.

##Setup

##### Building Firmware:

Before building firmware you must select the correct value for _BOARD_  in "Makefile", either "EvKit\_V1" or "FTHR\_RevA", depending on the EV kit you are using to run the example.

After doing so, navigate to the directory where the example is located using a terminal window. Enter the following comand to build all of the files needed to run the example.

```
$ make
```

##### Required Connections:

If using the Standard EV Kit (EvKit_V1):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Connect pins 1 and 2 (P0_1) of the JH1 (UART 0 EN) header.
-   Open an terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.
-   Connect pin 12 of JH4 (Camera) to pin 2 of JP1 (LED1).
-   Connect pin 9 of the DSP2 connector to pin 2 of JP2 (LED2).
-   Connect pins 1 and 2 of R52.

If using the Featherboard (FTHR_RevA):
-   Connect a USB cable between the PC and the CN1 (USB/PWR) connector.
-   Open a terminal application on the PC and connect to the EV kit's console UART at 115200, 8-N-1.

## Expected Output

The Console UART of the device will output these messages for the Standard EV Kit:

```
*************** Pulse Train Demo ****************
PT0 (P0.18) = Outputs continuous pattern of 10110b at 2bps
PT1 (P0.19) = Outputs 10Hz continuous square wave
```

The Console UART of the device will output these messages for the Featherboard:

```
***** Pulse Train Demo *****
This example outputs either a continuous pattern of 0x16 at
2bps or a 10Hz continuous square wave on PT channel 1 (P0.19).
```