# The MAX78000 SDK

Refer to [Getting Started with the MAX78000 Evaluation Kit](https://github.com/MaximIntegratedAI/MaximAI_Documentation/blob/master/MAX78000_Evaluation_Kit/README.md) for details on using the SDK.

## Release Notes

July 9, 2020
* Moved camera driver files from the example folders to the Libraries\Boards\MAX78000 folder.
* Replace the CNN/cats_dogs-chw-fifo example with the CNN/cats-dogs_demo example.

July 1, 2020
* Added macros for referencing evaluation kit LEDs by their name on the board.
* Improved support for the TFT display on the evaluation kit.
* Added MXC_UART_ReadCharacterRaw and MXC_UART_WriteCharacterRaw functions.  The "Raw" functions are non-blocking and will return an error code if a character cannot be read or written.  The "non-raw" functions will block until the character can be processed.
* Improved support for paralled camera interface.
* Added these examples: CameraIF, cats_dogs-chw-fifo, faceid, faceid_demo, 

June 16, 2020
* Updated the CNN/kws20_demo example application.  The most significant update being the addition of TFT support.
* Updated the wakeup timer (WUT) example to include deep sleep mode.
* Modified the TFT and touchscreen code in the board folder to be more board independent.
* Corrected some misnamed registers in adc_regs.h.
* Replaced all instances of IntEnable and IntDisable with EnableInt and Disable to maintain consistency across all drivers.
* Added support for the low-power GPIO pins (P3.0 and P3.1).

June 7, 2020
* In the Documentation folder of the SDK, added a MAX78000.html file which serves as a single source for all MAX78000 related documents.
* Added a README.md file for each example.
* Added automatic configuration of ADC GPIO pins.
* Fixed minor GPIO peripheral library bugs.
* Added optional configuration of TMR GPIO pins.

May 21, 2020
* Initial Release

## Known Issues & Upcoming Features

* Debug support for the RISC-V core has not been validated.
* There will be more examples coming in a future release.
* The Eclipse projects show errors and warnings indicating there are unresolved references.  However, the projects still build and run correctly.
* When uninstalling the SDK using the maintenance tool, an error will occur referring to the fstab file.  This error can be ignored - the uninstall will still complete.

## Example Status

| Example              |  GCC Status | Eclipse Support | Notes                                                                                           |
|----------------------|:-----------:|:---------------:|-------------------------------------------------------------------------------------------------|
| ADC                  |   Complete  |       Yes       | The overflow, low limit, and high limit are not reported properly in the example.               |
| AES                  |   Complete  |       Yes       |                                                                                                 |
| CNN/cats-vs-dogs-chw |   Complete  |        No       |                                                                                                 |
| CNN/cifar-10         |   Complete  |        No       |                                                                                                 |
| CNN/cifar-100        |   Complete  |        No       |                                                                                                 |
| CNN/kws20            |   Complete  |        No       |                                                                                                 |
| CNN/kws20_demo       |   Complete  |     Partial     | The example can be imported into Eclipse, but it has no debug configuration associated with it. |
| CNN/mnist            |   Complete  |        No       |                                                                                                 |
| CNN/mnist-riscv      |   Complete  |        No       |                                                                                                 |
| CRC                  |   Complete  |       Yes       |                                                                                                 |
| DMA                  |   Complete  |       Yes       |                                                                                                 |
| ECC                  |   Complete  |       Yes       |                                                                                                 |
| Flash                | In Progress |   In Progress   |                                                                                                 |
| GPIO                 |   Complete  |       Yes       |                                                                                                 |
| Hello_World          |   Complete  |       Yes       |                                                                                                 |
| I2C                  | In Progress |   In Progress   |                                                                                                 |
| I2S                  | In Progress |   In Progress   |                                                                                                 |
| ICC                  |   Complete  |       Yes       |                                                                                                 |
| LP                   | In Progress |   In Progress   |                                                                                                 |
| Pulse_Train          |   Complete  |       Yes       |                                                                                                 |
| RTC                  |   Complete  |       Yes       |                                                                                                 |
| SPI                  |   Complete  |       Yes       |                                                                                                 |
| TFT_Demo             |   Complete  |       Yes       | The Readme.md file needs to be completed.                                                       |
| TMR                  |   Complete  |       Yes       | The PWM signal created by this example is not currently working.                                |
| TRNG                 |   Complete  |       Yes       |                                                                                                 |
| UART                 | In Progress |   In Progress   |                                                                                                 |
| Watchdog             | In Progress |   In Progress   |                                                                                                 |
| WUT                  | In Progress |   In Progress   |                                                                                                 |
