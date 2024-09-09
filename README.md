# LoRa-PingPong-gmake-demo
A sample LoRa ping-pong program for the STM32WL5x dev boards. Built with gmake. 

## Building
**NOTE: It is REQUIRED to use stm32-for-vscode to build this project. This is due to the
toolchain that it uses internally. Using `arm-none-eabi-gcc` does NOT work outside of
Visual Studio Code because of the version of the compiler that `stm32-for-vscode` uses
internally. I will make a note here when this is figured out and fixed.**

<!-- ### Ignore:
GNU Make -- as well as whatever embedded ARM compiler preferred for your
system -- is **REQUIRED** to build this project.

It is recommended to use Visual Studio Code with the stm32-for-vscode extension
to build and flash this project, unless you know what you're doing (lord knows
I dont). -->

## Configuration
If changing the parameters/conditions for your STM32WL5x dev board, STM32CubeMX is
highly recommended as this project uses STM32Cube boilerplate.

## Resources
For doing all of this: https://forum.digikey.com/t/using-the-low-level-sub-ghz-radio-driver-for-the-stm32wl-series/18253

## To all uninformed, please read
https://www.reddit.com/r/robotics/comments/jhfrbg/looking_for_alternative_terminology_for/

