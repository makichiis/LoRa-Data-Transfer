# LoRa-PingPong-gmake-demo
A sample LoRa ping-pong program for the STM32WL5x dev boards. Built with gmake. 

## Building
**NOTE: It is REQUIRED to use stm32-for-vscode to build this project. This is due to the
toolchain that it uses internally. Using `arm-none-eabi-gcc` does NOT work outside of
Visual Studio Code because of the version of the compiler that `stm32-for-vscode` uses
internally. I will make a note here when this is figured out and fixed.**

## (Linux) Configuring and Listening to TTY ports
The STM32WL5x WL55JC1 dev board is able to send UART serial through its programming port
via the USART2 channel. On Linux, the first UART device is usually opened on `/dev/ttyACM0`,
which can be confirmed by running
```sh
$ sudo dmesg | grep tty
```

In order to listen to this port, the baud rate and some QoL settings must be specified to
properly consume UART output. This can be bootstrapped by running the local `configure-port.sh`
script located at the root of this project directory:
```sh
$ ./configure-port.sh /dev/ttyACM0 # or whatever device you want to configure
```

Note: If you get a permissions error trying to run the configure script, run
```sh
$ chmod +x ./configure-port.sh
```

This gives the script file permission to be executed by the owner of this folder (likely you).

If you still get permissions or device I/O errors, try running as root
```sh
$ sudo $(realpath .)/configure-port.sh ...
```

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

