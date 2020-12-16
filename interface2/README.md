# interface2

A modern USB connected interface using Lattice iCE40 FPGA and STM32 microcontroller.

## Design

## Hardware

This interface requires an [Arduino Mega 2560 R3](https://store.arduino.cc/usa/mega-2560-r3).

You can find the Gerber files for fabricating a PCB in the [fabrication](hardware/fabrication) directory. I have used JLCPCB to make the PCBs.

## FPGA

## Firmware

The firmware currently provides the ability to send commands and receive responses - it is designed to implement a terminal controller, not a terminal.

You will need [PlatformIO](https://platformio.org/) to build and upload the firmware, only Platform IO Core (the CLI) is required.

TODO:The firmware can be uploaded to the interface by placing it in DFU mode.

To build and upload the firmware:

```
platformio run -t upload
```
