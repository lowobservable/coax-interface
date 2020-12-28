# interface2

A modern USB connected interface using Lattice iCE40 FPGA and STM32 microcontroller.

## Design

## Hardware

You can find the Gerber files for fabricating a PCB in the [fabrication](hardware/fabrication) directory. I have used JLCPCB to make the PCBs.

## FPGA

An FPGA is used to handle the [3270 coax protocol](../protocol/protocol.md), it provides a simple TX and RX FIFO to a SPI connected microcontroller.

![Block Diagram](.images/fpga_block_diagram.png)

The design targets the [Lattice iCE40 UltraPlus 5K](http://www.latticesemi.com/view_document?document_id=51968) device.

## Firmware
