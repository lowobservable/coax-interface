#!/bin/bash

#BITSTREAM=../rtl/top.bin
BITSTREAM=/home/andrew/downloads/fpga-docker/Lattice_iCEcube2/home/lattice_coax/coax/coax_Implmnt/sbt/outputs/bitmap/top_bitmap.bin

xxd -i $BITSTREAM | tail -n +2 | head -n -2 > src/bitstream.inc
