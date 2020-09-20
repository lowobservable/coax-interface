#!/bin/bash

xxd -i ../rtl/top.bin | tail -n +2 | head -n -2 > src/bitstream.hex
