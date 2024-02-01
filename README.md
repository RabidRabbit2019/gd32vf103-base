# GD32VF103 baremetal software "library"

Source with startup code and headers with hardware constants.

If CK_SYS_VALUE not defined, it sets for 48000000 (48 MHz), assume presents high speed xtal 8 MHz at board.

Code at main.c - RGB led blink for "Sipeed Longan Nano" board. (https://longan.sipeed.com/en/)

Into libbaremetal/src/libc added some functions from musl libc sources.


# Toolchain

I use it with compiler from https://github.com/riscv-collab/riscv-gnu-toolchain
Built with ./configure --prefix=/opt/riscv --enable-multilib


# Build

make
