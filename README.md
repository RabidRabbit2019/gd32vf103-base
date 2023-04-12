# GD32VF103 baremetal software "library"

Source with startup code and headers with hardware constants.

If CK_SYS_VALUE not defined, it sets for 48000000 (48 MHz).

Code at main.c - RGB led blink for "Sipeed Longan Nano" board. (https://longan.sipeed.com/en/)

I use it with compiler from gcc-riscv64-unknown-elf package (debian)

# Build

make
