# GD32VF103 baremetal software "library"

Source with startup code and headers with hardware constants.

If CK_SYS_VALUE not defined, it sets for 48000000 (48 MHz).

Code at main.c - RGB led blink for "Loongan Nano" board.


build hint:
~/tools/riscv/bin/riscv-none-elf-gcc -DCK_SYS_VALUE=64000000 startup.S n200_func.c init.c main.c -Os -flto -s -march=rv32imac_zicsr -mabi=ilp32 -ffunction-sections -fdata-sections -nostartfiles -ffreestanding -Wl,-gc-sections -Wl,-TGD32VF103C8Tx_FLASH.ld -o a.elf
