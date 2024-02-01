TOOLS=/opt/riscv/bin/

all: a.elf

a.elf: libbaremetal/lib/libc.a libbaremetal/lib/libm.a
	$(TOOLS)riscv64-unknown-elf-gcc -DCK_SYS_VALUE=64000000 \
	    startup.S \
	    n200_func.c \
	    init.c \
	    main.c \
	    -Os -flto -s -march=rv32imac_zicsr -mabi=ilp32 \
	    -ffunction-sections -fdata-sections -nostartfiles \
	    -ffreestanding -nostdlib -nodefaultlibs \
	    -L./libbaremetal/lib -Wl,-gc-sections \
	    -Wl,-TGD32VF103C8Tx_FLASH.ld \
	    -o a.elf

libbaremetal/lib/libc.a:
	$(MAKE) -C ./libbaremetal

libbaremetal/lib/libm.a:
	$(MAKE) -C ./libbaremetal

clean:
	rm -f a.elf
	$(MAKE) -C ./libbaremetal clean
