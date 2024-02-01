TOOLS=/home/vzaytsev/tools/riscv-gcc/bin/

all: lib/libc.a lib/libm.a

lib/libc.a: src/libc/dummy_libc.o \
            src/libc/gmtime_r.o \
            src/libc/memset.o \
            src/libc/bzero.o \
            src/libc/snprintf.o \
            src/libc/strstr.o \
            src/libc/memmem.o \
            src/libc/memcmp.o \
            src/libc/memchr.o \
            src/libc/strchr.o \
            src/libc/strchrnul.o \
            src/libc/strlen.o \
            src/libc/strnlen.o \
            src/libc/__secs_to_tm.o \
            src/libc/__year_to_secs.o \
            src/libc/__month_to_secs.o \
            src/libc/__tm_to_secs.o \
            src/libc/timegm.o \
            src/libc/strcspn.o \
            src/libc/strsep.o \
            src/libc/memcpy.o
	$(TOOLS)riscv64-unknown-elf-ar cr lib/libc.a \
	          src/libc/dummy_libc.o \
	          src/libc/gmtime_r.o \
	          src/libc/memset.o \
	          src/libc/bzero.o \
	          src/libc/snprintf.o \
	          src/libc/strstr.o \
	          src/libc/memmem.o \
	          src/libc/memcmp.o \
	          src/libc/memchr.o \
	          src/libc/strchr.o \
	          src/libc/strchrnul.o \
	          src/libc/strlen.o \
	          src/libc/strnlen.o \
	          src/libc/__secs_to_tm.o \
	          src/libc/__year_to_secs.o \
	          src/libc/__month_to_secs.o \
	          src/libc/__tm_to_secs.o \
	          src/libc/timegm.o \
	          src/libc/strcspn.o \
	          src/libc/strsep.o \
	          src/libc/memcpy.o
	$(TOOLS)riscv64-unknown-elf-ranlib lib/libc.a

src/libc/dummy_libc.o: src/libc/dummy_libc.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/dummy_libc.c -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/dummy_libc.o

src/libc/gmtime_r.o: src/libc/gmtime_r.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/gmtime_r.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/gmtime_r.o
	
src/libc/memset.o: src/libc/memset.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/memset.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/memset.o

src/libc/bzero.o: src/libc/bzero.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/bzero.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/bzero.o

src/libc/snprintf.o: src/libc/snprintf.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/snprintf.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/snprintf.o

src/libc/strstr.o: src/libc/strstr.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strstr.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strstr.o

src/libc/memmem.o: src/libc/memmem.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/memmem.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/memmem.o

src/libc/memcmp.o: src/libc/memcmp.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/memcmp.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/memcmp.o

src/libc/memchr.o: src/libc/memchr.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/memchr.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/memchr.o

src/libc/strchr.o: src/libc/strchr.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strchr.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strchr.o

src/libc/strchrnul.o: src/libc/strchrnul.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strchrnul.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strchrnul.o

src/libc/strlen.o: src/libc/strlen.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strlen.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strlen.o

src/libc/strnlen.o: src/libc/strnlen.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strnlen.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strnlen.o

src/libc/__secs_to_tm.o: src/libc/__secs_to_tm.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/__secs_to_tm.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/__secs_to_tm.o

src/libc/__year_to_secs.o: src/libc/__year_to_secs.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/__year_to_secs.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/__year_to_secs.o

src/libc/__month_to_secs.o: src/libc/__month_to_secs.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/__month_to_secs.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/__month_to_secs.o

src/libc/__tm_to_secs.o: src/libc/__tm_to_secs.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/__tm_to_secs.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/__tm_to_secs.o

src/libc/timegm.o: src/libc/timegm.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/timegm.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/timegm.o

src/libc/strcspn.o: src/libc/strcspn.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strcspn.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strcspn.o

src/libc/strsep.o: src/libc/strsep.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/strsep.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/strsep.o

src/libc/memcpy.o: src/libc/memcpy.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libc/memcpy.c -I./include -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libc/memcpy.o


lib/libm.a: src/libm/dummy_libm.o
	$(TOOLS)riscv64-unknown-elf-ar cr lib/libm.a src/libm/dummy_libm.o
	$(TOOLS)riscv64-unknown-elf-ranlib lib/libm.a

src/libm/dummy_libm.o: src/libm/dummy_libm.c
	$(TOOLS)riscv64-unknown-elf-gcc src/libm/dummy_libm.c -Os -s --freestanding -march=rv32imac_zicsr -mabi=ilp32 -c -o src/libm/dummy_libm.o


clean:
	rm -f lib/libc.a  \
	lib/libm.a \
	src/libc/dummy_libc.o \
	src/libc/gmtime_r.o \
	src/libm/dummy_libm.o \
	src/libc/memset.o \
	src/libc/bzero.o \
	src/libc/snprintf.o \
	src/libc/strstr.o \
	src/libc/memmem.o \
	src/libc/memcmp.o \
	src/libc/memchr.o \
	src/libc/strchr.o \
	src/libc/strchrnul.o \
	src/libc/strnlen.o \
	src/libc/strlen.o \
	src/libc/__secs_to_tm.o \
	src/libc/__year_to_secs.o \
	src/libc/__month_to_secs.o \
	src/libc/__tm_to_secs.o \
	src/libc/timegm.o \
	src/libc/strcspn.o \
	src/libc/strsep.o \
	src/libc/memcpy.o
