PROJ_NAME=bringup

STANDALONE=..
BSP ?= efinix/EfxSapphireSoc
SRCS = 	$(wildcard src/*.c) \
		$(wildcard src/*.cpp) \
		$(wildcard src/*.S) \
        ${STANDALONE}/common/start.S \
        ${STANDALONE}/common/trap.S

# The whole image (text+rodata+data+bss+stack) has to fit the 32 KB OCR RAM.
# Measured worst-case stack depth is ~1 KB (main 240 B -> fft_run_one 368 B
# -> bsp_printf 96 B -> bsp_printf_d 48 B), so 2 KB keeps a 2x margin and
# hands the other 2 KB back to code, which is the scarce resource here.
# NOTE: this must precede the -T from standalone.mk.  ld only honours
# --defsym for a linker-script DEFINED() test when the symbol is already
# defined by the time the script is evaluated.
LDFLAGS += -Wl,--defsym=__stack_size=0x800

include ${STANDALONE}/common/bsp.mk
include ${STANDALONE}/common/riscv64-unknown-elf.mk
include ${STANDALONE}/common/standalone.mk

LDSCRIPT = ${BSP_PATH}/linker/default.ld
