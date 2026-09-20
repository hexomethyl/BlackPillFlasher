# BlackPillFlasher — PicoFlasher-compatible firmware for the WeAct
# STM32F411CEU6 "Black Pill", built out-of-tree against the bundled tinyusb
# submodule using the shared example build system (see
# tinyusb/examples/device/cdc_msc/Makefile for the pattern this mirrors).
#
# One-time setup:
#   git submodule update --init
#   python3 tinyusb/tools/get_deps.py stm32f4
#
# make -j BOARD=stm32f411blackpill
#
# Artifacts land in _build/stm32f411blackpill/BlackPillFlasher.{elf,bin,hex};
# the board's flash: target uploads the .bin with dfu-util at 0x08000000.

TOP = tinyusb

include $(TOP)/hw/bsp/family_support.mk

# Our sources.  They must stay plain relative paths: family_rules.mk
# locates sources with `vpath %.c . $(TOP)`, and make runs with the CWD set
# to this directory.  src/xbox.c is the PicoFlasher protocol core, absorbed
# from the upstream GPLv2 project into this tree (kept platform-neutral via
# src/xbox_hal.h; the STM32 side is src/xbox_hal_stm32.c).  The
# EXAMPLE_SOURCE/EXAMPLE_PATH mechanism is unusable here because it produces
# absolute paths only valid inside the tinyusb tree.
INC += src

SRC_C += \
	src/main.c \
	src/xbox_hal_stm32.c \
	src/usb_descriptors.c \
	src/xbox.c \

# The family HAL config shipped with tinyusb (hw/bsp/stm32f4/
# stm32f4xx_hal_conf.h) has HAL_SPI_MODULE_ENABLED commented out because no
# stm32f4 example uses SPI.  Our SPI2 Xbox 360 southbridge bus needs it:
# compile the SPI HAL driver and enable the module on the command line (the
# define reaches every TU, so the conf header picks up
# stm32f4xx_hal_spi.h as well).  tinyusb itself is never modified.
SRC_C += $(ST_HAL_DRIVER)/Src/stm32f4xx_hal_spi.c
CFLAGS += -DHAL_SPI_MODULE_ENABLED

# The absorbed PicoFlasher core (src/xbox.{c,h}) uses K&R-style empty
# parameter lists ("void xbox_init()"), which tinyusb's -Wstrict-prototypes
# -Werror would reject; the upstream code is not ours to restyle.
CFLAGS += -Wno-strict-prototypes

# Same story for -Wsign-compare and -Wcast-align: the shared core compares
# loop ints against sizeof() and casts byte buffers to uint32_t* (wire-level
# code that has shipped on the RP2040 build for years).  The classes are
# disabled globally; our own sources (src/main.c, src/xbox_hal_stm32.c,
# src/usb_descriptors.c) are warning-free with them enabled.
CFLAGS += -Wno-sign-compare -Wno-cast-align

include $(TOP)/hw/bsp/family_rules.mk

