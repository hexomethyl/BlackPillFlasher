#!/bin/bash
# Builds BlackPillFlasher for the WeAct STM32F411CEU6 Black Pill.
#
# Prerequisites:
# - arm-none-eabi-gcc on PATH
# - one-time: git submodule update --init
#             python3 tinyusb/tools/get_deps.py stm32f4
set -e

echo "Building BlackPillFlasher for the STM32F411 Black Pill..."
make -j$(nproc) BOARD=stm32f411blackpill
cp _build/stm32f411blackpill/BlackPillFlasher.bin BlackPillFlasher.bin

echo "Build complete:"
echo "- BlackPillFlasher.bin"
