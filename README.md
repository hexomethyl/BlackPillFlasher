# BlackPillFlasher — PicoFlasher on the WeAct STM32F411CEU6 Black Pill

A port of PicoFlasher — the open source Xbox 360 NAND/eMMC flasher — to the
WeAct **STM32F411CEU6 "Black Pill"** (25 MHz HSE, USB-C on the native USB OTG_FS
pins). It speaks the **unchanged PicoFlasher binary protocol** over a USB
CDC-ACM port with **VID 0x600D / PID 0x7001**, so it is auto-detected and works
out of the box with J-Runner-with-Extras (which requires `GET_VERSION >= 2`;
this firmware reports 4, same as the RP2040 build). On top of that it adds a
vendor bulk interface that multiplexes the two console debug UARTs.

The PicoFlasher protocol core is absorbed into this tree (`src/xbox.c`,
`src/xbox.h`, `src/protocol.h`) and kept platform-neutral via `src/xbox_hal.h`
(see [Licensing](#licensing)).  The upstream RP2040 PicoFlasher project is
unaffected and builds separately.

## Wiring

| Black Pill pin | Function | Xbox 360 signal |
|---|---|---|
| PB12 | SPI_SS_N (GPIO, manual chip select) | SPI_SS_N |
| PB13 | SPI2_SCK | SPI_CLK |
| PB14 | SPI2_MISO (input, internal pull-up) | SPI_MISO |
| PB15 | SPI2_MOSI | SPI_MOSI |
| PB10 | SMC_DBG_EN (GPIO out) | SMC_DBG_EN |
| PB11 | SMC_RST_XDK_N (GPIO out) | SMC_RST_XDK_N |
| PA9 | USART1_TX | KER_DBG_RXD |
| PA10 | USART1_RX | KER_DBG_TXD |
| PA2 | USART2_TX | SMC_DBG_RXD |
| PA3 | USART2_RX | SMC_DBG_TXD |
| GND | ground | GND |

All signals are 3.3 V (the STM32 GPIOs used are 5 V-tolerant inputs, but the
Xbox side is 3.3 V logic). PC13 is the activity LED (on-board, active low).

The Xbox SPI bus is on **SPI2** on purpose: **SPI1 (PA4–PA7) is left free for
the 8 MB W25Q64 flash chip mounted on this board revision.**

## Building

Prerequisites:

- `arm-none-eabi-gcc` on PATH
- `python3`
- `dfu-util`
- the pinned `tinyusb` submodule inside this checkout, with its STM32
  dependencies fetched (one-time, from this directory):

```sh
git submodule update --init
python3 tinyusb/tools/get_deps.py stm32f4
```

Then:

```sh
make BOARD=stm32f411blackpill
```

or, equivalently, `./build_release.sh`.

Artifacts land in `_build/stm32f411blackpill/`:
`BlackPillFlasher.elf`, `BlackPillFlasher.bin`, `BlackPillFlasher.hex`.

## Flashing

1. Hold **BOOT0**, plug in USB-C, release **BOOT0**. The board enumerates as
   the STM32 ROM DFU bootloader (`0483:df11`).
2. Flash and reboot:

```sh
make BOARD=stm32f411blackpill flash
```

which runs:

```sh
dfu-util -R -a 0 --dfuse-address 0x08000000 -D _build/stm32f411blackpill/BlackPillFlasher.bin
```

3. The device re-enumerates as `600d:7001` and a `/dev/ttyACM0` (CDC, MI_00)
   appears — that is the command port.

**Recovery:** if a firmware image ever misbehaves, hold **BOOT0** and press
**reset** to land back in the `0483:df11` ROM DFU bootloader and re-flash.

## USB layout

- **Interface 0/1 — CDC-ACM "command port"**: the PicoFlasher binary protocol,
  identical to the RP2040 build. Commands are packed 5-byte little-endian
  packets (`u8` opcode + `u32` lba); answered commands reply with a
  little-endian `u32` status (0 = success). See `src/protocol.h`.
- **Interface 2 — vendor bulk "debug bridge"**: multiplexes both console debug
  UARTs. Packet framing is identical in both directions:

```
[channel u8][len u8][len bytes of UART data]
```

  - channel 0 = KER_DBG (USART1: PA9/PA10)
  - channel 1 = SMC_DBG (USART2: PA2/PA3)
  - default rate 115200 8N1 on both channels
  - `len` is 1..64: **frames carry at most 64 payload bytes** (the bridge
    moves 64 bytes per direction per main-loop iteration, mirroring the
    RP2040 build's UART budgets); longer frames are dropped on the MCU and
    desync the stream, so split larger writes into multiple frames

### SET_UART_BAUD

Command `0x23` on the CDC command port changes a debug UART's baud rate:

```
cmd = 0x23, lba = (channel << 24) | baud   (baud <= 3000000, no response)
```

`channel` is 0 (KER_DBG) or 1 (SMC_DBG).

### Rebooting to the bootloader

- Setting the CDC port's line coding to **1200 baud**, or sending command
  `0xFE` (`REBOOT_TO_BOOTLOADER`), jumps to the STM32 ROM DFU bootloader
  (`0483:df11`).

## Host tool usage

`tools/blackpill_cli.py` (Python 3.8+, needs `pyserial`) is a reference
implementation of the whole command protocol and works against **both** the
RP2040 PicoFlasher and the BlackPillFlasher firmware:

```sh
python3 tools/blackpill_cli.py probe
python3 tools/blackpill_cli.py dump nand.bin --blocks 0x8000     # 16MB image
python3 tools/blackpill_cli.py dump nand.bin --blocks 0x20000    # 64MB image
python3 tools/blackpill_cli.py write nand.bin
python3 tools/blackpill_cli.py reboot-bootloader
```

`probe` prints the firmware version and decodes the console flash config;
`dump` streams `N` blocks of `0x210` bytes (STOP_SMC → READ_FLASH_STREAM →
START_SMC); `write` programs an image block by block (raw writes — erase the
range first if it holds data); `reboot-bootloader` jumps to DFU and prints the
`dfu-util` command to re-flash.

## Differences vs the RP2040 PicoFlasher build

- **Single CDC port.** The F411 USB controller allows only 3 IN endpoints
  besides EP0; the RP2040 build's three CDC-ACM ports (command + two debug
  UARTs) do
  not fit. The command port is unchanged; the two debug UARTs are multiplexed
  over the vendor bulk interface (interface 2) instead of CDC 1/2.
- **SET_UART_BAUD (0x23)** is new: it replaces setting the debug UART rate via
  CDC line coding on the two removed ports.
- **SPI clock**: SPI2 runs at 21 MHz (42 MHz APB1 clock, prescaler 2) — the
  same speed class as the RP2040 build, whose 28 MHz request actually yields
  ~22.2 MHz from the 133 MHz peripheral clock.
- **GET_VERSION is still 4**: J-Runner-with-Extras accepts anything >= 2.
- **Bootloader**: `REBOOT_TO_BOOTLOADER` / 1200 baud jumps to the STM32 ROM DFU
  bootloader (`0483:df11`, flashable with dfu-util) instead of the RP2040
  BOOTSEL mass-storage mode.

## Credits

This project builds on a chain of prior work:

- **Balázs Triszka (balika011)** — the original PicoFlasher firmware: the Xbox 360
  southbridge SPI register protocol, the NAND/eMMC engine and the USB CDC command
  protocol. This port compiles its `xbox.c` core verbatim.
- **15432** — eMMC-over-SPI support in PicoFlasher.
- **the hax360 / X360Tools PicoFlasher maintainers** — the upstream project whose
  protocol core is absorbed into `src/` here.
- **Ha Thach (hathach) and the TinyUSB contributors** — the TinyUSB stack and the
  `stm32f411blackpill` BSP, pinned as the `tinyusb` submodule.
- **STMicroelectronics** — the CMSIS device headers and STM32F4 HAL drivers
  fetched by `get_deps`, not modified.
- **Octal450 and the X360Tools J-Runner teams** — J-Runner-with-Extras /
  J-Runner-Pro, the host software whose device discovery (VID `600d:7001`,
  `GET_VERSION` gate) this firmware matches.
- **hexomethyl** — the STM32F411 Black Pill port (this repository).

## Licensing

- **GPLv2** — the PicoFlasher core absorbed from the upstream GPLv2 project
  (`src/xbox.c`, `src/xbox.h`, `src/protocol.h`) and the files ported from it
  (`src/main.c`, `src/usb_descriptors.c`, `src/tusb_config.h`); their GPL
  headers are retained. The firmware binary is therefore GPLv2.
- **MIT** (see [LICENSE](LICENSE)) — the original Black Pill port work:
  `src/xbox_hal.h`, `src/xbox_hal_stm32.c`, `tools/blackpill_cli.py`, the
  Makefile, `build_release.sh`, this README and LICENSE itself.
- Third-party dependencies: TinyUSB (MIT), ST CMSIS device headers (Apache-2.0),
  STM32F4 HAL drivers (BSD-3-Clause) — fetched via the submodule and
  `get_deps`, not modified.
- The host CLI is a separate work and is MIT.
