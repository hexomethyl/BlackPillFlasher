/*
 * Copyright (c) 2022 Balázs Triszka <balika011@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>

#include "bsp/board_api.h"
#include "stm32f4xx_hal.h"
#include "tusb.h"

#include "xbox.h"
#include "protocol.h"

// Debug UART channels multiplexed over the vendor interface.
#define DBG_UART_KER   0 // KER_DBG on USART1 (PA9 TX / PA10 RX, AF7)
#define DBG_UART_SMC   1 // SMC_DBG on USART2 (PA2 TX / PA3 RX, AF7)
#define DBG_UART_COUNT 2

void led_blink(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;

	uint32_t now = HAL_GetTick();

	if (now - start_ms < 50)
		return;

	start_ms = now;

	board_led_write(led_state);
	led_state = 1 - led_state;
}

bool stream_emmc = false;
bool do_stream = false;
uint32_t stream_offset = 0;
uint32_t stream_end = 0;

void blackpill_flasher_stream(void)
{
	if (do_stream)
	{
		if (stream_offset >= stream_end)
		{
			do_stream = false;
			return;
		}

		if (tud_cdc_write_available() < 4 + (stream_emmc ? 0x200 : 0x210))
			return;

		if (!stream_emmc)
		{
			static uint8_t buffer[4 + 0x210];
			uint32_t ret = xbox_nand_read_block(stream_offset, &buffer[4], &buffer[4 + 0x200]);
			memcpy(buffer, &ret, sizeof(ret));
			if (ret == 0)
			{
				tud_cdc_write(buffer, sizeof(buffer));
				++stream_offset;
			}
			else
			{
				tud_cdc_write(&ret, 4);
				do_stream = false;
			}
		}
		else
		{
			static uint8_t buffer[4 + 0x200];
			uint32_t ret = xbox_emmc_read_block(stream_offset, &buffer[4]);
			memcpy(buffer, &ret, sizeof(ret));
			if (ret == 0)
			{
				tud_cdc_write(buffer, sizeof(buffer));
				++stream_offset;
			}
			else
			{
				tud_cdc_write(&ret, 4);
				do_stream = false;
			}
		}
	}
}

static bool enable_smc_workaround = true;

//--------------------------------------------------------------------+
// Debug UART bridge
//--------------------------------------------------------------------+

// The RP2040 PicoFlasher build bridged the two debug UARTs to two extra CDC ports.  The
// F411 OTG_FS core only has 3 usable IN endpoints besides EP0 (not enough
// for 3 CDC functions), so they are multiplexed over the vendor bulk
// interface instead.  Framing, both directions: [channel u8][len u8][len
// payload bytes]; baud rates are set with the SET_UART_BAUD command on the
// CDC command port.

static UART_HandleTypeDef dbg_uart[DBG_UART_COUNT] = {
	{
		.Instance = USART1,
		.Init = {
			.BaudRate = 115200,
			.WordLength = UART_WORDLENGTH_8B,
			.StopBits = UART_STOPBITS_1,
			.Parity = UART_PARITY_NONE,
			.Mode = UART_MODE_TX_RX,
			.HwFlowCtl = UART_HWCONTROL_NONE,
			.OverSampling = UART_OVERSAMPLING_16,
		},
	},
	{
		.Instance = USART2,
		.Init = {
			.BaudRate = 115200,
			.WordLength = UART_WORDLENGTH_8B,
			.StopBits = UART_STOPBITS_1,
			.Parity = UART_PARITY_NONE,
			.Mode = UART_MODE_TX_RX,
			.HwFlowCtl = UART_HWCONTROL_NONE,
			.OverSampling = UART_OVERSAMPLING_16,
		},
	},
};

// Register-level access: the bridge polls SR flags directly, like the RP2040 PicoFlasher
// build's uart_is_readable/uart_is_writable, instead of paying the HAL's
// per-byte bookkeeping.
static USART_TypeDef *const dbg_usart[DBG_UART_COUNT] = {USART1, USART2};

static inline bool uart_rx_ready(int chan)
{
	return dbg_usart[chan]->SR & USART_SR_RXNE;
}

static inline bool uart_tx_ready(int chan)
{
	return dbg_usart[chan]->SR & USART_SR_TXE;
}

static inline uint8_t uart_read(int chan)
{
	return (uint8_t)dbg_usart[chan]->DR;
}

static inline void uart_write(int chan, uint8_t data)
{
	dbg_usart[chan]->DR = data;
}

// Also runs for the BSP board logger's USART2 init inside board_init(); that
// is harmless (same clock and pins) and its RXNE interrupt is disarmed in
// main() before the bridge takes over.
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef gpio = {0};

	if (huart->Instance == USART1)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_USART1_CLK_ENABLE();

		gpio.Pin = GPIO_PIN_9 | GPIO_PIN_10;
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pull = GPIO_PULLUP;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Alternate = GPIO_AF7_USART1;
		HAL_GPIO_Init(GPIOA, &gpio);
	}
	else if (huart->Instance == USART2)
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
		__HAL_RCC_USART2_CLK_ENABLE();

		gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
		gpio.Mode = GPIO_MODE_AF_PP;
		gpio.Pull = GPIO_PULLUP;
		gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		gpio.Alternate = GPIO_AF7_USART2;
		HAL_GPIO_Init(GPIOA, &gpio);
	}
}

static void dbg_uart_init(int chan)
{
	HAL_UART_Init(&dbg_uart[chan]);
}

static void dbg_uart_set_baud(int chan, uint32_t baud)
{
	HAL_UART_DeInit(&dbg_uart[chan]);
	dbg_uart[chan].Init.BaudRate = baud;
	HAL_UART_Init(&dbg_uart[chan]); // HAL_UART_MspInit re-configures clock + pins
}

// Per-channel byte budget per main-loop iteration, exactly like the RP2040 PicoFlasher
// build's uart_bridge_task.
#define DBG_BRIDGE_BUDGET 64

// Host -> MCU: bytes queued for a (possibly slow) UART.
#define DBG_TX_PENDING_SIZE 128

static uint8_t dbg_tx_pending[DBG_UART_COUNT][DBG_TX_PENDING_SIZE];
static uint8_t dbg_tx_count[DBG_UART_COUNT];
static uint8_t dbg_tx_pos[DBG_UART_COUNT];

// MCU -> host: readable UART bytes waiting for vendor FIFO space.
static uint8_t dbg_rx_pending[DBG_UART_COUNT][DBG_BRIDGE_BUDGET];
static uint8_t dbg_rx_pending_len[DBG_UART_COUNT];

// Host -> MCU frame parser state.
static uint8_t dbg_frame_buf[2 + DBG_BRIDGE_BUDGET];
static uint8_t dbg_frame_len;
static uint8_t dbg_frame_need = 2;

static void dbg_bridge_uart_tx(uint8_t chan, const uint8_t *data, uint8_t len)
{
	// Queue bytes for the UART.  If the host outruns a slow UART the excess
	// is dropped; the RP2040 PicoFlasher build had the same property (it simply stopped
	// reading while the UART was busy, so overflow had nowhere to go).
	for (uint8_t i = 0; i < len; ++i)
	{
		if (dbg_tx_count[chan] >= DBG_TX_PENDING_SIZE)
			break;

		dbg_tx_pending[chan][(dbg_tx_pos[chan] + dbg_tx_count[chan]) % DBG_TX_PENDING_SIZE] = data[i];
		++dbg_tx_count[chan];
	}
}

static void dbg_bridge_uart_drain(int chan)
{
	for (int maxw = DBG_BRIDGE_BUDGET; dbg_tx_count[chan] && uart_tx_ready(chan) && maxw; --maxw)
	{
		uart_write(chan, dbg_tx_pending[chan][dbg_tx_pos[chan]]);
		dbg_tx_pos[chan] = (dbg_tx_pos[chan] + 1) % DBG_TX_PENDING_SIZE;
		--dbg_tx_count[chan];
	}
}

static void dbg_bridge_uart_to_host(int chan)
{
	for (int maxr = DBG_BRIDGE_BUDGET; dbg_rx_pending_len[chan] < DBG_BRIDGE_BUDGET && uart_rx_ready(chan) && maxr; --maxr)
		dbg_rx_pending[chan][dbg_rx_pending_len[chan]++] = uart_read(chan);

	if (dbg_rx_pending_len[chan] == 0)
		return;

	if (tud_vendor_write_available() >= (uint32_t)dbg_rx_pending_len[chan] + 2)
	{
		uint8_t header[2] = {(uint8_t)chan, dbg_rx_pending_len[chan]};

		tud_vendor_write(header, sizeof(header));
		tud_vendor_write(dbg_rx_pending[chan], dbg_rx_pending_len[chan]);
		tud_vendor_flush();

		dbg_rx_pending_len[chan] = 0;
	}
	// else: not enough vendor FIFO space yet, keep the bytes pending
}

static void dbg_bridge_feed(uint8_t data)
{
	dbg_frame_buf[dbg_frame_len++] = data;

	if (dbg_frame_need == 2)
	{
		if (dbg_frame_len < 2)
			return;

		uint8_t len = dbg_frame_buf[1];
		if (dbg_frame_buf[0] >= DBG_UART_COUNT || len == 0 || len > DBG_BRIDGE_BUDGET)
		{
			// Malformed header: drop it and resync on the next byte.
			dbg_frame_len = 0;
			return;
		}

		dbg_frame_need = 2 + len;
	}

	if (dbg_frame_len >= dbg_frame_need)
	{
		dbg_bridge_uart_tx(dbg_frame_buf[0], &dbg_frame_buf[2], dbg_frame_need - 2);
		dbg_frame_len = 0;
		dbg_frame_need = 2;
	}
}

static void dbg_bridge_task(void)
{
	// (a) MCU -> host
	dbg_bridge_uart_to_host(DBG_UART_KER);
	dbg_bridge_uart_to_host(DBG_UART_SMC);

	// (b) host -> MCU
	uint8_t chunk[DBG_BRIDGE_BUDGET];

	for (int maxr = DBG_BRIDGE_BUDGET; tud_vendor_available() && maxr > 0;)
	{
		uint32_t count = tud_vendor_read(chunk, maxr < (int)sizeof(chunk) ? maxr : (int)sizeof(chunk));
		if (count == 0)
			break;

		for (uint32_t i = 0; i < count; ++i)
			dbg_bridge_feed(chunk[i]);

		maxr -= count;
	}

	dbg_bridge_uart_drain(DBG_UART_KER);
	dbg_bridge_uart_drain(DBG_UART_SMC);
}

//--------------------------------------------------------------------+
// STM32 system-memory DFU bootloader
//--------------------------------------------------------------------+

// Jump to the F411 system-memory DFU bootloader (0483:df11 per AN2606) so
// the host can reflash with dfu-util without touching the BOOT0 button.
// tud_disconnect() + delay happen before __disable_irq() because HAL_Delay
// needs the SysTick interrupt; afterwards we stop the tick, reset the clock
// tree to its reset (HSI) state for the bootloader, remap the vectors and
// hand over.  Holding BOOT0 during reset is the recovery path.
static void enter_stm32_dfu(void)
{
	tud_disconnect();
	HAL_Delay(100);

	__disable_irq();
	HAL_RCC_DeInit();

	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	SCB->VTOR = 0x1FFF0000;
	__DSB();
	__ISB();
	__set_MSP(*(volatile uint32_t *)0x1FFF0000);
	__enable_irq();
	((void (*)(void))(*(volatile uint32_t *)0x1FFF0004))();

	while (1)
		;
}

//--------------------------------------------------------------------+
// CDC command port (PicoFlasher binary protocol)
//--------------------------------------------------------------------+

static void blackpill_flasher_rx_cb(void)
{
	led_blink();

	uint32_t avilable_data = tud_cdc_available();

	uint32_t needed_data = sizeof(struct cmd);
	{
		uint8_t cmd = 0; // peek always succeeds: needed_data >= 5 <= avilable_data below
		tud_cdc_peek(&cmd);
		if (cmd == WRITE_FLASH)
			needed_data += 0x210;
		else if (cmd == EMMC_WRITE)
			needed_data += 0x200;
	}

	if (avilable_data >= needed_data)
	{
		struct cmd cmd;

		uint32_t count = tud_cdc_read(&cmd, sizeof(cmd));
		if (count != sizeof(cmd))
			return;

		switch (cmd.cmd)
		{
		case GET_VERSION:
		{
			uint32_t ver = 4;
			tud_cdc_write(&ver, 4);
			break;
		}
		case GET_FLASH_CONFIG:
		{
			// Stop SMC before reading the flash config.
			// Workaround for existing software not using the SMC control commands.
			if (enable_smc_workaround)
				xbox_stop_smc();

			uint32_t fc = xbox_get_flash_config();
			tud_cdc_write(&fc, 4);
			break;
		}
		case READ_FLASH:
		{
			uint8_t buffer[0x210];
			uint32_t ret = xbox_nand_read_block(cmd.lba, buffer, &buffer[0x200]);
			tud_cdc_write(&ret, 4);
			if (ret == 0)
				tud_cdc_write(buffer, sizeof(buffer));
			break;
		}
		case WRITE_FLASH:
		{
			uint8_t buffer[0x210];
			uint32_t read_count = tud_cdc_read(&buffer, sizeof(buffer));
			if (read_count != sizeof(buffer))
				return;
			uint32_t ret = xbox_nand_write_block(cmd.lba, buffer, &buffer[0x200]);
			tud_cdc_write(&ret, 4);
			break;
		}
		case ERASE_FLASH:
		{
			uint32_t ret = xbox_nand_erase_block(cmd.lba);
			tud_cdc_write(&ret, 4);
			break;
		}
		case READ_FLASH_STREAM:
			stream_emmc = false;
			do_stream = true;
			stream_offset = 0;
			stream_end = cmd.lba;
			break;
		case SET_SMC_WORKAROUND:
			enable_smc_workaround = cmd.lba & 1;
			break;
		case STOP_SMC:
			xbox_stop_smc();
			break;
		case START_SMC:
			xbox_start_smc();
			break;
		case SET_UART_BAUD:
		{
			uint8_t chan = cmd.lba >> 24;
			uint32_t baud = cmd.lba & 0xFFFFFF;
			if (chan < DBG_UART_COUNT && baud >= 110 && baud <= 3000000)
				dbg_uart_set_baud(chan, baud);
			break;
		}
		case EMMC_DETECT:
		{
			uint32_t fc = xbox_get_flash_config();
			int emmc_detect_result = (fc & 0xF0000000) == 0xC0000000;
			tud_cdc_write(&emmc_detect_result, 1);
			break;
		}
		case EMMC_INIT:
		{
			uint32_t ret = xbox_emmc_init();
			tud_cdc_write(&ret, 4);
			break;
		}
		case EMMC_GET_CID:
		{
			uint8_t cid_raw[16] = {0};
			xbox_emmc_read_cid(cid_raw);
			tud_cdc_write(cid_raw, sizeof(cid_raw));
			break;
		}
		case EMMC_GET_CSD:
		{
			uint8_t csd_raw[16] = {0};
			xbox_emmc_read_csd(csd_raw);
			tud_cdc_write(csd_raw, sizeof(csd_raw));
			break;
		}
		case EMMC_GET_EXT_CSD:
		{
			uint8_t ext_csd[512];
			xbox_emmc_read_ext_csd(ext_csd);
			tud_cdc_write(ext_csd, sizeof(ext_csd));
			break;
		}
		case EMMC_READ:
		{
			uint8_t buffer[0x200];
			int ret = xbox_emmc_read_block(cmd.lba, buffer);
			tud_cdc_write(&ret, 4);
			if (ret == 0)
				tud_cdc_write(buffer, sizeof(buffer));
			break;
		}
		case EMMC_READ_STREAM:
			stream_emmc = true;
			do_stream = true;
			stream_offset = 0;
			stream_end = cmd.lba;
			break;
		case EMMC_WRITE:
		{
			uint8_t buffer[0x200];
			uint32_t read_count = tud_cdc_read(&buffer, sizeof(buffer));
			if (read_count != sizeof(buffer))
				return;
			uint32_t ret = xbox_emmc_write_block(cmd.lba, buffer);
			tud_cdc_write(&ret, 4);
			break;
		}
		case REBOOT_TO_BOOTLOADER:
			enter_stm32_dfu();
			break;
		}

		tud_cdc_write_flush();
	}
}

// Invoked when CDC interface received data from host
void tud_cdc_rx_cb(uint8_t itf)
{
	(void)itf;

	blackpill_flasher_rx_cb();
}

void tud_cdc_tx_complete_cb(uint8_t itf)
{
	(void)itf;

	led_blink();
}

void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *line_coding)
{
	(void)itf;

	// Start SMC on UART config change (e.g. setting speed of debug UART).
	// Workaround for existing software not using the SMC control commands.
	if (enable_smc_workaround && xbox_smc_stopped)
		xbox_start_smc();

	// 1200 baud is the host's reboot-to-bootloader gesture (the RP2040 PicoFlasher build
	// called rom_reset_usb_boot_extra here).
	if (line_coding->bit_rate == 1200)
		enter_stm32_dfu();

	// NOTE: the CDC line coding is deliberately not applied to any UART.
	// Unlike the RP2040 PicoFlasher build (one CDC port per UART), the two debug UARTs are
	// configured with the SET_UART_BAUD command on this command port.
}

int main(void)
{
	board_init();

	// The tinyusb BSP wired its board logger to USART2 (LOGGER_UART with
	// UART_ID=2 on the same PA2/PA3 pins) and armed the USART2 RXNE
	// interrupt.  CFG_TUSB_DEBUG is 0 so it never transmits, but its IRQ
	// would steal SMC_DBG bytes from our bridge.  Disarm it before any of
	// our USART2 setup below.
	HAL_NVIC_DisableIRQ(USART2_IRQn);
	USART2->CR1 &= ~USART_CR1_RXNEIE;

	xbox_init();
	dbg_uart_init(DBG_UART_KER);
	dbg_uart_init(DBG_UART_SMC);

	tusb_init();
	board_init_after_tusb();

	while (1)
	{
		tud_task();
		blackpill_flasher_stream();
		dbg_bridge_task();
	}

	return 0;
}
