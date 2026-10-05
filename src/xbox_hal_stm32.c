/* SPDX-License-Identifier: MIT */
/*
 * Copyright (c) 2026 hexomethyl
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, subject to the following
 * conditions: the above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY.
 */

#include "stm32f4xx_hal.h"

#include "xbox_hal.h"

//--------------------------------------------------------------------+
// Pin map (Xbox 360 southbridge bus, all on GPIOB)
//--------------------------------------------------------------------+
// PB12 SPI_SS_N       manual chip select, idle high
// PB13 SPI2_SCK       AF5
// PB14 SPI2_MISO      AF5, input with internal pull-up
// PB15 SPI2_MOSI      AF5
// PB10 SMC_DBG_EN     output, idle low
// PB1  SMC_RST_XDK_N  output, idle high

#define SPI_SS_N_PIN      GPIO_PIN_12
#define SPI_SCK_PIN       GPIO_PIN_13
#define SPI_MISO_PIN      GPIO_PIN_14
#define SPI_MOSI_PIN      GPIO_PIN_15
#define SMC_DBG_EN_PIN    GPIO_PIN_10
#define SMC_RST_XDK_N_PIN GPIO_PIN_1

// SPI2 hangs off APB1 (42 MHz after the BSP clock init).  Prescaler 2 gives
// a 21 MHz SCK: the RP2040 build asks spi_init() for 28 MHz but its clock
// tree (133 MHz, prescaler 2, postdiv 3) actually runs the bus at ~22.2 MHz,
// so this is the same speed class, not a slowdown.  If the southbridge
// proves flaky at 21 MHz, drop to SPI_BAUDRATEPRESCALER_4 (10.5 MHz).
//
// FirstBit stays MSB on purpose: the shared xbox.c reverses bits in
// software with its lsb2msb[] table, and keeping the wire bytes identical
// to the RP2040 build avoids any behavioral delta.
static SPI_HandleTypeDef hspi2 = {
	.Instance = SPI2,
	.Init = {
		.Mode = SPI_MODE_MASTER,
		.Direction = SPI_DIRECTION_2LINES,
		.DataSize = SPI_DATASIZE_8BIT,
		.CLKPolarity = SPI_POLARITY_LOW,	// SPI mode 0
		.CLKPhase = SPI_PHASE_1EDGE,
		.NSS = SPI_NSS_SOFT,			// chip select is GPIO PB12
		.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2,
		.FirstBit = SPI_FIRSTBIT_MSB,
		.TIMode = SPI_TIMODE_DISABLE,
		.CRCCalculation = SPI_CRCCALCULATION_DISABLE,
		.CRCPolynomial = 10,
	},
};

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	GPIO_InitTypeDef gpio = {0};

	if (hspi->Instance != SPI2)
		return;

	/* SCK + MOSI: AF push-pull, no pull. */
	gpio.Pin = SPI_SCK_PIN | SPI_MOSI_PIN;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	gpio.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOB, &gpio);

	/* MISO mirrors the RP2040 build's gpio_pull_up(SPI_MISO): the shared
	   bus idles high between command phases. */
	gpio.Pin = SPI_MISO_PIN;
	gpio.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &gpio);
}

void xbox_hal_init(void)
{
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_SPI2_CLK_ENABLE();

	/* Preload the output latches before switching the pins to outputs so a
	   (re-)init never glitches SS_N/RST_XDK_N low or DBG_EN high. */
	HAL_GPIO_WritePin(GPIOB, SPI_SS_N_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, SMC_RST_XDK_N_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, SMC_DBG_EN_PIN, GPIO_PIN_RESET);

	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = SPI_SS_N_PIN | SMC_DBG_EN_PIN | SMC_RST_XDK_N_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOB, &gpio);

	HAL_SPI_Init(&hspi2);
}

void xbox_hal_spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len)
{
	/* len <= 6 in practice: every southbridge register access is one short
	   CS-framed transaction, so a blocking HAL call is the right shape. */
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, len, 100);
}

void xbox_hal_spi_write(const uint8_t *tx, uint32_t len)
{
	HAL_SPI_Transmit(&hspi2, tx, len, 100);
}

void xbox_hal_ss(bool selected)
{
	HAL_GPIO_WritePin(GPIOB, SPI_SS_N_PIN, selected ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void xbox_hal_dbg_en(bool asserted)
{
	HAL_GPIO_WritePin(GPIOB, SMC_DBG_EN_PIN, asserted ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void xbox_hal_rst_xdk_n(bool high)
{
	HAL_GPIO_WritePin(GPIOB, SMC_RST_XDK_N_PIN, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void xbox_hal_delay_ms(uint32_t ms)
{
	HAL_Delay(ms);
}

uint32_t xbox_hal_time_ms(void)
{
	return HAL_GetTick();
}
