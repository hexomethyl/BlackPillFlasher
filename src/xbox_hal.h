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

#pragma once

#include <stdint.h>
#include <stdbool.h>

void   xbox_hal_init(void);              /* SPI + SS/SMC pins setup, MISO pull-up */
void   xbox_hal_spi_xfer(const uint8_t *tx, uint8_t *rx, uint32_t len); /* CS stays high around call */
void   xbox_hal_spi_write(const uint8_t *tx, uint32_t len);
void   xbox_hal_ss(bool selected);       /* SPI_SS_N low when selected */
void   xbox_hal_dbg_en(bool asserted);   /* SMC_DBG_EN */
void   xbox_hal_rst_xdk_n(bool high);    /* SMC_RST_XDK_N */
void   xbox_hal_delay_ms(uint32_t ms);
uint32_t xbox_hal_time_ms(void);         /* monotonic ms */
