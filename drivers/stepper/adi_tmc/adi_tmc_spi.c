/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>

#include "adi_tmc_spi.h"

#define BUFFER_SIZE 5U

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc_spi, CONFIG_SPI_LOG_LEVEL);

static void parse_tmc_spi_status(const uint8_t status_byte)
{
#ifdef CONFIG_STEPPER_ADI_TMC50XX_SPI_LOG_STATUS_BYTE
	uint8_t status_flags[8];
	status_flags[0] = status_byte & BIT(0);
	status_flags[1] = status_byte & BIT(1);
	status_flags[2] = status_byte & BIT(2);
	status_flags[3] = status_byte & BIT(3);
	status_flags[4] = status_byte & BIT(4);
	status_flags[5] = status_byte & BIT(5);
	status_flags[6] = status_byte & BIT(6);
	status_flags[7] = status_byte & BIT(7);

	LOG_HEXDUMP_DBG(status_flags, 8, "reset: | drv_err_1: | drv_err_2: | velocity_reached_1: | velocity_reached_2: | status_stop_l1: | status_stop_l2: |");
#endif
/** Add further status byte parsing here */
}

static void print_tx_rx_buffer(const uint8_t *const tx_buffer, const uint8_t *const rx_buffer)
{
	LOG_HEXDUMP_DBG(tx_buffer, BUFFER_SIZE, "TX: ");
	LOG_HEXDUMP_DBG(rx_buffer, BUFFER_SIZE, "RX: ");
}

int tmc_spi_read_register(const struct spi_dt_spec *bus, const uint8_t read_address_mask,
			  const uint8_t register_address, uint32_t *data)
{
	uint8_t tx_buffer[BUFFER_SIZE] = {read_address_mask & register_address, 0U, 0U, 0U, 0U};
	uint8_t rx_buffer[BUFFER_SIZE];
	int status;

	const struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer,
		.len = sizeof(tx_buffer),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	/** send read with the address byte */
	status = spi_transceive_dt(bus, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (status < 0) {
		return status;
	}

	print_tx_rx_buffer(tx_buffer, rx_buffer);
	parse_tmc_spi_status(rx_buffer[0]);

	/** read the value from the address */
	status = spi_transceive_dt(bus, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (status < 0) {
		return status;
	}

	*data = ((uint32_t)rx_buffer[1] << 24) + ((uint32_t)rx_buffer[2] << 16) +
		((uint32_t)rx_buffer[3] << 8) + (uint32_t)rx_buffer[4];

	print_tx_rx_buffer(tx_buffer, rx_buffer);
	parse_tmc_spi_status(rx_buffer[0]);
	return status;
}

int tmc_spi_write_register(const struct spi_dt_spec *bus, const uint8_t write_bit,
			   const uint8_t register_address, const uint32_t data)
{
	uint8_t tx_buffer[BUFFER_SIZE] = {write_bit | register_address, data >> 24, data >> 16,
					  data >> 8, data};
	uint8_t rx_buffer[BUFFER_SIZE];
	int status;

	const struct spi_buf spi_buffer_tx = {
		.buf = &tx_buffer,
		.len = sizeof(tx_buffer),
	};
	struct spi_buf_set spi_buffer_array_tx = {
		.buffers = &spi_buffer_tx,
		.count = 1U,
	};

	struct spi_buf spi_buffer_rx = {
		.buf = &rx_buffer,
		.len = sizeof(rx_buffer),
	};
	struct spi_buf_set spi_buffer_array_rx = {
		.buffers = &spi_buffer_rx,
		.count = 1U,
	};

	status = spi_transceive_dt(bus, &spi_buffer_array_tx, &spi_buffer_array_rx);
	if (status < 0) {
		return status;
	}

	print_tx_rx_buffer(tx_buffer, rx_buffer);
	parse_tmc_spi_status(rx_buffer[0]);

	return status;
}
