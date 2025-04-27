/**
 * SPDX-FileCopyrightText: Copyright (c) 2024 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "adi_tmc5xxx_common.h"

int tmc5xxx_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val)
{
	const struct tmc5xxx_config *config = dev->config;
	struct tmc5xxx_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = tmc_spi_write_register(&bus, TMC5XXX_WRITE_BIT, reg_addr, reg_val);

	k_sem_give(&data->sem);

	return err;
}

int tmc5xxx_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc5xxx_config *config = dev->config;
	struct tmc5xxx_data *data = dev->data;
	const struct spi_dt_spec bus = config->spi;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = tmc_spi_read_register(&bus, TMC5XXX_ADDRESS_MASK, reg_addr, reg_val);

	k_sem_give(&data->sem);

	return err;
}