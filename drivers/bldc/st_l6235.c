/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_l6235

#include <zephyr/drivers/bldc.h>
#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(st_l6235, CONFIG_BLDC_LOG_LEVEL);

struct st_l6235_config {
	struct gpio_dt_spec en_pin;
	struct gpio_dt_spec dir_pin;
	struct gpio_dt_spec brk_pin;
};

static int st_l6235_enable(const struct device *dev)
{
	const struct st_l6235_config *config = dev->config;

	return gpio_pin_set_dt(&config->en_pin, 1);
}

static int st_l6235_disable(const struct device *dev)
{
	const struct st_l6235_config *config = dev->config;

	return gpio_pin_set_dt(&config->en_pin, 0);
}

static int st_l6235_rotate(const struct device *dev, const enum bldc_direction direction)
{
	const struct st_l6235_config *config = dev->config;
	int ret;

	ret = gpio_pin_set_dt(&config->brk_pin, 1);
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_set_dt(&config->dir_pin, direction);
}

static int st_l6235_brake(const struct device *dev)
{
	const struct st_l6235_config *config = dev->config;

	return gpio_pin_set_dt(&config->brk_pin, 0);
}

static DEVICE_API(bldc, st_l6235_bldc_api) = {
	.enable = st_l6235_enable,
	.disable = st_l6235_disable,
	.rotate = st_l6235_rotate,
	.brake = st_l6235_brake,
};

static int st_l6235_init(const struct device *dev)
{
	const struct st_l6235_config *config = dev->config;
	int ret;

	ret = gpio_pin_configure_dt(&config->en_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure enable pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT_INACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure direction pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->brk_pin, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure brake pin: %d", ret);
		return ret;
	}

	LOG_INF("Initializing ST L6235 BLDC driver");
	return 0;
}

#define ST_L6235_DEVICE(inst)                                                                      \
	static const struct st_l6235_config st_l6235_config_##inst = {                             \
		.en_pin = GPIO_DT_SPEC_INST_GET_OR(inst, en_gpios, {0}),                           \
		.brk_pin = GPIO_DT_SPEC_INST_GET_OR(inst, brk_gpios, {0}),                         \
		.dir_pin = GPIO_DT_SPEC_INST_GET_OR(inst, dir_gpios, {0}),                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, &st_l6235_init, NULL, NULL, &st_l6235_config_##inst,           \
			      POST_KERNEL, CONFIG_BLDC_INIT_PRIORITY, &st_l6235_bldc_api);

DT_INST_FOREACH_STATUS_OKAY(ST_L6235_DEVICE)