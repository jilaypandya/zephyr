/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../motion_controller/stepper_motion_controller.h"

#include <zephyr/drivers/gpio.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc22xx, CONFIG_STEPPER_LOG_LEVEL);

#define MSX_PIN_COUNT       2
#define MSX_PIN_STATE_COUNT 4

struct tmc22xx_config
{
	const struct stepper_motion_controller *motion_controller;
	const struct gpio_dt_spec enable_pin;
	const struct gpio_dt_spec step_pin;
	const struct gpio_dt_spec dir_pin;
	const struct gpio_dt_spec *msx_pins;
	enum stepper_micro_step_resolution *msx_resolutions;
};

struct tmc22xx_data
{
	enum stepper_micro_step_resolution resolution;
	int32_t position;

	stepper_event_callback_t event_callback;
	void *event_callback_user_data;
};

static int tmc22xx_stepper_enable(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;

	LOG_DBG("Enabling Stepper motor controller %s", dev->name);
	return gpio_pin_set_dt(&config->enable_pin, 0);
}

static int tmc22xx_stepper_disable(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;

	LOG_DBG("Disabling Stepper motor controller %s", dev->name);
	return gpio_pin_set_dt(&config->enable_pin, 1);
}

static int tmc22xx_stepper_step(const struct device *dev, const enum stepper_direction direction)
{
	const struct tmc22xx_config *config = dev->config;

	// TODO: handle invert direction here
	// TODO: handle dual edge here
	int ret = gpio_pin_set_dt(&config->dir_pin, direction);

	if (ret < 0) {
		LOG_ERR("Failed to set direction pin: %d", ret);
		return ret;
	}
	ret = gpio_pin_set_dt(&config->step_pin, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set step pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_set_dt(&config->step_pin, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set step pin: %d", ret);
		return ret;
	}

	return 0;
}

static int tmc22xx_stepper_set_position(const struct device *dev, const int32_t value)
{
	struct tmc22xx_data *data = dev->data;

	data->position = value;
	return 0;
}

static int tmc22xx_stepper_get_position(const struct device *dev, int32_t *value)
{
	const struct tmc22xx_data *data = dev->data;
	*value = data->position;
	return 0;
}

static int tmc22xx_stepper_move_to(const struct device *dev, const int32_t position)
{
	const struct tmc22xx_data *data = dev->data;

	return stepper_motion_controller_move_by(dev, position - data->position);
}

static int tmc22xx_stepper_run(const struct device *dev, enum stepper_direction direction)
{
	if (direction == STEPPER_DIRECTION_POSITIVE) {
		return stepper_motion_controller_move_by(dev, INT32_MAX);
	}

	if (direction == STEPPER_DIRECTION_NEGATIVE) {
		return stepper_motion_controller_move_by(dev, INT32_MIN);
	}

	LOG_ERR("Invalid direction");
	return -EINVAL;
}

static int tmc22xx_stepper_get_micro_step_res(const struct device *dev,
                                              enum stepper_micro_step_resolution *micro_step_res)
{
	const struct tmc22xx_data *data = dev->data;

	*micro_step_res = data->resolution;
	return 0;
}

static int tmc22xx_stepper_set_micro_step_res(const struct device *dev,
                                              enum stepper_micro_step_resolution micro_step_res)
{
	struct tmc22xx_data *data = dev->data;
	const struct tmc22xx_config *config = dev->config;
	int ret;

	if (!config->msx_pins) {
		LOG_ERR("Microstep resolution pins are not configured");
		return -ENODEV;
	}

	for (uint8_t i = 0; i < MSX_PIN_STATE_COUNT; i++) {
		if (micro_step_res != config->msx_resolutions[i]) {
			continue;
		}

		ret = gpio_pin_set_dt(&config->msx_pins[0], i & 0x01);
		if (ret < 0) {
			LOG_ERR("Failed to set MS1 pin: %d", ret);
			return ret;
		}

		ret = gpio_pin_set_dt(&config->msx_pins[1], (i & 0x02) >> 1);
		if (ret < 0) {
			LOG_ERR("Failed to set MS2 pin: %d", ret);
			return ret;
		}

		data->resolution = micro_step_res;
		return 0;
	}

	LOG_ERR("Unsupported microstep resolution: %d", micro_step_res);
	return -ENOTSUP;
}

static int tmc22xx_stepper_configure_msx_pins(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;
	int ret;

	for (uint8_t i = 0; i < MSX_PIN_COUNT; i++) {
		if (!gpio_is_ready_dt(&config->msx_pins[i])) {
			LOG_ERR("MSX pin %u are not ready", i);
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&config->msx_pins[i], GPIO_OUTPUT);
		if (ret < 0) {
			LOG_ERR("Failed to configure msx pin %u", i);
			return ret;
		}
	}
	return 0;
}

static int tmc22xx_set_event_callback(const struct device *dev, stepper_event_callback_t callback, void *user_data)
{
	struct tmc22xx_data *data = dev->data;

	data->event_callback = callback;
	data->event_callback_user_data = user_data;

	return 0;
}

static void tmc22xx_handle_stepper_event(const struct device *dev, const enum stepper_event event)
{
	const struct tmc22xx_data *data = dev->data;

	data->event_callback(dev, event, data->event_callback_user_data);
}

static int tmc22xx_stepper_init(const struct device *dev)
{
	const struct tmc22xx_config *config = dev->config;
	struct tmc22xx_data *data = dev->data;

	if (!gpio_is_ready_dt(&config->enable_pin)) {
		LOG_ERR("Enable pin is not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->step_pin)) {
		LOG_ERR("Step pin is not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&config->dir_pin)) {
		LOG_ERR("Dir pin is not ready");
		return -ENODEV;
	}

	int ret = gpio_pin_configure_dt(&config->enable_pin, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure enable pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->step_pin, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure step pin: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&config->dir_pin, GPIO_OUTPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure dir pin: %d", ret);
		return ret;
	}

	if (config->msx_pins) {
		ret = tmc22xx_stepper_configure_msx_pins(dev);
		if (ret < 0) {
			LOG_ERR("Failed to configure MSX pins: %d", ret);
			return ret;
		}

		ret = tmc22xx_stepper_set_micro_step_res(dev, data->resolution);
		if (ret < 0) {
			LOG_ERR("Failed to set microstep resolution: %d", ret);
			return ret;
		}
	}

	ret = stepper_motion_controller_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to init step dir common stepper: %d", ret);
		return ret;
	}

	return 0;
}

static DEVICE_API(stepper, tmc22xx_stepper_api) = {
	.enable = tmc22xx_stepper_enable,
	.disable = tmc22xx_stepper_disable,
	.move_by = stepper_motion_controller_move_by,
	.is_moving = stepper_motion_controller_is_moving,
	.set_reference_position = tmc22xx_stepper_set_position,
	.get_actual_position = tmc22xx_stepper_get_position,
	.move_to = tmc22xx_stepper_move_to,
	.run = tmc22xx_stepper_run,
	.stop = stepper_motion_controller_stop,
	.set_event_callback = tmc22xx_set_event_callback,
	.set_micro_step_res = tmc22xx_stepper_set_micro_step_res,
	.get_micro_step_res = tmc22xx_stepper_get_micro_step_res,
	.set_ramp_profile = stepper_motion_controller_set_ramp_profile,
};

#define TMC22XX_STEPPER_DEFINE(inst, msx_table)                                                    \
	STEPPER_MOTION_CONTROLLER_DT_INST_DEFINE(inst,						   \
						 &tmc22xx_stepper_step,				   \
						 &tmc22xx_handle_stepper_event)			   \
	IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, msx_gpios), (                                       \
	static const struct gpio_dt_spec tmc22xx_stepper_msx_pins_##inst[] = {                     \
		DT_INST_FOREACH_PROP_ELEM_SEP(                                                     \
			inst, msx_gpios, GPIO_DT_SPEC_GET_BY_IDX, (,)                              \
		),                                                                                 \
	};                                                                                         \
	BUILD_ASSERT(                                                                              \
		ARRAY_SIZE(tmc22xx_stepper_msx_pins_##inst) == MSX_PIN_COUNT,                      \
		"Two microstep config pins needed");                                               \
	))                                                                                         \
                                                                                                   \
	static const struct tmc22xx_config tmc22xx_config_##inst = {                               \
		.motion_controller = STEPPER_MOTION_CONTROLLER_DT_INST_GET(inst),		   \
		.enable_pin = GPIO_DT_SPEC_INST_GET(inst, en_gpios),	                           \
		.step_pin = GPIO_DT_SPEC_INST_GET(inst, step_gpios),	                           \
		.dir_pin = GPIO_DT_SPEC_INST_GET(inst, dir_gpios),	                           \
		.msx_resolutions = msx_table,                                                      \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(inst, msx_gpios),				   \
		(.msx_pins = tmc22xx_stepper_msx_pins_##inst))					   \
	};                                                                                         \
	static struct tmc22xx_data tmc22xx_data_##inst = {                                         \
		.resolution = DT_INST_PROP(inst, micro_step_res),                                  \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc22xx_stepper_init, NULL, &tmc22xx_data_##inst,              \
			      &tmc22xx_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,   \
			      &tmc22xx_stepper_api);

#define DT_DRV_COMPAT adi_tmc2209
static enum stepper_micro_step_resolution tmc2209_msx_resolutions[MSX_PIN_STATE_COUNT] = {
	STEPPER_MICRO_STEP_8,
	STEPPER_MICRO_STEP_32,
	STEPPER_MICRO_STEP_64,
	STEPPER_MICRO_STEP_16,
};
DT_INST_FOREACH_STATUS_OKAY_VARGS(TMC22XX_STEPPER_DEFINE, tmc2209_msx_resolutions)

#undef DT_DRV_COMPAT
