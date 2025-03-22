/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/bldc.h>
#include <zephyr/kernel.h>
#include <zephyr/input/input.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bldc, CONFIG_BLDC_LOG_LEVEL);

enum bldc_mode {
	BLDC_MODE_ENABLE,
	BLDC_MODE_ROTATE_CW,
	BLDC_MODE_ROTATE_CCW,
	BLDC_MODE_BRAKE,
	BLDC_MODE_DISABLE,
};

static atomic_t bldc_mode = ATOMIC_INIT(BLDC_MODE_DISABLE);

static const struct device *bldc = DEVICE_DT_GET(DT_ALIAS(bldc));

static K_SEM_DEFINE(bldc_generic_sem, 0, 1);

static void button_pressed(struct input_event *event, void *user_data)
{
	ARG_UNUSED(user_data);

	if (event->value == 0 && event->type == INPUT_EV_KEY) {
		return;
	}
	enum bldc_mode mode = atomic_get(&bldc_mode);

	if (mode == BLDC_MODE_DISABLE) {
		atomic_set(&bldc_mode, BLDC_MODE_ENABLE);
	} else {
		atomic_inc(&bldc_mode);
	}
	k_sem_give(&bldc_generic_sem);
}

INPUT_CALLBACK_DEFINE(NULL, button_pressed, NULL);

int main(void)
{
	LOG_INF("Starting generic bldc sample\n");
	if (!device_is_ready(bldc)) {
		LOG_ERR("Device %s is not ready\n", bldc->name);
		return -ENODEV;
	}
	LOG_DBG("bldc is %p, name is %s\n", bldc, bldc->name);

	for (;;) {
		k_sem_take(&bldc_generic_sem, K_FOREVER);
		switch (atomic_get(&bldc_mode)) {
		case BLDC_MODE_ENABLE:
			bldc_enable(bldc);
			LOG_INF("mode: enable\n");
			break;
		case BLDC_MODE_DISABLE:
			bldc_disable(bldc);
			LOG_INF("mode: disable\n");
			break;
		case BLDC_MODE_ROTATE_CW:
			bldc_rotate(bldc, BLDC_DIRECTION_CW);
			LOG_INF("mode: rotate cw\n");
			break;
		case BLDC_MODE_ROTATE_CCW:
			bldc_rotate(bldc, BLDC_DIRECTION_CCW);
			LOG_INF("mode: rotate ccw\n");
			break;
		case BLDC_MODE_BRAKE:
			bldc_brake(bldc);
			LOG_INF("mode: brake\n");
			break;
		}
	}
	return 0;
}