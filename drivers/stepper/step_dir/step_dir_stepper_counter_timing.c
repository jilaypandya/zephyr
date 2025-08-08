/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/counter.h>
#include "step_dir_stepper_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(step_dir_stepper);

static int update_counter_top_cfg(const struct device *dev, const uint64_t next_time_interval_ns)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	data->counter_top_cfg.ticks = DIV_ROUND_UP(
		counter_get_frequency(config->counter) * next_time_interval_ns, NSEC_PER_SEC);

	/* Lock interrupts while modifying counter settings */
	int key = irq_lock();

	ret = counter_set_top_value(config->counter, &data->counter_top_cfg);

	irq_unlock(key);

	if (ret != 0) {
		LOG_ERR("%s: Failed to set counter top value (error: %d)", dev->name, ret);
		return ret;
	}
}

static void step_counter_top_interrupt(const struct device *dev, void *user_data)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = user_data;

	switch (data->next_timing_event) {
	case STEP_DIR_TIMING_SOURCE_PULSE_WIDTH_COMPLETED:
		data->timing_event_cb(dev, STEP_DIR_TIMING_SOURCE_PULSE_WIDTH_COMPLETED,
		      data->timing_event_cb_user_data);
		data->next_timing_event = STEP_DIR_TIMING_SOURCE_LOW_LEVEL_WIDTH_COMPLETED;
		update_counter_top_cfg(dev, data->microstep_interval_ns - config->min_pulse_width_ns);
		break;
	case STEP_DIR_TIMING_SOURCE_LOW_LEVEL_WIDTH_COMPLETED:
		data->timing_event_cb(dev, STEP_DIR_TIMING_SOURCE_LOW_LEVEL_WIDTH_COMPLETED,
				      data->timing_event_cb_user_data);
		data->next_timing_event = STEP_DIR_TIMING_SOURCE_PULSE_WIDTH_COMPLETED;
		update_counter_top_cfg(dev, config->min_pulse_width_ns);
		break;
	default:
		break;
	}
	stepper_handle_timing_signal(data->dev);
}

int step_counter_timing_source_update(const struct device *dev,
				      const uint64_t microstep_interval_ns)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;

	if (microstep_interval_ns == 0) {
		return -EINVAL;
	}

	if (microstep_interval_ns > (config->min_pulse_width_ns + config->min_low_level_width_ns)) {
		return -EINVAL;
	}

	return 0;
}

int step_counter_timing_source_start(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	ret = counter_start(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to start counter: %d", ret);
		return ret;
	}

	data->counter_running = true;

	data->timing_event_cb(dev, STEP_DIR_TIMING_SOURCE_EVENT_START,
			      data->timing_event_cb_user_data);
	data->next_timing_event = STEP_DIR_TIMING_SOURCE_PULSE_WIDTH_COMPLETED;
	update_counter_top_cfg(dev, config->min_pulse_width_ns);
	return 0;
}

int step_counter_timing_source_stop(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;
	int ret;

	ret = counter_stop(config->counter);
	if (ret < 0 && ret != -EALREADY) {
		LOG_ERR("Failed to stop counter: %d", ret);
		return ret;
	}

	data->counter_running = false;

	data->timing_event_cb(dev, STEP_DIR_TIMING_SOURCE_EVENT_STOP,
			      data->timing_event_cb_user_data);
	return 0;
}

bool step_counter_timing_source_needs_reschedule(const struct device *dev)
{
	ARG_UNUSED(dev);
	return false;
}

bool step_counter_timing_source_is_running(const struct device *dev)
{
	struct step_dir_stepper_common_data *data = dev->data;

	return data->counter_running;
}

int step_counter_timing_source_init(const struct device *dev)
{
	const struct step_dir_stepper_common_config *config = dev->config;
	struct step_dir_stepper_common_data *data = dev->data;

	if (!device_is_ready(config->counter)) {
		LOG_ERR("Counter device is not ready");
		return -ENODEV;
	}

	data->counter_top_cfg.callback = step_counter_top_interrupt;
	data->counter_top_cfg.user_data = data;
	data->counter_top_cfg.flags = 0;
	data->counter_top_cfg.ticks = counter_us_to_ticks(config->counter, 1000000);

	return 0;
}

int step_counter_set_event_callback(const struct device *dev,
				    stepper_timing_source_event_callback_t cb, void *user_data)
{
	struct step_dir_stepper_common_data *data = dev->data;
	data->timing_event_cb = cb;
	data->timing_event_cb_user_data = user_data;
	return 0;
}

const struct stepper_timing_source_api step_counter_timing_source_api = {
	.init = step_counter_timing_source_init,
	.update = step_counter_timing_source_update,
	.start = step_counter_timing_source_start,
	.needs_reschedule = step_counter_timing_source_needs_reschedule,
	.stop = step_counter_timing_source_stop,
	.is_running = step_counter_timing_source_is_running,
	.set_event_callback = step_counter_set_event_callback,
};
