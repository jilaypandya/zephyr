/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz <fabianblatz@gmail.com>
 *			   Copyright (c) 2025 Andre Stefanov <mail@andrestefanov.de>
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stepper_motion_controller.h"

#include <stdlib.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stepper_motion_controller, CONFIG_STEPPER_LOG_LEVEL);

#define CONTROLLER_FROM_DEV(dev) (((const struct stepper_dev_config_base *)dev->config)->controller)

static int stepper_motion_controller_perform_step(const struct device *dev)
{
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	struct stepper_motion_controller_data *data = controller->data;

	const int ret = controller->config->step_callback(dev, data->direction);

	if (ret < 0) {
		LOG_ERR("Failed to step: %d", ret);
		return ret;
	}

	// TODO: move position into stepper
	// TODO: handle overshoots
	if (data->direction == STEPPER_DIRECTION_POSITIVE) {
		data->relative_target_position--;
	} else {
		data->relative_target_position++;
	}

	return 0;
}

void stepper_handle_timing_signal(const void *user_data)
{
	const struct device *dev = user_data;
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	struct stepper_motion_controller_data *data = controller->data;
	const struct stepper_motion_controller_config *config = controller->config;

	int ret = stepper_motion_controller_perform_step(dev);

	if (ret < 0) {
		LOG_ERR("Failed to perform step: %d", ret);
	}

	const uint64_t next_interval = config->ramp_api->get_next_interval(&data->ramp_data);

	if (next_interval > 0) {
		/* the next interval is greater zero, movement is not finished yet */
		K_SPINLOCK(&data->lock) {
			ret = config->timing_source_api->start(
				data->timing_source_data, next_interval);

			if (ret < 0) {
				LOG_ERR("Failed to start timing source: %d", ret);
			}
		}
	} else {
		/* the next interval is zero, movement has finished */
		K_SPINLOCK(&data->lock) {
			ret = config->timing_source_api->stop(data->timing_source_data);

			if (ret < 0) {
				LOG_ERR("Failed to stop timing source: %d", ret);
			}

			LOG_DBG("Motion completed");

			config->event_callback(dev, STEPPER_EVENT_STEPS_COMPLETED);
		}
	}
}

int stepper_motion_controller_init(const struct device *dev)
{
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	const struct stepper_motion_controller_config *config = controller->config;
	struct stepper_motion_controller_data *data = controller->data;

	config->timing_source->callback = &stepper_handle_timing_signal;
	config->timing_source->user_data = dev;

	data->direction = STEPPER_DIRECTION_POSITIVE;

	const int ret = config->timing_source_api->init(config->timing_source);
	if (ret < 0) {
		LOG_ERR("Failed to initialize timing source: %d", ret);
		return ret;
	}

	return 0;
}

// TODO: pass ramp profile as param
int stepper_motion_controller_move_by(const struct device *dev,
                                      const int32_t micro_steps)
{
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	struct stepper_motion_controller_data *data = controller->data;
	const struct stepper_motion_controller_config *config = controller->config;

	K_SPINLOCK(&data->lock) {
		LOG_DBG("Moving by %d microsteps", micro_steps);

		// TODO: "velocity" mode if INT32_MAX or INT32_MIN
		const enum stepper_direction direction = micro_steps < 0
			                                         ? STEPPER_DIRECTION_NEGATIVE
			                                         : STEPPER_DIRECTION_POSITIVE;

		data->relative_target_position += micro_steps;

		if (config->timing_source_api->get_interval(data->timing_source_data) && data->
		    direction != direction) {
			/*
			 * the stepper is moving in the opposite direction, we need to stop the
			 * stepper and after that initiate a movement to the target position
			 */
			const uint64_t stop_steps_count = config->ramp_api->prepare_stop(
				&data->ramp_data,
				&data->ramp_profile);

			if (stop_steps_count > 0) {
				const uint64_t interval = config->ramp_api->get_next_interval(
					&data->ramp_data);
				const int ret = config->timing_source_api->start(
					data->timing_source_data, interval);

				if (ret < 0) {
					LOG_ERR("Failed to start timing source: %d", ret);
				}

				return ret;
			}
		}

		const uint64_t movement_steps_count = config->ramp_api->prepare_move(
			&data->ramp_data,
			&data->ramp_profile,
			abs(micro_steps));

		LOG_DBG("Movement steps count: %llu", movement_steps_count);

		if (movement_steps_count > 0) {
			data->direction = direction;

			const uint64_t interval = config->ramp_api->get_next_interval(
				&data->ramp_data);

			const int ret = config->timing_source_api->start(
				data->timing_source_data, interval);

			if (ret < 0) {
				LOG_ERR("Failed to start timing source: %d", ret);
				return ret;
			}
		} else {
			LOG_DBG("Motion completed");
			config->event_callback(dev, STEPPER_EVENT_STEPS_COMPLETED);
		}
	}

	return 0;
}

int stepper_motion_controller_is_moving(const struct device *dev, bool *is_moving)
{
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	const struct stepper_motion_controller_config *config = controller->config;
	const struct stepper_motion_controller_data *data = controller->data;

	*is_moving = config->timing_source_api->get_interval(data->timing_source_data) != 0;
	return 0;
}

int stepper_motion_controller_set_ramp_profile(const struct device *dev,
                                               const struct stepper_ramp_profile *const
                                               ramp_profile)
{
	if (ramp_profile->acceleration == 0) {
		LOG_ERR("Acceleration cannot be zero");
		return -EINVAL;
	}

	if (ramp_profile->min_interval == 0) {
		LOG_ERR("Max velocity cannot be zero");
		return -EINVAL;
	}

	if (ramp_profile->deceleration == 0) {
		LOG_ERR("Deceleration cannot be zero");
		return -EINVAL;
	}

	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	struct stepper_motion_controller_data *data = controller->data;

	K_SPINLOCK(&data->lock) {
		data->ramp_profile.acceleration_rate = ramp_profile->acceleration;
		data->ramp_profile.run_interval = ramp_profile->min_interval;
		data->ramp_profile.deceleration_rate = ramp_profile->deceleration;
	}

	return 0; // TODO: why not void?
}

int stepper_motion_controller_stop(const struct device *dev)
{
	const struct stepper_motion_controller *controller = CONTROLLER_FROM_DEV(dev);
	const struct stepper_motion_controller_config *config = controller->config;
	const struct stepper_motion_controller_data *data = controller->data;

	const int ret = config->timing_source_api->stop(data->timing_source_data);
	if (ret != 0) {
		LOG_ERR("Failed to stop timing source: %d", ret);
		return ret;
	}

	config->event_callback(dev, STEPPER_EVENT_STOPPED);
	return 0;
}
