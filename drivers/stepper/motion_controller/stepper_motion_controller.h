/*
 * Copyright 2024 Fabian Blatz <fabianblatz@gmail.com>
 * Copyright 2025 Andre Stefanov <mail@andrestefanov.de>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STEPPER_MOTION_CONTROLLER_H
#define STEPPER_MOTION_CONTROLLER_H

/**
 * @brief Stepper Driver APIs
 * @defgroup step_dir_stepper Stepper Driver APIs
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/device.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/counter.h>
#include "timing_source/stepper_timing_source.h"
#include "ramp/stepper_ramp.h"

typedef int (*stepper_motion_controller_step_callback_t)(const struct device *, enum stepper_direction);

typedef void (*stepper_motion_controller_event_callback_t)(const struct device *, enum stepper_event);

/**
 * @brief Common step direction stepper config.
 *
 * This structure **must** be placed first in the driver's config structure.
 */
struct stepper_motion_controller_config
{
	const struct stepper_timing_source_api *timing_source_api;
	struct stepper_timing_source *timing_source;

	const struct stepper_ramp_api *ramp_api;

	const stepper_motion_controller_step_callback_t step_callback;
	const stepper_motion_controller_event_callback_t event_callback;
};

/**
 * @brief Common step direction stepper data.
 *
 * This structure **must** be placed first in the driver's data structure.
 */
struct stepper_motion_controller_data
{
	struct k_spinlock lock;
	enum stepper_direction direction;
	int32_t relative_target_position;

	void *timing_source_data;

	struct avr446_ramp_data ramp_data;
	struct avr446_ramp_profile ramp_profile; // TODO: no need to store ramp profile here
};

struct stepper_motion_controller
{
	const struct stepper_motion_controller_config *const config;
	struct stepper_motion_controller_data *const data;
};

struct stepper_dev_config_base
{
	const struct stepper_motion_controller *controller;
};

#define STEPPER_MOTION_CONTROLLER_DT_INST_DEFINE(inst, step_fn, event_handler_fn)			\
	STEPPER_TIMING_SOURCE_DT_INST_DEFINE(inst)							\
	static struct stepper_motion_controller_data stepper_motion_controller_data_##inst = {		\
		.timing_source_data = STEPPER_TIMING_SOURCE_DT_INST_GET(inst)				\
	};												\
	static const struct stepper_motion_controller_config stepper_motion_controller_cfg_##inst =	\
	{												\
		.timing_source_api = STEPPER_TIMING_SOURCE_DT_SPEC_GET_API(inst),			\
		.timing_source = STEPPER_TIMING_SOURCE_DT_INST_GET(inst),				\
		.ramp_api = RAMP_DT_INST_SPEC_GET_API(inst),						\
		.step_callback = step_fn,								\
		.event_callback = event_handler_fn,							\
	};												\
	static struct stepper_motion_controller stepper_motion_controller_##inst = {			\
		.config = &stepper_motion_controller_cfg_##inst,					\
		.data = &stepper_motion_controller_data_##inst,						\
	};

#define STEPPER_MOTION_CONTROLLER_DT_INST_GET(inst) (&stepper_motion_controller_##inst)

/**
 * @brief Common function to initialize a step direction stepper device at init time.
 *
 * This function must be called at the end of the device init function.
 *
 * @param dev Pointer to the stepper device structure.
 *
 * @retval 0 If initialized successfully.
 * @retval -errno Negative errno in case of failure.
 */
int stepper_motion_controller_init(const struct device *dev);

/**
 * @brief Move the stepper motor by a given number of micro_steps.
 *
 * @param dev Pointer to the device structure.
 * @param micro_steps Number of micro_steps to move. Can be positive or negative.
 * @return 0 on success, or a negative error code on failure.
 */
int stepper_motion_controller_move_by(const struct device *dev, int32_t micro_steps);

/**
 * @brief Check if the stepper motor is still moving.
 *
 * @param dev Pointer to the device structure.
 * @param is_moving Pointer to a boolean where the movement status will be stored.
 * @return 0 on success, or a negative error code on failure.
 */
int stepper_motion_controller_is_moving(const struct device *dev, bool *is_moving);

/**
 * @brief Configures the ramp profile for the stepper motion controller.
 *
 * This function sets the ramp profile parameters for acceleration, deceleration, and
 * minimal step interval (maximum velocity) to the stepper motion controller. It ensures
 * that the provided parameters are non-zero before applying the configuration.
 *
 * @param dev Device instance pointer corresponding to the stepper motor controller.
 * @param ramp_profile Pointer to the structure holding ramp profile parameters to configure.
 *
 * @retval 0 If the configuration is successful.
 * @retval -EINVAL If any of the input ramp profile parameters (acceleration, deceleration,
 * or minimum interval) are zero.
 */
int stepper_motion_controller_set_ramp_profile(const struct device *dev,
                                               const struct stepper_ramp_profile *ramp_profile);

/**
 * @brief Stop the stepper motor.
 *
 * @param dev Pointer to the device structure.
 * @return 0 on success, or a negative error code on failure.
 */
int stepper_motion_controller_stop(const struct device *dev);

/** @} */

#endif /* STEPPER_MOTION_CONTROLLER_H */
