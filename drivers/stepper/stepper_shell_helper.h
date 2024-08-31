/*
 * Copyright (c) 2024, Jilay Sandeep Pandya
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef STEPPER_SHELL_HELPER_H
#define STEPPER_SHELL_HELPER_H

#include <zephyr/drivers/stepper.h>

/**
 * @brief Map of microstep resolution names to values
 */
struct stepper_microstep_map {
	const char *name;
	enum micro_step_resolution microstep;
};

/**
 * @brief Map of stepper direction names to values
 */
struct stepper_direction_map {
	const char *name;
	enum stepper_direction direction;
};

/**
 * @brief Macro to define a stepper direction map entry
 *
 * @param _name Name of the direction
 * @param _dir Direction value
 */
#define STEPPER_DIRECTION_MAP_ENTRY(_name, _dir)                                                   \
	{                                                                                          \
		.name = _name,                                                                     \
		.direction = _dir,                                                                 \
	}

/**
 * @brief Macro to define a stepper microstep map entry
 *
 * @param _name Name of the microstep
 * @param _microstep Microstep value
 */
#define STEPPER_MICROSTEP_MAP(_name, _microstep)                                                   \
	{                                                                                          \
		.name = _name,                                                                     \
		.microstep = _microstep,                                                           \
	}

#endif /* STEPPER_SHELL_HELPER_H */
