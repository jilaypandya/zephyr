/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Andre Stefanov
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stepper_ramp.h"

#include <sys/errno.h>
#include <zephyr/logging/log.h>

#include "zephyr/sys_clock.h"

LOG_MODULE_REGISTER(stepper_ramp, CONFIG_STEPPER_LOG_LEVEL);

/**
 * Computes the integer square root of a given 64-bit unsigned integer using
 * the Babylonian method (also known as Heron's method) for approximation.
 *
 * This function returns the largest integer value whose square is less than
 * or equal to the input value.
 *
 * @param n The 64-bit unsigned integer input for which the integer square root
 *          is to be computed.
 * @return The largest integer that, when squared, does not exceed the input value.
 */
static uint32_t isqrt(const uint64_t n)
{
	if (n <= 1) {
		return (uint32_t)n;
	}

	uint64_t x = n;
	uint64_t y = (x + 1) / 2;

	while (y < x) {
		x = y;
		y = (x + n / x) / 2;
	}
	return (uint32_t)x;
}

static uint64_t avr446_start_interval(const uint32_t acceleration)
{
	if (acceleration == 0) {
		LOG_ERR("Error: Acceleration cannot be zero");
		return 0;
	}

	/* the value of (2 * factor * factor) may not overflow uint64_t but at the same time be as
	 * large as possible to ensure maximal possible precision of isqrt */
	const uint64_t factor = 3037000499ULL;

	/* Calculate the start velocity based on the acceleration
	 *
	 * Using the formula: t = f * sqrt(2 * d / a)
	 * where f = counter frequency, d = 1 step, a = acceleration
	 *
	 * This value will be used in approximation as described in AVR446 section 2.3.1
	 * The approximation introduces an error which has to be corrected by multiplying first
	 * interval by factor of 0.676 The resulting formula is:
	 *
	 * start_interval = f * sqrt(2 / acceleration) * 0.676
	 *
	 * Since division of integer 2 by acceleration is problematic without usage of floating
	 * points,the formula is rewritten as:
	 *
	* start_interval = f * sqrt(2 * factor * factor / acceleration) / factor */
	const uint64_t step_interval_in_ns = NSEC_PER_SEC * 676ULL / 1000ULL *
	                                     isqrt(2ULL * factor * factor / acceleration) / factor;

	return step_interval_in_ns;
}

static uint32_t avr446_acceleration_steps_needed(const uint32_t interval_in_ns,
                                                 const uint32_t acceleration)
{
	if (interval_in_ns == 0) {
		return 0;
	}

	return (NSEC_PER_SEC / interval_in_ns) * (NSEC_PER_SEC / interval_in_ns) / (
		       acceleration * 2);
}

static void avr446_calculate_next_accel_step(struct avr446_ramp_data *ramp)
{
	if (ramp->acceleration_idx++ == 0) {
		ramp->interval_calculation_rest = 0;
		ramp->current_interval = ramp->first_acceleration_interval;
		return;
	}

	const uint64_t numerator = 2 * ramp->current_interval + ramp->interval_calculation_rest;
	const uint64_t denominator = 4 * ramp->acceleration_idx;

	ramp->interval_calculation_rest = numerator % denominator;
	ramp->current_interval -= numerator / denominator;

	ramp->accel_steps_left--;
}

static void avr446_calculate_next_pre_decel_step(struct avr446_ramp_data *ramp)
{
	const uint64_t numerator = 2 * ramp->current_interval + ramp->interval_calculation_rest;
	const uint64_t denominator = 4 * (--ramp->pre_decel_steps_left + ramp->decel_steps_left);

	ramp->interval_calculation_rest = numerator % denominator;
	ramp->current_interval += numerator / denominator;
}

static void avr446_calculate_next_decel_step(struct avr446_ramp_data *ramp)
{
	if (--ramp->decel_steps_left == 0) {
		ramp->interval_calculation_rest = 0;
		ramp->current_interval = ramp->last_deceleration_interval;
		return;
	}

	const uint64_t numerator = 2 * ramp->current_interval + ramp->interval_calculation_rest;
	const uint64_t denominator = 4 * ramp->decel_steps_left;

	ramp->interval_calculation_rest = numerator % denominator;
	ramp->current_interval += numerator / denominator;
}

static uint64_t prepare_move(struct avr446_ramp_data *ramp,
                             const struct avr446_ramp_profile *profile,
                             const uint32_t step_count)
{
	if (!profile) {
		LOG_ERR("Error: Profile cannot be NULL");
		return -EINVAL;
	}

	LOG_DBG("Parameters: current_interval=%llu run_interval=%llu step_count=%u "
	        "acceleration_rate=%u deceleration_rate=%u",
	        ramp->current_interval,
	        profile->run_interval,
	        step_count,
	        profile->acceleration_rate,
	        profile->deceleration_rate);

	// TODO: use accel and decel idx
	ramp->first_acceleration_interval = avr446_start_interval(profile->acceleration_rate);

	ramp->last_deceleration_interval = avr446_start_interval(profile->deceleration_rate);

	/* steps needed to stop from the current velocity */
	const uint32_t stop_lim = avr446_acceleration_steps_needed(
		ramp->current_interval, profile->deceleration_rate);

	/* steps needed to speed up from zero to requested velocity */
	const uint32_t accel_lim = avr446_acceleration_steps_needed(
		profile->run_interval, profile->acceleration_rate);

	/* steps needed to decelerate from the requested velocity to zero */
	const uint32_t decel_lim = avr446_acceleration_steps_needed(
		profile->run_interval, profile->deceleration_rate);

	if (ramp->current_interval != 0 && ramp->current_interval < profile->run_interval) {
		/* the requested velocity is slower than the current one, slow down */

		/* steps needed to decelerate from the current velocity to the requested one */
		ramp->pre_decel_steps_left = stop_lim - decel_lim;

		ramp->accel_steps_left = 0;
		ramp->acceleration_idx = 0;

		const uint32_t total_decel_steps =
			ramp->pre_decel_steps_left + ramp->decel_steps_left;
		if (total_decel_steps < step_count) {
			ramp->run_steps_left = step_count - total_decel_steps;
		} else {
			ramp->run_steps_left = 0;
		}

		ramp->acceleration_idx = accel_lim;

		ramp->decel_steps_left = decel_lim;
	}

	if (ramp->current_interval == 0 || ramp->current_interval > profile->run_interval) {
		/* the requested velocity is faster than the current one, speed up */

		ramp->pre_decel_steps_left = 0;

		/* steps needed to speed up from the current velocity to the requested one */
		ramp->accel_steps_left = accel_lim - stop_lim;

		if (ramp->accel_steps_left + decel_lim >= step_count) {
			ramp->decel_steps_left = step_count * profile->acceleration_rate /
			                         (profile->deceleration_rate +
			                          profile->acceleration_rate);
			ramp->accel_steps_left = step_count - ramp->decel_steps_left;
		} else {
			ramp->decel_steps_left = decel_lim;
		}

		ramp->run_steps_left = step_count - ramp->accel_steps_left - ramp->decel_steps_left;

		ramp->acceleration_idx = 0;
	}

	ramp->run_interval = profile->run_interval;

	LOG_DBG(
		"Distance Profile: pre_decel_steps=%d accel_steps=%d run_steps=%d decel_steps=%d "
		"for steps=%d",
		ramp->pre_decel_steps_left, ramp->accel_steps_left, ramp->run_steps_left,
		ramp->decel_steps_left, step_count);

	return ramp->pre_decel_steps_left
	       + ramp->accel_steps_left
	       + ramp->run_steps_left
	       + ramp->decel_steps_left;
}

static uint64_t prepare_stop(struct avr446_ramp_data *ramp,
                             const struct avr446_ramp_profile *profile)
{
	LOG_DBG("Prepare decelerated stop");

	if (!profile) {
		LOG_ERR("Error: Profile cannot be NULL");
		return -EINVAL;
	}

	/* The deceleration rate may not be zero */
	if (profile->deceleration_rate == 0) {
		LOG_ERR("Error: Deceleration rate cannot be zero");
		return -EINVAL;
	}

	const uint32_t deceleration_steps = avr446_acceleration_steps_needed(
		ramp->current_interval, profile->deceleration_rate);

	ramp->pre_decel_steps_left = 0;
	ramp->accel_steps_left = 0;
	ramp->run_steps_left = 0;
	ramp->run_interval = 0;
	ramp->decel_steps_left = deceleration_steps;

	return deceleration_steps;
}

static uint64_t get_next_interval(struct avr446_ramp_data *ramp)
{
	if (ramp->pre_decel_steps_left > 0) {
		avr446_calculate_next_pre_decel_step(ramp);
	} else if (ramp->accel_steps_left > 0) {
		avr446_calculate_next_accel_step(ramp);
	} else if (ramp->run_steps_left > 0) {
		ramp->run_steps_left--;
		ramp->current_interval = ramp->run_interval;
	} else if (ramp->decel_steps_left > 0) {
		avr446_calculate_next_decel_step(ramp);
	} else {
		/* movement finished */
		ramp->current_interval = 0;
	}

	return ramp->current_interval;
}

const struct stepper_ramp_api trapezoidal_ramp_api = {
	.prepare_move = prepare_move,
	.prepare_stop = prepare_stop,
	.get_next_interval = get_next_interval,
};
