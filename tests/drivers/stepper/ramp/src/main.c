/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Andre Stefanov
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <zephyr/ztest.h>

#include <zephyr/drivers/stepper/stepper_ramp.h>
#include "../../../drivers/stepper/ramp/ramp.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ramp, CONFIG_STEPPER_LOG_LEVEL);

struct test_params
{
	uint32_t acceleration;
	uint32_t max_velocity;
	uint32_t deceleration;
	uint32_t pre_deceleration_steps;
	uint32_t steps_to_move;
};

struct reset_ramp_data_expectation
{
	struct stepper_ramp_distance_profile distance_profile;
};

struct get_next_step_interval_expectation
{
	uint64_t *intervals;
	uint32_t count;
};

static void test_reset_ramp_data(struct test_params params,
                                 struct reset_ramp_data_expectation expectation)
{
	struct stepper_ramp_common common = {
		.ramp_profile =
		{
			.acceleration = params.acceleration,
			.max_velocity = params.max_velocity,
			.deceleration = params.deceleration,
		},
		.ramp_distance_profile =
		{
			.acceleration = 0,
			.const_speed = 0,
			.deceleration = 0,
		},
	};

	struct stepper_ramp_config config = {
		.pre_deceleration_steps = params.pre_deceleration_steps,
	};

	trapezoidal_ramp_api.reset_ramp_runtime_data(&config, &common, params.steps_to_move);

	trapezoidal_ramp_api.recalculate_ramp(&common, params.steps_to_move);

	zexpect_equal(common.ramp_distance_profile.acceleration +
	              common.ramp_distance_profile.const_speed +
	              common.ramp_distance_profile.deceleration,
	              params.steps_to_move, "Expected total steps %d but got %d",
	              params.steps_to_move,
	              common.ramp_distance_profile.acceleration +
	              common.ramp_distance_profile.const_speed +
	              common.ramp_distance_profile.deceleration);

	zexpect_equal(common.ramp_distance_profile.acceleration,
	              expectation.distance_profile.acceleration,
	              "Expected acceleration %d but got %d",
	              expectation.distance_profile.acceleration,
	              common.ramp_distance_profile.acceleration);

	zexpect_equal(
		common.ramp_distance_profile.const_speed, expectation.distance_profile.const_speed,
		"Expected const_speed %d but got %d",
		expectation.distance_profile.const_speed,
		common.ramp_distance_profile.const_speed);

	zexpect_equal(common.ramp_distance_profile.deceleration,
	              expectation.distance_profile.deceleration,
	              "Expected deceleration %d but got %d",
	              expectation.distance_profile.deceleration,
	              common.ramp_distance_profile.deceleration);
}

ZTEST_SUITE(ramp, NULL, NULL, NULL, NULL, NULL);

ZTEST(ramp, test_ramp_distance_profile)
{
	// motion of 0 steps should not result in steps in any ramp phase
	LOG_INF("Test zero steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 0,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 0,
				.const_speed = 0,
				.deceleration = 0,
			},
		});

	// motion of 1 step should result in acceleration of 1 step if the requested velocity is higher
	// than the start velocity of the ramp.
	LOG_INF("Test 1 fast step");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 1,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 1,
				.const_speed = 0,
				.deceleration = 0,
			},
		});

	// motion of 1 step should result in a single constant speed step if the requested velocity is lower
	// than the start velocity of the ramp.
	LOG_INF("Test 1 slow step");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 1,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 1,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 0,
				.const_speed = 1,
				.deceleration = 0,
			},
		});

	LOG_INF("Test 2 steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 2,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 1,
				.const_speed = 0,
				.deceleration = 1,
			},
		});

	LOG_INF("Test 3 steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 3,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 2,
				.const_speed = 0,
				.deceleration = 1,
			},
		});

	LOG_INF("Test 100000 steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 1000,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 500,
				.const_speed = 0,
				.deceleration = 500,
			},
		});

	LOG_INF("Test 100001 steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 1001,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 501,
				.const_speed = 0,
				.deceleration = 500,
			},
		});

	LOG_INF("Test 200000 steps");
	test_reset_ramp_data(
		(struct test_params){
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
			.pre_deceleration_steps = 0,
			.steps_to_move = 110000,
		},
		(struct reset_ramp_data_expectation){
			.distance_profile =
			{
				.acceleration = 50000,
				.const_speed = 10000,
				.deceleration = 50000,
			},
		});
}

ZTEST(ramp, test_first_interval)
{
	uint32_t test_accelerations[] = {1, 100, 1000, UINT32_MAX};

	for (size_t i = 0; i < ARRAY_SIZE(test_accelerations); i++) {
		LOG_INF("Test acceleration %u steps/s/s", test_accelerations[i]);
		const uint64_t start_interval = trapezoidal_ramp_api.calculate_start_interval(
			test_accelerations[i]);
		LOG_INF("Start interval in ns: %llu", start_interval);

		// see AVR446 section 2.3.1, exact calculations of the inter-step delay
		const double ideal_start_interval =
			NSEC_PER_SEC * sqrt(2.0 / test_accelerations[i]) * 0.676;
		LOG_INF("Ideal start interval in ns: %f", ideal_start_interval);

		const double ratio = fabs((double)start_interval / ideal_start_interval);
		zassert_within(ratio, 1.0, 0.01);
	}

	LOG_INF("Test acceleration %u steps/s/s", 0);
	const uint64_t invalid_interval = trapezoidal_ramp_api.calculate_start_interval(0);
	zassert_equal(invalid_interval, UINT64_MAX);
}

ZTEST(ramp, test_get_next_step_interval)
{
	struct stepper_ramp_common common = {
		.ramp_profile =
		{
			.acceleration = 1000,
			.max_velocity = 10000,
			.deceleration = 1000,
		},
		.ramp_distance_profile =
		{
			.acceleration = 0,
			.const_speed = 0,
			.deceleration = 0,
		},
		.ramp_runtime_data =
		{
			.ramp_actual_position = 0,
			.ramp_target_position = 0,
			.pre_deceleration_steps = 0,
			.ramp_stop_step_interval_threshold_in_ns = 0,
			.current_ramp_state = STEPPER_RAMP_STATE_NOT_MOVING,
		},
	};

	const struct stepper_ramp_config config = {
		.pre_deceleration_steps = 0,
	};

	trapezoidal_ramp_api.reset_ramp_runtime_data(&config, &common, 1000);

	trapezoidal_ramp_api.recalculate_ramp(&common, 1000);

	uint64_t intervals[500] = {0};

	intervals[0] = trapezoidal_ramp_api.calculate_start_interval(1000);

	for (uint32_t i = 1; i < 500; i++) {
		intervals[i] = trapezoidal_ramp_api.get_next_step_interval(&common, intervals[i-1], STEPPER_RUN_MODE_POSITION);
	}

	for (uint32_t i = 1; i < 500; i++) {
		LOG_INF("Compare %llu vs %llu", intervals[i-1], intervals[i]);
		zexpect(intervals[i-1] > intervals[i], "Acceleration intervals have to decrease over time");
	}

	zexpect(NSEC_PER_SEC / 1000 < intervals[499], "Last ramp interval has to be smaller than the requested velocity");
}
