/**
 * @file drivers/stepper/adi_tmc/adi_tmc5xxx_common.h
 *
 * @brief Common TMC5xxx stepper controller driver definitions
 */

/**
 * SPDX-FileCopyrightText: Copyright (c) 2024 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_COMMON_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_COMMON_H_

#include "adi_tmc_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name TMC5xxx module functions
 * @anchor TMC5XXX_FUNCTIONS
 *
 * @{
 */

/**
 * @brief Calculate the velocity in full clock cycles from the velocity in Hz
 *
 * @param velocity_hz Velocity in Hz
 * @param velocity_fclk Pointer to store the velocity in full clock cycles
 * @param clock_frequency Clock frequency in Hz
 */
inline void tmc5xxx_calculate_velocity_from_hz_to_fclk(const uint32_t velocity_hz,
						       uint32_t *velocity_fclk,
						       const uint32_t clock_frequency)
{
	*velocity_fclk = ((uint64_t)(velocity_hz) << TMC5XXX_CLOCK_FREQ_SHIFT) / clock_frequency;
}

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_ADI_TMC5XXX_COMMON_H_ */
