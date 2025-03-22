/*
 * SPDX-FileCopyrightText: 2025 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file drivers/bldc.h
 * @brief Public API for BLDC Motor Driver
*/

#ifndef ZEPHYR_INCLUDE_DRIVERS_BLDC_H_
#define ZEPHYR_INCLUDE_DRIVERS_BLDC_H_

/**
 * @brief BLDC Motor Driver Interface
 * @defgroup bldc_interface BLDC Motor Driver Interface
 * @since 4.1
 * @version 0.1.0
 * @ingroup io_interfaces
 * @{
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <errno.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLDC Motor direction options
 */
enum bldc_direction {
	/** Clockwise direction */
	BLDC_DIRECTION_CW = 0,
	/** Counter-clockwise direction */
	BLDC_DIRECTION_CCW = 1,
};

/**
 * @cond INTERNAL_HIDDEN
 *
 * Stepper driver API definition and system call entry points.
 *
 */

/**
 * @brief enable the bldc driver.
 *
 * @see bldc_enable() for details.
 */
typedef int (*bldc_enable_t)(const struct device *dev);

/**
 * @brief disable the bldc driver.
 *
 * @see bldc_disable() for details.
 */
typedef int (*bldc_disable_t)(const struct device *dev);

/**
 * @brief rotate the bldc motor in the given direction.
 *
 * @see bldc_rotate() for details.
 */
typedef int (*bldc_rotate_t)(const struct device *dev,
				    const enum bldc_direction direction);

/**
 * @brief brake the bldc motor
 *
 * @see bldc_brake() for details.
 */
typedef int (*bldc_brake_t)(const struct device *dev);

/**
 * @brief BLDC Driver API
 */
__subsystem struct bldc_driver_api {
	bldc_enable_t enable;
	bldc_disable_t disable;
	bldc_rotate_t rotate;
	bldc_brake_t brake;
};

/**
 * @endcond
 */

/**
 * @brief Enable the BLDC driver
 *
 * @details Enabling the bldc driver will set the motor into motion in the selected direction.
 *
 * @param dev pointer to the bldc device instance
 *
 * @retval -EIO Error during Enabling
 * @retval 0 If successful.
 */
__syscall int bldc_enable(const struct device *dev);

static inline int z_impl_bldc_enable(const struct device *dev)
{
	const struct bldc_driver_api *api = (const struct bldc_driver_api *)dev->api;

	return api->enable(dev);
}

/**
 * @brief Disable the BLDC driver
 *
 * @details Disabling the bldc driver will stop the motor and de-energize the coils.
 *
 * @param dev pointer to the bldc device instance
 *
 * @retval -EIO Error during Disabling
 * @retval 0 Success.
 */
__syscall int bldc_disable(const struct device *dev);

static inline int z_impl_bldc_disable(const struct device *dev)
{
	const struct bldc_driver_api *api = (const struct bldc_driver_api *)dev->api;

	return api->disable(dev);
}

/**
 * @brief rotate the BLDC motor in the given direction
 *
 * @param dev pointer to the bldc device instance
 * @param direction The direction to set
 *
 * @retval -EIO General input / output error
 * @retval 0 Success
 */
__syscall int bldc_rotate(const struct device *dev, const enum bldc_direction direction);

static inline int z_impl_bldc_rotate(const struct device *dev, const enum bldc_direction direction)
{
 const struct bldc_driver_api *api = (const struct bldc_driver_api *)dev->api;

 return api->rotate(dev, direction);
}

/**
 * @brief Brake the BLDC motor
 *
 * @param dev pointer to the bldc device instance
 *
 * @retval -EIO General input / output error
 * @retval -ENOSYS If not implemented by device driver
 * @retval 0 Success
 */
__syscall int bldc_brake(const struct device *dev);

static inline int z_impl_bldc_brake(const struct device *dev)
{
	const struct bldc_driver_api *api = (const struct bldc_driver_api *)dev->api;

	if (api->brake == NULL) {
		return -ENOSYS;
	}

	return api->brake(dev);
}

/**
 * @}
 */
#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/bldc.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_BLDC_H_ */