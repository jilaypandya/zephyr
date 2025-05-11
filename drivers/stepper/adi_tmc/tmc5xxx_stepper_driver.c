/*
 * SPDX-FileCopyrightText: Copyright (c) 2024 Carl Zeiss Meditec AG
 * SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_tmc5xxx_stepper_driver

#include <stdlib.h>

#include <zephyr/drivers/stepper_control.h>
#include <zephyr/drivers/stepper.h>
#include <zephyr/drivers/stepper/stepper_trinamic.h>
#include <zephyr/drivers/stepper/adi_tmc_reg.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc50xx_stepper_driver, CONFIG_STEPPER_LOG_LEVEL);

struct tmc5xxx_stepper_config {
	const uint8_t index;
	const uint16_t default_micro_step_res;
	/* parent controller required for bus communication, device pointer to tmc50xx */
	const struct device *controller;
};

static int tmc50xx_stepper_enable(const struct device *dev)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	LOG_DBG("Enabling Stepper driver %s for controller %s", dev->name,
		config->controller->name);

	err = stepper_controller_read(config->controller, TMC50XX_CHOPCONF(config->index),
				      &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value |= TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return stepper_controller_write(config->controller, TMC50XX_CHOPCONF(config->index),
					reg_value);
}

static int tmc50xx_stepper_disable(const struct device *dev)
{
	LOG_DBG("Disabling Stepper motor controller %s", dev->name);
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = stepper_controller_read(config->controller, TMC50XX_CHOPCONF(config->index),
				      &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return stepper_controller_write(config->controller, TMC50XX_CHOPCONF(config->index),
					reg_value);
}

static int tmc50xx_stepper_set_micro_step_res(const struct device *dev,
					      enum stepper_micro_step_resolution res)
{
	if (!VALID_MICRO_STEP_RES(res)) {
		LOG_ERR("Invalid micro step resolution %d", res);
		return -ENOTSUP;
	}

	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = stepper_controller_read(config->controller, TMC50XX_CHOPCONF(config->index),
				      &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value |= ((MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - LOG2(res))
		      << TMC5XXX_CHOPCONF_MRES_SHIFT);

	err = stepper_controller_write(config->controller, TMC50XX_CHOPCONF(config->index),
				       reg_value);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Stepper motor controller %s set micro step resolution to 0x%x", dev->name,
		reg_value);
	return 0;
}

static int tmc50xx_stepper_get_micro_step_res(const struct device *dev,
					      enum stepper_micro_step_resolution *res)
{
	const struct tmc5xxx_stepper_config *config = dev->config;
	uint32_t reg_value;
	int err;

	err = stepper_controller_read(config->controller, TMC50XX_CHOPCONF(config->index),
				      &reg_value);
	if (err != 0) {
		return -EIO;
	}
	reg_value &= TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value >>= TMC5XXX_CHOPCONF_MRES_SHIFT;
	*res = (1 << (MICRO_STEP_RES_INDEX(STEPPER_MICRO_STEP_256) - reg_value));
	LOG_DBG("Stepper motor controller %s get micro step resolution: %d", dev->name, *res);
	return 0;
}

static int tmc50xx_stepper_init(const struct device *dev)
{
	const struct tmc5xxx_stepper_config *stepper_config = dev->config;
	int err;

	LOG_DBG("Controller: %s, Stepper: %s", stepper_config->controller->name, dev->name);

	err = tmc50xx_stepper_set_micro_step_res(dev, stepper_config->default_micro_step_res);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

static DEVICE_API(stepper, tmc50xx_stepper_api) = {
	.enable = tmc50xx_stepper_enable,
	.disable = tmc50xx_stepper_disable,
	.set_micro_step_res = tmc50xx_stepper_set_micro_step_res,
	.get_micro_step_res = tmc50xx_stepper_get_micro_step_res,
};

#define TMC5XXX_STEPPER_DRIVER_DEFINE(inst)                                                        \
	static const struct tmc5xxx_stepper_config tmc5xxx_stepper_config_##inst = {               \
		.controller = DEVICE_DT_GET(DT_GPARENT(DT_DRV_INST(inst))),                        \
		.default_micro_step_res = DT_INST_PROP(inst, micro_step_res),                      \
		.index = DT_REG_ADDR(DT_PARENT(DT_DRV_INST(inst))),                                \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tmc50xx_stepper_init, NULL, NULL,                              \
			      &tmc5xxx_stepper_config_##inst, POST_KERNEL,                         \
			      CONFIG_STEPPER_INIT_PRIORITY, &tmc50xx_stepper_api);

DT_INST_FOREACH_STATUS_OKAY(TMC5XXX_STEPPER_DRIVER_DEFINE)
