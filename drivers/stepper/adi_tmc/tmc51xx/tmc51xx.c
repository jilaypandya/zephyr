/*
 * SPDX-FileCopyrightText: Copyright (c) 2025 Prevas A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>

#include <zephyr/drivers/stepper.h>

#include <adi_tmc_bus.h>
#include "tmc51xx.h"
#include "../adi_tmc5xxx_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tmc51xx, CONFIG_STEPPER_LOG_LEVEL);

static inline int tmc51xx_bus_check(const struct device *dev)
{
	const struct tmc51xx_config *config = dev->config;

	return config->bus_io->check(&config->bus, config->comm_type);
}

/**
 * @param dev Pointer to tmc51xx motion controller device
 */
static int read_actual_position(const struct device *dev, int32_t *position);
static void rampstat_work_handler(struct k_work *work);
static void tmc51xx_diag0_gpio_callback_handler(const struct device *port, struct gpio_callback *cb,
						gpio_port_pins_t pins);

/**
 * @param dev Pointer to tmc51xx device
 */
static int rampstat_read_clear(const struct device *dev, uint32_t *rampstat_value);

static int tmc51xx_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val)
{
	const struct tmc51xx_config *config = dev->config;
	struct tmc51xx_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->write(dev, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to write register 0x%x with value 0x%x", reg_addr, reg_val);
		return err;
	}
	return 0;
}

static int tmc51xx_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val)
{
	const struct tmc51xx_config *config = dev->config;
	struct tmc51xx_data *data = dev->data;
	int err;

	k_sem_take(&data->sem, K_FOREVER);

	err = config->bus_io->read(dev, reg_addr, reg_val);

	k_sem_give(&data->sem);

	if (err < 0) {
		LOG_ERR("Failed to read register 0x%x", reg_addr);
		return err;
	}
	return 0;
}

static int tmc51xx_stepper_set_event_callback(const struct device *dev,
					      stepper_event_callback_t callback, void *user_data)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	struct tmc51xx_motion_controller_data *data = dev->data;
	const struct device *controller = config->controller;
	__maybe_unused const struct tmc51xx_config *comm_config = controller->config;
	__maybe_unused struct tmc51xx_data *comm_data = controller->data;
	__maybe_unused int err;

	data->callback = callback;
	data->event_cb_user_data = user_data;

	/* Configure DIAG0 GPIO interrupt pin */
	IF_ENABLED(TMC51XX_BUS_SPI, ({
	if ((comm_config->comm_type == TMC_COMM_SPI) && comm_config->diag0_gpio.port) {
		LOG_INF("Configuring DIAG0 GPIO interrupt pin");
		if (!gpio_is_ready_dt(&comm_config->diag0_gpio)) {
			LOG_ERR("DIAG0 interrupt GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&comm_config->diag0_gpio, GPIO_INPUT);
		if (err < 0) {
			LOG_ERR("Could not configure DIAG0 GPIO (%d)", err);
			return err;
		}
		k_work_init_delayable(&comm_data->rampstat_callback_dwork, rampstat_work_handler);

		err = gpio_pin_interrupt_configure_dt(&comm_config->diag0_gpio, GPIO_INT_EDGE_RISING);
		if (err) {
			LOG_ERR("failed to configure DIAG0 interrupt (err %d)", err);
			return -EIO;
		}

		/* Initialize and add GPIO callback */
		gpio_init_callback(&comm_data->diag0_cb, tmc51xx_diag0_gpio_callback_handler,
				   BIT(comm_config->diag0_gpio.pin));

		err = gpio_add_callback(comm_config->diag0_gpio.port, &comm_data->diag0_cb);
		if (err < 0) {
			LOG_ERR("Could not add DIAG0 pin GPIO callback (%d)", err);
			return -EIO;
		}

		/* Clear any pending interrupts */
		uint32_t rampstat_value;

		err = rampstat_read_clear(controller, &rampstat_value);
		if (err != 0) {
			return -EIO;
		}
	}}))
	return 0;
}

static int tmc51xx_set_stepper_drv_event_callback(const struct device *stepper,
						  stepper_drv_event_cb_t callback, void *user_data)
{
	struct tmc51xx_stepper_data *data = stepper->data;

	data->drv_event_cb = callback;
	data->drv_event_cb_user_data = user_data;

	return 0;
}

static int read_vactual(const struct device *dev, int32_t *actual_velocity)
{
	__ASSERT(actual_velocity != NULL, "actual_velocity pointer must not be NULL");
	const struct tmc51xx_motion_controller_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;
	uint32_t raw_value;

	err = tmc51xx_read(controller, TMC51XX_VACTUAL, &raw_value);
	if (err) {
		LOG_ERR("Failed to read VACTUAL register");
		return err;
	}

	*actual_velocity = sign_extend(raw_value, TMC_RAMP_VACTUAL_SHIFT);
	if (*actual_velocity) {
		LOG_DBG("actual velocity: %d", *actual_velocity);
	}
	return 0;
}

static int stallguard_enable(const struct device *dev, const bool enable)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	const struct device *controller   = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(controller, TMC51XX_SWMODE, &reg_value);
	if (err) {
		LOG_ERR("Failed to read SWMODE register");
		return -EIO;
	}

	if (enable) {
		reg_value |= TMC5XXX_SW_MODE_SG_STOP_ENABLE;

		int32_t actual_velocity;

		err = read_vactual(dev, &actual_velocity);
		if (err) {
			return -EIO;
		}
		if (abs(actual_velocity) < config->sg_threshold_velocity) {
			return -EAGAIN;
		}
	} else {
		reg_value &= ~TMC5XXX_SW_MODE_SG_STOP_ENABLE;
	}
	err = tmc51xx_write(controller, TMC51XX_SWMODE, reg_value);
	if (err) {
		LOG_ERR("Failed to write SWMODE register");
		return -EIO;
	}

	LOG_DBG("Stallguard %s", enable ? "enabled" : "disabled");
	return 0;
}

static void stallguard_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct tmc51xx_motion_controller_data const *data =
		CONTAINER_OF(dwork, struct tmc51xx_motion_controller_data, stallguard_dwork);
	const struct device *dev = data->dev;
	const struct tmc51xx_motion_controller_config *config = dev->config;
	int err;

	err = stallguard_enable(dev, true);
	if (err == -EAGAIN) {
		k_work_reschedule(dwork, K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (err == -EIO) {
		LOG_ERR("Failed to enable stallguard because of I/O error");
	}
}

static void driver_trigger_cb(const struct device *dev, const enum stepper_event event)
{
	struct tmc51xx_stepper_data *data = dev->data;

	if (!data->drv_event_cb) {
		LOG_WRN_ONCE("No %s callback registered", "stepper driver");
		return;
	}
	data->drv_event_cb(dev, event, data->drv_event_cb_user_data);
}


static void motion_controller_trigger_cb(const struct device *dev, const enum stepper_event event)
{
	struct tmc51xx_motion_controller_data *data = dev->data;

	if (!data->callback) {
		LOG_WRN_ONCE("No %s callback registered", "motion controller");
		return;
	}
	data->callback(dev, event, data->event_cb_user_data);
}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_STALLGUARD_LOG

static void log_stallguard(const struct device *dev, const uint32_t drv_status)
{
	int32_t position;
	int err;

	err = read_actual_position(dev, &position);
	if (err != 0) {
		LOG_ERR("%s: Failed to read XACTUAL register", dev->name);
		return;
	}

	const uint8_t sg_result = FIELD_GET(TMC5XXX_DRV_STATUS_SG_RESULT_MASK, drv_status);
	const bool sg_status = FIELD_GET(TMC5XXX_DRV_STATUS_SG_STATUS_MASK, drv_status);

	LOG_DBG("%s position: %d | sg result: %3d status: %d", dev->name, position, sg_result,
		sg_status);
}

#endif /* CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_STALLGUARD_LOG */

static int rampstat_read_clear(const struct device *dev, uint32_t *rampstat_value)
{
	int err;

	err = tmc51xx_read(dev, TMC51XX_RAMPSTAT, rampstat_value);
	if (err == 0) {
		err = tmc51xx_write(dev, TMC51XX_RAMPSTAT, *rampstat_value);
	}
	return err;
}

static void rampstat_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);

	struct tmc51xx_data *stepper_data =
		CONTAINER_OF(dwork, struct tmc51xx_data, rampstat_callback_dwork);
	const struct device *dev = stepper_data->dev;
	__maybe_unused const struct tmc51xx_config *config = dev->config;
	const struct device *motion_controller = config->motion_controller;
	const struct device *stepper_driver = config->stepper_driver;

	__ASSERT_NO_MSG(dev);

	uint32_t drv_status;
	int err;

	err = tmc51xx_read(dev, TMC51XX_DRVSTATUS, &drv_status);
	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return;
	}
#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_STALLGUARD_LOG
	log_stallguard(dev, drv_status);
#endif
	if (FIELD_GET(TMC5XXX_DRV_STATUS_SG_STATUS_MASK, drv_status) == 1U) {
		LOG_INF("%s: Stall detected", dev->name);
		err = tmc51xx_write(dev, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_HOLD_MODE);
		if (err != 0) {
			LOG_ERR("%s: Failed to stop motor", dev->name);
			return;
		}
	}

	uint32_t rampstat_value;

	err = rampstat_read_clear(dev, &rampstat_value);
	if (err != 0) {
		LOG_ERR("%s: Failed to read RAMPSTAT register", dev->name);
		return;
	}

	const uint8_t ramp_stat_values = FIELD_GET(TMC5XXX_RAMPSTAT_INT_MASK, rampstat_value);

	if (ramp_stat_values > 0) {
		switch (ramp_stat_values) {
		case TMC5XXX_STOP_LEFT_EVENT:
			LOG_DBG("RAMPSTAT %s:Left end-stop detected", dev->name);
			motion_controller_trigger_cb(motion_controller, STEPPER_EVENT_LEFT_END_STOP_DETECTED);
			break;

		case TMC5XXX_STOP_RIGHT_EVENT:
			LOG_DBG("RAMPSTAT %s:Right end-stop detected", dev->name);
			motion_controller_trigger_cb(motion_controller, STEPPER_EVENT_RIGHT_END_STOP_DETECTED);
			break;

		case TMC5XXX_POS_REACHED_EVENT:
		case TMC5XXX_POS_REACHED:
		case TMC5XXX_POS_REACHED_AND_EVENT:
			LOG_DBG("RAMPSTAT %s:Position reached", dev->name);
			motion_controller_trigger_cb(motion_controller, STEPPER_EVENT_STEPS_COMPLETED);
			break;

		case TMC5XXX_STOP_SG_EVENT:
			LOG_DBG("RAMPSTAT %s:Stall detected", dev->name);
			stallguard_enable(dev, false);
			driver_trigger_cb(stepper_driver, STEPPER_DRV_EVENT_STALL_DETECTED);
			break;
		default:
			LOG_ERR("Illegal ramp stat bit field 0x%x", ramp_stat_values);
			break;
		}
	} else {
		/* For SPI with DIAG0 pin, we use interrupt-driven approach */
		IF_ENABLED(TMC51XX_BUS_SPI, ({
			if (config->comm_type == TMC_COMM_SPI && config->diag0_gpio.port) {
				/* Using interrupt-driven approach - no polling needed */
				return;
			}
			}))

		/* For UART or SPI without DIAG0, reschedule RAMPSTAT polling */
#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC
		k_work_reschedule(
			&stepper_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
#endif /* CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC */
	}
}

static void __maybe_unused tmc51xx_diag0_gpio_callback_handler(const struct device *port,
							       struct gpio_callback *cb,
							       gpio_port_pins_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	struct tmc51xx_data *stepper_data = CONTAINER_OF(cb, struct tmc51xx_data, diag0_cb);

	k_work_reschedule(&stepper_data->rampstat_callback_dwork, K_NO_WAIT);
}

static int tmc51xx_stepper_enable(const struct device *dev)
{
	const struct tmc51xx_stepper_config *config = dev->config;
	const struct device *controller   = config->controller;
	uint32_t reg_value;
	int err;

	LOG_DBG("Enabling Stepper motor controller %s", dev->name);

	err = tmc51xx_read(controller, TMC51XX_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value |= TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc51xx_write(controller, TMC51XX_CHOPCONF, reg_value);
}

static int tmc51xx_stepper_disable(const struct device *dev)
{
	LOG_DBG("Disabling Stepper motor controller %s", dev->name);
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(dev, TMC51XX_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_DRV_ENABLE_MASK;

	return tmc51xx_write(dev, TMC51XX_CHOPCONF, reg_value);
}

static int tmc51xx_stepper_is_moving(const struct device *dev, bool *is_moving)
{
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(dev, TMC51XX_DRVSTATUS, &reg_value);

	if (err != 0) {
		LOG_ERR("%s: Failed to read DRVSTATUS register", dev->name);
		return -EIO;
	}

	*is_moving = (FIELD_GET(TMC5XXX_DRV_STATUS_STST_BIT, reg_value) != 1U);
	LOG_DBG("Stepper motor controller %s is moving: %d", dev->name, *is_moving);
	return 0;
}

int tmc51xx_stepper_set_max_velocity(const struct device *dev, uint32_t velocity)
{
	const struct tmc51xx_config *config = dev->config;
	const uint32_t clock_frequency = config->clock_frequency;
	uint32_t velocity_fclk;
	int err;

	velocity_fclk = tmc5xxx_calculate_velocity_from_hz_to_fclk(velocity, clock_frequency);

	err = tmc51xx_write(dev, TMC51XX_VMAX, velocity_fclk);
	if (err != 0) {
		LOG_ERR("%s: Failed to set max velocity", dev->name);
		return -EIO;
	}
	return 0;
}

static int tmc51xx_stepper_set_micro_step_res(const struct device *dev,
					      enum stepper_drv_micro_step_resolution res)
{
	const struct tmc51xx_stepper_config *config = dev->config;
	const struct device *controller   = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(controller, TMC51XX_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}

	reg_value &= ~TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value |= ((MICRO_STEP_RES_INDEX(STEPPER_DRV_MICRO_STEP_256) - LOG2(res))
		      << TMC5XXX_CHOPCONF_MRES_SHIFT);

	err = tmc51xx_write(controller, TMC51XX_CHOPCONF, reg_value);
	if (err != 0) {
		return -EIO;
	}

	LOG_DBG("Stepper motor controller %s set micro step resolution to 0x%x", dev->name,
		reg_value);
	return 0;
}

static int tmc51xx_stepper_get_micro_step_res(const struct device *dev,
					      enum stepper_drv_micro_step_resolution *res)
{
	const struct tmc51xx_stepper_config *config = dev->config;
	const struct device *controller   = config->controller;
	uint32_t reg_value;
	int err;

	err = tmc51xx_read(controller, TMC51XX_CHOPCONF, &reg_value);
	if (err != 0) {
		return -EIO;
	}
	reg_value &= TMC5XXX_CHOPCONF_MRES_MASK;
	reg_value >>= TMC5XXX_CHOPCONF_MRES_SHIFT;
	*res = (1 << (MICRO_STEP_RES_INDEX(STEPPER_DRV_MICRO_STEP_256) - reg_value));
	LOG_DBG("Stepper motor controller %s get micro step resolution: %d", dev->name, *res);
	return 0;
}

static int tmc51xx_stepper_set_reference_position(const struct device *dev, const int32_t position)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	const struct device *controller   = config->controller;
	int err;

	err = tmc51xx_write(controller, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_HOLD_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_write(controller, TMC51XX_XACTUAL, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Stepper motor controller %s set actual position to %d", dev->name, position);
	return 0;
}

static int read_actual_position(const struct device *dev, int32_t *position)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	const struct device *comm_device = config->controller;
	const struct tmc51xx_config *comm_config = comm_device->config;
	int err;
	uint32_t raw_value;

	/* Check if device is using UART and is currently moving */
	if (comm_config->comm_type == TMC_COMM_UART) {
		bool is_moving;

		err = tmc51xx_stepper_is_moving(dev, &is_moving);
		if (err != 0) {
			return -EIO;
		}

		if (is_moving) {
			LOG_WRN("%s: Reading position while moving over UART is not supported",
				dev->name);
			return -ENOTSUP;
		}
	}

	err = tmc51xx_read(comm_device, TMC51XX_XACTUAL, &raw_value);
	if (err != 0) {
		return -EIO;
	}

	*position = sign_extend(raw_value, TMC_RAMP_XACTUAL_SHIFT);
	return 0;
}

static int tmc51xx_stepper_get_actual_position(const struct device *dev, int32_t *position)
{
	int err;

	err = read_actual_position(dev, position);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("%s actual position: %d", dev->name, *position);
	return 0;
}

static int tmc51xx_stepper_move_to(const struct device *dev, const int32_t micro_steps)
{
	LOG_DBG("%s set target position to %d", dev->name, micro_steps);
	const struct tmc51xx_motion_controller_config *config = dev->config;
	struct tmc51xx_motion_controller_data *data = dev->data;
	const struct device *comm_device = config->controller;
	const struct tmc51xx_config *comm_config = comm_device->config;
	__maybe_unused struct tmc51xx_data *comm_data = comm_device->data;
	int err;

	if (config->is_sg_enabled) {
		stallguard_enable(dev, false);
	}

	err = tmc51xx_write(comm_device, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_POSITIONING_MODE);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(comm_device, TMC51XX_XTARGET, micro_steps);
	if (err != 0) {
		return -EIO;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use interrupt-driven approach */
		IF_ENABLED(TMC51XX_BUS_SPI, ({
		if (comm_config->comm_type == TMC_COMM_SPI && comm_config->diag0_gpio.port) {
			/* Using interrupt-driven approach - no polling needed */
			return 0;
		}
		}))

		/* For UART or SPI without DIAG0, reschedule RAMPSTAT polling */
#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC
		k_work_reschedule(
			&comm_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
#endif /* CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC */
	}
	return 0;
}

static int tmc51xx_stepper_move_by(const struct device *dev, const int32_t micro_steps)
{
	int err;
	int32_t position;

	err = tmc51xx_stepper_get_actual_position(dev, &position);
	if (err != 0) {
		return -EIO;
	}
	int32_t target_position = position + micro_steps;

	LOG_DBG("%s moved to %d by steps: %d", dev->name, target_position, micro_steps);

	return tmc51xx_stepper_move_to(dev, target_position);
}

static int tmc51xx_stepper_run(const struct device *dev, const enum stepper_direction direction)
{
	LOG_DBG("Stepper motor controller %s run", dev->name);
	const struct tmc51xx_motion_controller_config *config = dev->config;
	struct tmc51xx_motion_controller_data *data = dev->data;
	const struct device *comm_device = config->controller;
	__maybe_unused const struct tmc51xx_config *comm_config = comm_device->config;
	struct tmc51xx_data *comm_data = comm_device->data;
	int err;

	if (config->is_sg_enabled) {
		err = stallguard_enable(dev, false);
		if (err != 0) {
			return -EIO;
		}
	}

	switch (direction) {
	case STEPPER_DIRECTION_POSITIVE:
		err = tmc51xx_write(dev, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;

	case STEPPER_DIRECTION_NEGATIVE:
		err = tmc51xx_write(dev, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_NEGATIVE_VELOCITY_MODE);
		if (err != 0) {
			return -EIO;
		}
		break;
	}

	if (config->is_sg_enabled) {
		k_work_reschedule(&data->stallguard_dwork,
				  K_MSEC(config->sg_velocity_check_interval_ms));
	}
	if (data->callback) {
		/* For SPI with DIAG0 pin, we use interrupt-driven approach */
		IF_ENABLED(TMC51XX_BUS_SPI, ({
		if (comm_config->comm_type == TMC_COMM_SPI && comm_config->diag0_gpio.port) {
			/* Using interrupt-driven approach - no polling needed */
			return 0;
		}
		}))

		/* For UART or SPI without DIAG0, reschedule RAMPSTAT polling */
#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC
		k_work_reschedule(
			&comm_data->rampstat_callback_dwork,
			K_MSEC(CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC));
#endif /* CONFIG_STEPPER_ADI_TMC51XX_RAMPSTAT_POLL_INTERVAL_IN_MSEC */
	}
	return 0;
}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN

int tmc51xx_stepper_set_ramp(const struct device *dev,
			     const struct tmc_ramp_generator_data *ramp_data)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	const struct device* controller = config->controller;
	LOG_DBG("Stepper motor controller %s set ramp", dev->name);
	int err;

	err = tmc51xx_write(controller, TMC51XX_VSTART, ramp_data->vstart);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_A1, ramp_data->a1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_AMAX, ramp_data->amax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_D1, ramp_data->d1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_DMAX, ramp_data->dmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_V1, ramp_data->v1);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_VMAX, ramp_data->vmax);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_VSTOP, ramp_data->vstop);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TZEROWAIT, ramp_data->tzerowait);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_THIGH, ramp_data->thigh);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TCOOLTHRS, ramp_data->tcoolthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TPWMTHRS, ramp_data->tpwmthrs);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_TPOWER_DOWN, ramp_data->tpowerdown);
	if (err != 0) {
		return -EIO;
	}
	err = tmc51xx_write(controller, TMC51XX_IHOLD_IRUN, ramp_data->iholdrun);
	if (err != 0) {
		return -EIO;
	}
	return 0;
}

#endif

static int tmc51xx_init(const struct device *dev)
{
	const struct tmc51xx_config *config = dev->config;
	struct tmc51xx_data *data = dev->data;
	int err;

	LOG_DBG("Initializing TMC51XX stepper motor controller %s, stepper motor driver %s",
	config->motion_controller->name, config->stepper_driver->name);
	k_sem_init(&data->sem, 1, 1);

	err = tmc51xx_bus_check(dev);
	if (err < 0) {
		LOG_ERR("Bus not ready for '%s'", dev->name);
		return err;
	}

#if TMC51XX_BUS_UART
	/* Initialize SW_SEL GPIO if using UART and GPIO is specified */
	if (config->comm_type == TMC_COMM_UART && config->sw_sel_gpio.port) {
		if (!gpio_is_ready_dt(&config->sw_sel_gpio)) {
			LOG_ERR("SW_SEL GPIO not ready");
			return -ENODEV;
		}

		err = gpio_pin_configure_dt(&config->sw_sel_gpio, GPIO_OUTPUT_ACTIVE);
		if (err < 0) {
			LOG_ERR("Failed to configure SW_SEL GPIO");
			return err;
		}
	}
#endif

	LOG_DBG("GCONF: %d", config->gconf);
	err = tmc51xx_write(dev, TMC5XXX_GCONF, config->gconf);
	if (err != 0) {
		return -EIO;
	}
	uint32_t gconf;
	err = tmc51xx_read(dev, TMC5XXX_GCONF, &gconf);

	/* Read and write GSTAT register to clear any SPI Datagram errors. */
	uint32_t gstat_value;

	err = tmc51xx_read(dev, TMC5XXX_GSTAT, &gstat_value);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_write(dev, TMC5XXX_GSTAT, gstat_value);
	if (err != 0) {
		return -EIO;
	}

	k_work_init_delayable(&data->rampstat_callback_dwork, rampstat_work_handler);
	uint32_t rampstat_value;
	(void)rampstat_read_clear(dev, &rampstat_value);

	return 0;
}

static int tmc51xx_stepper_init(const struct device *dev)
{
	const struct tmc51xx_stepper_config *config = dev->config;
	const struct device *controller = config->controller;
	int err;

	if (!IN_RANGE(config->sg_threshold, TMC5XXX_SG_MIN_VALUE, TMC5XXX_SG_MAX_VALUE)) {
		LOG_ERR("Stallguard threshold out of range");
		return -EINVAL;
	}

	int32_t stall_guard_threshold = (int32_t)config->sg_threshold;

	err = tmc51xx_write(controller, TMC51XX_COOLCONF,
			stall_guard_threshold << TMC5XXX_COOLCONF_SG2_THRESHOLD_VALUE_SHIFT);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_stepper_set_micro_step_res(dev, config->default_micro_step_res);
	if (err != 0) {
		return -EIO;
	}
	LOG_DBG("Setting stallguard %d", config->sg_threshold);
	return 0;
}

static int tmc51xx_motion_controller_init(const struct device *dev)
{
	const struct tmc51xx_motion_controller_config *config = dev->config;
	struct tmc51xx_motion_controller_data *data = dev->data;
	const struct device *controller = config->controller;
	int err;

	data->dev = dev;

	if (config->is_sg_enabled) {
		k_work_init_delayable(&data->stallguard_dwork, stallguard_work_handler);

		err = tmc51xx_write(controller, TMC51XX_SWMODE, BIT(10));
		if (err != 0) {
			return -EIO;
		}

		LOG_DBG("stallguard delay %d ms", config->sg_velocity_check_interval_ms);

		k_work_reschedule(&data->stallguard_dwork, K_NO_WAIT);
	}

#ifdef CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN
	err = tmc51xx_stepper_set_ramp(dev, &config->default_ramp_config);
	if (err != 0) {
		return -EIO;
	}
#endif
	return 0;
}

static int tmc51xx_stepper_stop(const struct device *dev)
{
	int err;

	err = tmc51xx_write(dev, TMC51XX_RAMPMODE, TMC5XXX_RAMPMODE_POSITIVE_VELOCITY_MODE);
	if (err != 0) {
		return -EIO;
	}

	err = tmc51xx_write(dev, TMC51XX_VMAX, 0);
	if (err != 0) {
		return -EIO;
	}

	return 0;
}

static DEVICE_API(stepper_drv, tmc51xx_stepper_drv_api) = {
	.enable = tmc51xx_stepper_enable,
	.disable = tmc51xx_stepper_disable,
	.set_micro_step_res = tmc51xx_stepper_set_micro_step_res,
	.get_micro_step_res = tmc51xx_stepper_get_micro_step_res,
	.set_event_cb = tmc51xx_set_stepper_drv_event_callback,
};

static DEVICE_API(stepper, tmc51xx_stepper_api) = {
	.is_moving = tmc51xx_stepper_is_moving,
	.move_by = tmc51xx_stepper_move_by,
	.set_reference_position = tmc51xx_stepper_set_reference_position,
	.get_actual_position = tmc51xx_stepper_get_actual_position,
	.move_to = tmc51xx_stepper_move_to,
	.run = tmc51xx_stepper_run,
	.stop = tmc51xx_stepper_stop,
	.set_event_callback = tmc51xx_stepper_set_event_callback,
};

#define DT_CHILD_BY_COMPATIBLE(parent_node_id, compat)						\
	DT_FOREACH_CHILD_STATUS_OKAY_VARGS(parent_node_id, _DT_CHILD_BY_COMPAT_HELPER, compat)

#define _DT_CHILD_BY_COMPAT_HELPER(node_id, compat)						\
	COND_CODE_1(DT_NODE_HAS_COMPAT(node_id, compat), (node_id), ())

#define TMC51XX_STEPPER_DRIVER_DEFINE(inst)							\
	COND_CODE_1(DT_PROP_EXISTS(inst, stallguard_threshold_velocity),			\
	BUILD_ASSERT(DT_PROP(inst, stallguard_threshold_velocity),				\
			"stallguard threshold velocity must be a positive value"), ());		\
	static const struct tmc51xx_stepper_config tmc51xx_stepper_config_##inst = {		\
		.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),			\
		.default_micro_step_res = DT_INST_PROP(inst, micro_step_res),			\
		.sg_threshold = DT_INST_PROP(inst, stallguard2_threshold),			\
	};											\
	static struct tmc51xx_stepper_data tmc51xx_stepper_data_##inst;				\
	DEVICE_DT_INST_DEFINE(inst, tmc51xx_stepper_init, NULL, &tmc51xx_stepper_data_##inst,	\
			 &tmc51xx_stepper_config_##inst, POST_KERNEL,				\
			 CONFIG_STEPPER_INIT_PRIORITY, &tmc51xx_stepper_drv_api);

#define TMC51XX_MOTION_CONTROLLER_DEFINE(inst)							\
	IF_ENABLED(CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN,	(CHECK_RAMP_DT_DATA(inst)));		\
	static const struct tmc51xx_motion_controller_config tmc51xx_motion_ctrl_cfg_##inst = {	\
	.controller = DEVICE_DT_GET(DT_PARENT(DT_DRV_INST(inst))),				\
	.sg_threshold_velocity = DT_INST_PROP(inst, stallguard_threshold_velocity),		\
	.sg_velocity_check_interval_ms =							\
			DT_INST_PROP(inst, stallguard_velocity_check_interval_ms),		\
	.is_sg_enabled = DT_INST_PROP(inst, activate_stallguard2),				\
	IF_ENABLED(CONFIG_STEPPER_ADI_TMC51XX_RAMP_GEN,						\
		(.default_ramp_config = TMC_RAMP_DT_SPEC_GET_TMC51XX(inst)))			\
	};											\
	static struct tmc51xx_motion_controller_data tmc51xx_motion_controller_data_##inst;	\
	DEVICE_DT_INST_DEFINE(inst, tmc51xx_motion_controller_init, NULL,			\
				&tmc51xx_motion_controller_data_##inst,				\
				&tmc51xx_motion_ctrl_cfg_##inst, POST_KERNEL,			\
				CONFIG_STEPPER_INIT_PRIORITY, &tmc51xx_stepper_api);

/* Initializes a struct tmc51xx_config for an instance on a SPI bus. */
#define TMC51XX_CONFIG_SPI(inst)                                                                   \
	.comm_type = TMC_COMM_SPI,                                                                 \
	.bus.spi = SPI_DT_SPEC_INST_GET(inst,                                                      \
					(SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_MODE_CPOL |   \
					 SPI_MODE_CPHA | SPI_WORD_SET(8))),                        \
	.bus_io = &tmc51xx_spi_bus_io,                                                             \
	.diag0_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, diag0_gpios, {0})

/* Initializes a struct tmc51xx_config for an instance on a UART bus. */
#define TMC51XX_CONFIG_UART(inst)                                                                  \
	.comm_type = TMC_COMM_UART, .bus.uart = DEVICE_DT_GET(DT_INST_BUS(inst)),                  \
	.bus_io = &tmc51xx_uart_bus_io, .uart_addr = DT_INST_PROP_OR(inst, uart_device_addr, 1U),  \
	.sw_sel_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, sw_sel_gpios, {0})

/* Device initialization macros */
#define TMC51XX_DEFINE(inst)                                                                       \
	BUILD_ASSERT((DT_INST_PROP(inst, clock_frequency) > 0),                                    \
		     "clock frequency must be non-zero positive value");                           \
	static struct tmc51xx_data tmc51xx_data_##inst = {                                         \
		.dev = DEVICE_DT_GET(DT_DRV_INST(inst))};                                          \
	COND_CODE_1(DT_PROP_EXISTS(inst, stallguard_threshold_velocity),			   \
	BUILD_ASSERT(DT_PROP(inst, stallguard_threshold_velocity),				   \
		     "stallguard threshold velocity must be a positive value"), ());               \
	static const struct tmc51xx_config tmc51xx_config_##inst = {COND_CODE_1			   \
		(DT_INST_ON_BUS(inst, spi),							   \
		(TMC51XX_CONFIG_SPI(inst)),							   \
		(TMC51XX_CONFIG_UART(inst))),							   \
		 .gconf = ((DT_INST_PROP(inst, en_pwm_mode) << TMC51XX_GCONF_EN_PWM_MODE_SHIFT) |  \
			   (DT_INST_PROP(inst, test_mode) << TMC51XX_GCONF_TEST_MODE_SHIFT) |      \
			   (DT_INST_PROP(inst, shaft) << TMC51XX_GCONF_SHAFT_SHIFT) |              \
			   (DT_INST_NODE_HAS_PROP(inst, diag0_gpios)                               \
				    ? BIT(TMC51XX_GCONF_DIAG0_INT_PUSHPULL_SHIFT)                  \
				    : 0)),                                                         \
		 .clock_frequency = DT_INST_PROP(inst, clock_frequency),                           \
		 .motion_controller = DEVICE_DT_GET_OR_NULL(                                       \
				DT_CHILD_BY_COMPATIBLE(DT_DRV_INST(inst),                          \
							adi_tmc51xx_motion_controller)),           \
		 .stepper_driver = DEVICE_DT_GET_OR_NULL(                                          \
				DT_CHILD_BY_COMPATIBLE(DT_DRV_INST(inst),                          \
							adi_tmc51xx_stepper_driver))               \
	};											   \
	DEVICE_DT_INST_DEFINE(inst, tmc51xx_init, NULL, &tmc51xx_data_##inst,                      \
			      &tmc51xx_config_##inst, POST_KERNEL, CONFIG_STEPPER_INIT_PRIORITY,   \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(TMC51XX_DEFINE)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_tmc51xx_motion_controller
DT_INST_FOREACH_STATUS_OKAY(TMC51XX_MOTION_CONTROLLER_DEFINE)
#undef DT_DRV_COMPAT

#define DT_DRV_COMPAT adi_tmc51xx_stepper_driver
DT_INST_FOREACH_STATUS_OKAY(TMC51XX_STEPPER_DRIVER_DEFINE)
#undef DT_DRV_COMPAT