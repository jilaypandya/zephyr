#ifndef ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC50XX_H_
#define ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC50XX_H_

#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/stepper.h>

int tmc50xx_write(const struct device *dev, const uint8_t reg_addr, const uint32_t reg_val);

int tmc50xx_read(const struct device *dev, const uint8_t reg_addr, uint32_t *reg_val);

void motion_controller_trigger_cb(const struct device *dev, const enum stepper_event event);

void driver_trigger_cb(const struct device *dev, const enum stepper_event event);

int tmc50xx_stallguard_enable(const struct device *dev, const bool enable);

int read_actual_position(const struct device *dev, const uint8_t index, int32_t *position);

int tmc50xx_get_clock_frequency(const struct device *dev);

void rampstat_work_reschedule(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_STEPPER_ADI_TMC_TMC50XX_H_ */