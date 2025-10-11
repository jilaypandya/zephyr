# SPDX-FileCopyrightText: Copyright (c) 2024 Fabian Blatz
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(gpio_stepper_common.c)
zephyr_library_sources(stepper_work_timing.c)
zephyr_library_sources_ifdef(CONFIG_GPIO_STEPPER_COUNTER_TIMING stepper_counter_timing.c)
