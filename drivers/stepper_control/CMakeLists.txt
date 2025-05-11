# SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/stepper_control.h)

# zephyr-keep-sorted-start
add_subdirectory_ifdef(CONFIG_STEPPER_CONTROL_ADI_TMC adi_tmc)
add_subdirectory_ifdef(CONFIG_ZEPHYR_STEPPER_MOTION_CONTROL zephyr_stepper_motion_control)
# zephyr-keep-sorted-stop
