# SPDX-FileCopyrightText: Copyright (c) 2025 Jilay Sandeep Pandya
# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/bldc.h)

zephyr_library()
zephyr_library_property(ALLOW_EMPTY TRUE)

zephyr_library_sources_ifdef(CONFIG_ST_L6235 st_l6235.c)