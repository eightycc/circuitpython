// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
//
// SPDX-License-Identifier: MIT
#pragma once

#include "py/obj.h"
#include "hardware/flash.h"
#include "pico/config.h"

// The BTstack TLV flash banks are located at the top of the firmware image area and are
// aligned on a FLASH_SECTOR_SIZE boundary.
#define CIRCUITPY_BTSTACK_TLV_BANK_OFFSET ((CIRCUITPY_FIRMWARE_SIZE - PICO_FLASH_BANK_TOTAL_SIZE) & ~(FLASH_SECTOR_SIZE - 1))

uint32_t btstack_flash_bank_get_storage_offset(void);
