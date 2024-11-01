// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2021 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

#pragma once

// Put board-specific pico-sdk definitions here. This file must exist.

// Although CircruitPython determines flash size dynamically, we need to set a default here
// to satisfy asserts inside Pico SDK BTstack integration. This value does nothing but satisfy
// the asserts, BTstack flash allocation is done by btstack_flash_bank_get_storage_offset().
#define PICO_FLASH_SIZE_BYTES (2 * 1024 * 1024)

// BTstack uses two flash banks, each of which is a single sector.
#define PICO_FLASH_BANK_TOTAL_SIZE  (FLASH_SECTOR_SIZE * 2u)

// CircuityPython allocates flash storage for BTstack TLV at build time using this function.
#define pico_flash_bank_get_storage_offset_func btstack_flash_bank_get_storage_offset
