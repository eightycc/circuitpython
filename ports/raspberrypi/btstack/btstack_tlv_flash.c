// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
//
// SPDX-License-Identifier: MIT

#include "btstack_tlv_flash.h"
#include <assert.h>

extern char __flash_binary_end;

uint32_t btstack_flash_bank_get_storage_offset(void) {
    // Check we're not overlapping the binary in flash
    assert(((uintptr_t)&__flash_binary_end - XIP_BASE <= CIRCUITPY_BTSTACK_TLV_BANK_OFFSET));

    return CIRCUITPY_BTSTACK_TLV_BANK_OFFSET;
}
