/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "stdint.h"
#include "stdbool.h"
#include "stddef.h"

#include "py/obj.h"

typedef struct {
    uint16_t value_handle;
    uint8_t *buf;
    size_t len;
    volatile uint16_t att_status;
    volatile bool done;
} bleio_gattc_read_t;

// size_t common_hal_bleio_gattc_read(uint16_t handle, uint16_t conn_handle, uint8_t *buf, size_t len);
// void common_hal_bleio_gattc_write(uint16_t handle, uint16_t conn_handle, mp_buffer_info_t *bufinfo, bool write_no_response);

// size_t common_hal_bleio_gatts_read(uint16_t handle, uint16_t conn_handle, uint8_t *buf, size_t len);
// void common_hal_bleio_gatts_write(uint16_t handle, uint16_t conn_handle, mp_buffer_info_t *bufinfo);

typedef struct {
    uint8_t *buf;
    size_t alloc_sz;
    size_t len;
    bool append;
} bleio_gatts_db_entry_t;

typedef mp_map_t *bleio_gatts_db_t;

void bleio_gatts_db_create(bleio_gatts_db_t *db);
bleio_gatts_db_entry_t *bleio_gatts_db_lookup(bleio_gatts_db_t db, uint16_t handle);
void bleio_gatts_db_create_entry(bleio_gatts_db_t db, uint16_t handle, size_t len);
void bleio_gatts_db_read(bleio_gatts_db_t db, uint16_t handle, const uint8_t **value, size_t *value_len);
void bleio_gatts_db_write(bleio_gatts_db_t db, uint16_t handle, const uint8_t *value, size_t value_len);
