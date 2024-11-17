// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019-2020 Jim Mussared
// SPDX-FileCopyrightText: Copyright (c) 2018 Ayke van Laethem
// SPDX-FileCopyrightText: Copyright (c) 2018 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

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
