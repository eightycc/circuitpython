// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

#pragma once

#include "stdint.h"

#include "py/obj.h"

// Enum specifies type of UUID as its bit length.
typedef enum {
    BLE_UUID_TYPE_16 = 16,
    BLE_UUID_TYPE_32 = 32,
    BLE_UUID_TYPE_128 = 128
} ble_uuid_type_t;

typedef uint16_t ble_uuid16_t;
typedef uint32_t ble_uuid32_t;
typedef uint8_t ble_uuid128_t[16];

typedef struct {
    ble_uuid_type_t type;
    union {
        ble_uuid16_t uuid16;
        ble_uuid32_t uuid32;
        ble_uuid128_t uuid128;
    } uuid;
} ble_uuid_t;

typedef struct {
    mp_obj_base_t base;
    // UUIDs are stored as a union of the three possible UUID sizes.
    ble_uuid_t ble_uuid;
} bleio_uuid_obj_t;
