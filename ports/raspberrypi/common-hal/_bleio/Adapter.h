/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
 * Copyright (c) 2018 Dan Halbert for Adafruit Industries
 * Copyright (c) 2018 Artur Pacholec
 * Copyright (c) 2016 Glenn Ruben Bakke
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

#include "py/obj.h"
#include "py/objstr.h"

#include "supervisor/background_callback.h"
#include "shared-bindings/_bleio/Address.h"
#include "shared-bindings/_bleio/Connection.h"
#include "shared-bindings/_bleio/ScanResults.h"

#include "btstack_config.h"

// Keep CP internal connection object count in synch with BTstack MAX_NR_HCI_CONNECTIONS.
#define BLEIO_TOTAL_CONNECTION_COUNT MAX_NR_HCI_CONNECTIONS
extern bleio_connection_internal_t bleio_connections[BLEIO_TOTAL_CONNECTION_COUNT];

typedef enum {
    BTSTACK_STATE_OFF,
    BTSTACK_STATE_INITIALIZING,
    BTSTACK_STATE_WORKING,
    BTSTACK_STATE_HALTING,
    BTSTACK_STATE_SLEEPING,
    BTSTACK_STATE_FALLING_ASLEEP,
    BTSTACK_STATE_FIRST_START,
    BTSTACK_STATE_ERROR,
} btstack_state_t;

typedef enum {
    ADAPTER_STATE_DISABLED,
    ADAPTER_STATE_ENABLED,
} adapter_state_t;

typedef struct {
    mp_obj_base_t base;
    volatile btstack_state_t btstack_state;
    volatile adapter_state_t adapter_state;
    bleio_gatts_db_t gatts_db;

    // bool scanning;
    // bool advertising;
    bleio_scanresults_obj_t *scan_results;

    uint8_t *current_advertising_data;
    uint8_t *advertising_data;
    // mp_obj_t name;
    mp_obj_tuple_t *connection_objs;
    background_callback_t background_callback;
    // bool user_advertising;
} bleio_adapter_obj_t;

void bleio_adapter_gc_collect(bleio_adapter_obj_t *adapter);
void bleio_adapter_reset(bleio_adapter_obj_t *adapter);
