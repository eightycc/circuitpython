// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// #include <string.h>
// #include <stdio.h>

// #include "shared/runtime/interrupt_char.h"
#include "py/runtime.h"
// #include "py/stream.h"

// #include "shared-bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/Connection.h"
#include "shared-bindings/_bleio/CharacteristicBuffer.h"
// #include "supervisor/shared/tick.h"
// #include "common-hal/_bleio/CharacteristicBuffer.h"

// void _common_hal_bleio_characteristic_buffer_construct(bleio_characteristic_buffer_obj_t *self,
//    bleio_characteristic_obj_t *characteristic,
//    mp_float_t timeout,
//    uint8_t *buffer, size_t buffer_size,
//    void *static_handler_entry,
//    bool watch_for_interrupt_char);

void common_hal_bleio_characteristic_buffer_construct(bleio_characteristic_buffer_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    mp_float_t timeout,
    size_t buffer_size) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

uint32_t common_hal_bleio_characteristic_buffer_read(bleio_characteristic_buffer_obj_t *self,
    uint8_t *data, size_t len, int *errcode) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

uint32_t common_hal_bleio_characteristic_buffer_rx_characters_available(bleio_characteristic_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

void common_hal_bleio_characteristic_buffer_clear_rx_buffer(bleio_characteristic_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bool common_hal_bleio_characteristic_buffer_deinited(bleio_characteristic_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

void common_hal_bleio_characteristic_buffer_deinit(bleio_characteristic_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bool common_hal_bleio_characteristic_buffer_connected(bleio_characteristic_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}
