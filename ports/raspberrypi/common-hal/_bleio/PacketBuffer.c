// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019-2020 Scott Shawcroft for Adafruit Industries
//
// SPDX-License-Identifier: MIT

// #include <string.h>
// #include <stdio.h>

// #include "shared/runtime/interrupt_char.h"
#include "py/runtime.h"
// #include "py/stream.h"

// #include "shared-bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/Connection.h"
#include "shared-bindings/_bleio/PacketBuffer.h"
// #include "supervisor/shared/tick.h"

// packet event handler
// static void packet_buffer_client_evt(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
//
// }

void common_hal_bleio_packet_buffer_construct(
    bleio_packet_buffer_obj_t *self, bleio_characteristic_obj_t *characteristic,
    size_t buffer_size, size_t max_packet_size) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

// Allocation free
// void _common_hal_bleio_packet_buffer_construct(
//    bleio_packet_buffer_obj_t *self, bleio_characteristic_obj_t *characteristic,
//    uint32_t *incoming_buffer, size_t incoming_buffer_size,
//    uint32_t *outgoing_buffer1, uint32_t *outgoing_buffer2, size_t outgoing_buffer_size,
//    void *static_handler_entry);

mp_int_t common_hal_bleio_packet_buffer_write(bleio_packet_buffer_obj_t *self,
    const uint8_t *data, size_t len, uint8_t *header, size_t header_len) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

mp_int_t common_hal_bleio_packet_buffer_readinto(bleio_packet_buffer_obj_t *self,
    uint8_t *data, size_t len) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

mp_int_t common_hal_bleio_packet_buffer_get_incoming_packet_length(bleio_packet_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

mp_int_t common_hal_bleio_packet_buffer_get_outgoing_packet_length(bleio_packet_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

// void common_hal_bleio_packet_buffer_flush(bleio_packet_buffer_obj_t *self);

bool common_hal_bleio_packet_buffer_deinited(bleio_packet_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

void common_hal_bleio_packet_buffer_deinit(bleio_packet_buffer_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
