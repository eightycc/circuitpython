// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

#include "py/runtime.h"
// #include "common-hal/_bleio/UUID.h"
#include "shared-bindings/_bleio/UUID.h"

void common_hal_bleio_uuid_construct(bleio_uuid_obj_t *self,
    mp_int_t uuid16, const uint8_t uuid128[16]) {
    if (uuid128 == NULL) {
        self->ble_uuid.type = BLE_UUID_TYPE_16;
        self->ble_uuid.uuid.uuid16 = uuid16;
    } else {
        self->ble_uuid.type = BLE_UUID_TYPE_128;
        memcpy(self->ble_uuid.uuid.uuid128, uuid128, sizeof(self->ble_uuid.uuid.uuid128));
        self->ble_uuid.uuid.uuid128[12] = uuid16 & 0xff;
        self->ble_uuid.uuid.uuid128[13] = (uuid16 >> 8) & 0xff;
    }
}

uint32_t common_hal_bleio_uuid_get_uuid16(bleio_uuid_obj_t *self) {
    return self->ble_uuid.uuid.uuid16;
}

void common_hal_bleio_uuid_get_uuid128(bleio_uuid_obj_t *self, uint8_t uuid128[16]) {
    memcpy(uuid128, self->ble_uuid.uuid.uuid128, sizeof(self->ble_uuid.uuid.uuid128));
}

uint32_t common_hal_bleio_uuid_get_size(bleio_uuid_obj_t *self) {
    // .type is UUID size in bits
    return self->ble_uuid.type;
}

void common_hal_bleio_uuid_pack_into(bleio_uuid_obj_t *self, uint8_t *buf) {
    if (self->ble_uuid.type == BLE_UUID_TYPE_16) {
        buf[0] = self->ble_uuid.uuid.uuid16 & 0xff;
        buf[1] = self->ble_uuid.uuid.uuid16 >> 8;
    } else {
        memcpy(buf, self->ble_uuid.uuid.uuid128, sizeof(self->ble_uuid.uuid.uuid128));
    }
}
