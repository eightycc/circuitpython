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
