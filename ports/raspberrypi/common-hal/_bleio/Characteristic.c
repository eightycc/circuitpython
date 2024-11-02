/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) Dan Halbert for Adafruit Industries
 * Copyright (c) 2018 Artur Pacholec
 * Copyright (c) 2023 Bob Abeles
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

// #include "shared-bindings/_bleio/__init__.h"
#include "shared-bindings/_bleio/Characteristic.h"
// #include "shared-bindings/_bleio/CharacteristicBuffer.h"
// #include "shared-bindings/_bleio/Descriptor.h"
// #include "shared-bindings/_bleio/PacketBuffer.h"
// #include "shared-bindings/_bleio/Service.h"

// #include "common-hal/_bleio/Adapter.h"
// #include "common-hal/_bleio/att.h"

bleio_characteristic_properties_t common_hal_bleio_characteristic_get_properties(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

mp_obj_tuple_t *common_hal_bleio_characteristic_get_descriptors(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

bleio_service_obj_t *common_hal_bleio_characteristic_get_service(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

bleio_uuid_obj_t *common_hal_bleio_characteristic_get_uuid(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

size_t common_hal_bleio_characteristic_get_max_length(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

size_t common_hal_bleio_characteristic_get_value(bleio_characteristic_obj_t *self, uint8_t *buf, size_t len) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

void common_hal_bleio_characteristic_add_descriptor(bleio_characteristic_obj_t *self,
    bleio_descriptor_obj_t *descriptor) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_characteristic_construct(bleio_characteristic_obj_t *self,
    bleio_service_obj_t *service, uint16_t handle, bleio_uuid_obj_t *uuid,
    bleio_characteristic_properties_t props, bleio_attribute_security_mode_t read_perm,
    bleio_attribute_security_mode_t write_perm, mp_int_t max_length, bool fixed_length,
    mp_buffer_info_t *initial_value_bufinfo, const char *user_description) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_characteristic_set_cccd(bleio_characteristic_obj_t *self, bool notify, bool indicate) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_characteristic_set_value(bleio_characteristic_obj_t *self, mp_buffer_info_t *bufinfo) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bool common_hal_bleio_characteristic_deinited(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

void common_hal_bleio_characteristic_deinit(bleio_characteristic_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
