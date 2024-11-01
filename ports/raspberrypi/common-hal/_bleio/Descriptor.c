/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Dan Halbert for Adafruit Industries
 * Copyright (c) 2018 Artur Pacholec
 * Copyright (c) 2016 Glenn Ruben Bakke
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
#include "shared-bindings/_bleio/Descriptor.h"
// #include "shared-bindings/_bleio/Service.h"
#include "shared-bindings/_bleio/UUID.h"

void common_hal_bleio_descriptor_construct(bleio_descriptor_obj_t *self,
    bleio_characteristic_obj_t *characteristic, bleio_uuid_obj_t *uuid,
    bleio_attribute_security_mode_t read_perm, bleio_attribute_security_mode_t write_perm,
    mp_int_t max_length, bool fixed_length, mp_buffer_info_t *initial_value_bufinfo) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bleio_uuid_obj_t *common_hal_bleio_descriptor_get_uuid(bleio_descriptor_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

bleio_characteristic_obj_t *common_hal_bleio_descriptor_get_characteristic(bleio_descriptor_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

size_t common_hal_bleio_descriptor_get_value(bleio_descriptor_obj_t *self, uint8_t *buf, size_t len) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

void common_hal_bleio_descriptor_set_value(bleio_descriptor_obj_t *self, mp_buffer_info_t *bufinfo) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
