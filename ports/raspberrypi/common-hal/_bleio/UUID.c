/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Dan Halbert for Adafruit Industries
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

// #include <string.h>

#include "py/runtime.h"
#include "shared-bindings/_bleio/UUID.h"
// #include "shared-bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/Adapter.h"

void common_hal_bleio_uuid_construct(bleio_uuid_obj_t *self,
    mp_int_t uuid16, const uint8_t uuid128[16]) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

uint32_t common_hal_bleio_uuid_get_uuid16(bleio_uuid_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

void common_hal_bleio_uuid_get_uuid128(bleio_uuid_obj_t *self, uint8_t uuid128[16]) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

uint32_t common_hal_bleio_uuid_get_size(bleio_uuid_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

void common_hal_bleio_uuid_pack_into(bleio_uuid_obj_t *self, uint8_t *buf) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
