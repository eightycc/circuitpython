/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
 * Copyright (c) 2020 Dan Halbert for Adafruit Industries
 * Copyright (c) 2018 Artur Pacholec
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
// #include "shared-bindings/_bleio/Characteristic.h"
// #include "shared-bindings/_bleio/Descriptor.h"
#include "shared-bindings/_bleio/Service.h"
// #include "shared-bindings/_bleio/Adapter.h"

// Private version that doesn't allocate on the heap
// uint32_t _common_hal_bleio_service_construct(bleio_service_obj_t *self, bleio_uuid_obj_t *uuid, bool is_secondary, mp_obj_list_t *characteristic_list);

void common_hal_bleio_service_construct(bleio_service_obj_t *self, bleio_uuid_obj_t *uuid, bool is_secondary) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

// void common_hal_bleio_service_from_remote_service(bleio_service_obj_t *self,
//    bleio_connection_obj_t *connection, bleio_uuid_obj_t *uuid, bool is_secondary);

bleio_uuid_obj_t *common_hal_bleio_service_get_uuid(bleio_service_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

mp_obj_tuple_t *common_hal_bleio_service_get_characteristics(bleio_service_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

bool common_hal_bleio_service_get_is_remote(bleio_service_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

bool common_hal_bleio_service_get_is_secondary(bleio_service_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

// void common_hal_bleio_service_add_characteristic(bleio_service_obj_t *self,
//    bleio_characteristic_obj_t *characteristic, mp_buffer_info_t *initial_value_bufinfo,
//    const char *user_description);

void common_hal_bleio_service_deinit(bleio_service_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
