// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2020 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
//
// SPDX-License-Identifier: MIT

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
