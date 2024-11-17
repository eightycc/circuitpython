// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2018 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
//
// SPDX-License-Identifier: MIT

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
