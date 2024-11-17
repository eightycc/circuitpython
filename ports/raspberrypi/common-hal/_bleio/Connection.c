// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2018 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
//
// SPDX-License-Identifier: MIT

#include "shared-bindings/_bleio/Connection.h"

// #include "att.h"

// #include <string.h>
// #include <stdio.h>

// #include "shared/runtime/interrupt_char.h"
// #include "py/gc.h"
// #include "py/objlist.h"
// #include "py/objstr.h"
// #include "py/qstr.h"
#include "py/runtime.h"
// #include "shared-bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/Adapter.h"
// #include "shared-bindings/_bleio/Attribute.h"
// #include "shared-bindings/_bleio/Characteristic.h"
// #include "shared-bindings/_bleio/Service.h"
// #include "shared-bindings/_bleio/UUID.h"
// #include "supervisor/shared/tick.h"

void common_hal_bleio_connection_pair(bleio_connection_internal_t *self, bool bond) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_connection_disconnect(bleio_connection_internal_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bool common_hal_bleio_connection_get_connected(bleio_connection_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

mp_int_t common_hal_bleio_connection_get_max_packet_length(bleio_connection_internal_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

bool common_hal_bleio_connection_get_paired(bleio_connection_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

mp_obj_tuple_t *common_hal_bleio_connection_discover_remote_services(bleio_connection_obj_t *self,
    mp_obj_t service_uuids_whitelist) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0;
}

mp_float_t common_hal_bleio_connection_get_connection_interval(bleio_connection_internal_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return 0.0;
}

void common_hal_bleio_connection_set_connection_interval(bleio_connection_internal_t *self,
    mp_float_t new_interval) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
