/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Bob Abeles
 * Copyright (c) 2018 Dan Halbert for Adafruit Industries
 * Copyright (c) 2016 Glenn Ruben Bakke
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

// #include <math.h>
// #include <stdint.h>
// #include <stdio.h>
// #include <string.h>

// #include "py/gc.h"
// #include "py/objstr.h"
#include "py/runtime.h"
// #include "supervisor/shared/bluetooth/bluetooth.h"
// #include "supervisor/shared/safe_mode.h"
// #include "supervisor/shared/tick.h"
// #include "supervisor/usb.h"
// #include "shared-bindings/_bleio/__init__.h"
#include "shared-bindings/_bleio/Adapter.h"
// #include "shared-bindings/_bleio/Address.h"
// #include "shared-bindings/nvm/ByteArray.h"
// #include "shared-bindings/_bleio/Connection.h"
// #include "shared-bindings/_bleio/ScanEntry.h"
// #include "shared-bindings/time/__init__.h"

// #include "controller/ble_ll_adv.h"
// #include "nimble/hci_common.h"
// #include "nimble/nimble_port.h"
// #include "nimble/nimble_port_freertos.h"
// #include "host/ble_gap.h"
// #include "host/util/util.h"
// #include "services/gap/ble_svc_gap.h"

// #include "common-hal/_bleio/Connection.h"

// #include "esp_bt.h"
// #include "esp_nimble_hci.h"


bool common_hal_bleio_adapter_get_enabled(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

void common_hal_bleio_adapter_set_enabled(bleio_adapter_obj_t *self, bool enabled) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bleio_address_obj_t *common_hal_bleio_adapter_get_address(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

bool common_hal_bleio_adapter_set_address(bleio_adapter_obj_t *self, bleio_address_obj_t *address) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

mp_obj_str_t *common_hal_bleio_adapter_get_name(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

void common_hal_bleio_adapter_set_name(bleio_adapter_obj_t *self, const char *name) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

bool common_hal_bleio_adapter_get_advertising(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

bool common_hal_bleio_adapter_get_connected(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return false;
}

mp_obj_t common_hal_bleio_adapter_get_connections(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

void common_hal_bleio_adapter_erase_bonding(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

mp_obj_t common_hal_bleio_adapter_connect(bleio_adapter_obj_t *self, bleio_address_obj_t *address, mp_float_t timeout) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

mp_obj_t common_hal_bleio_adapter_start_scan(bleio_adapter_obj_t *self, uint8_t *prefixes, size_t prefix_length, bool extended, mp_int_t buffer_size, mp_float_t timeout, mp_float_t interval, mp_float_t window, mp_int_t minimum_rssi, bool active) {
    // TODO
    mp_raise_NotImplementedError(NULL);
    return NULL;
}

void common_hal_bleio_adapter_stop_scan(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_adapter_start_advertising(bleio_adapter_obj_t *self,
    bool connectable, bool anonymous, uint32_t timeout, mp_float_t interval,
    mp_buffer_info_t *advertising_data_bufinfo,
    mp_buffer_info_t *scan_response_data_bufinfo,
    mp_int_t tx_power, const bleio_address_obj_t *directed_to) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}

void common_hal_bleio_adapter_stop_advertising(bleio_adapter_obj_t *self) {
    // TODO
    mp_raise_NotImplementedError(NULL);
}
