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

// #include <string.h>

#include "py/runtime.h"
#include "py/mpprint.h"

#include "btstack.h"
// #include "hci_dump_mp_printf.h"

#include "shared-bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/Adapter.h"
// #include "shared-bindings/_bleio/Characteristic.h"
// #include "shared-bindings/_bleio/Connection.h"
// #include "shared-bindings/_bleio/Descriptor.h"
// #include "shared-bindings/_bleio/Service.h"
// #include "shared-bindings/_bleio/UUID.h"
// #include "supervisor/shared/bluetooth/bluetooth.h"
// #include "common-hal/_bleio/__init__.h"

// The Raspberry Pi SDK initializes the Bluetooth stack when port_init() invokes
// cyw43_arch_init_country() early in the boot process. The SDK initializes both
// the Bluetooth and WiFi stacks. SDK Bluetooth initialization is performed by
// btstack_cyw43_init() as follows:
//
//   - btstack_memory_init() is invoked to initialize BTStack memory.
//   - btstack_run_loop_init() is invoked to initialize the BTStack run loop.
//   - (optional, WANT_HCI_DUMP) hci_dump_init() is invoked to initialize HCI
//     tracing.
//   - hci_init() is invoked to initialize the HCI layer. It is passed
//     an hci_transport_t object that enumerates the callbacks into the CYW43
//     driver that BTStack will use to communicate with the CYW43.
//   - setup_tlv() is invoked to initialize the TLV storage layer. This is the
//     flash storage database used to store persistent bonding information.
//
// The SDK also provides the btstack_cyw43_deinit() function that is invoked by
// cyw43_arch_deinit() to de-initialize the Bluetooth and WiFi stacks. It is not
// currently used by CircuitPython, but is invoked by cyw43_arch_init() if it
// encounters an error during initialization.
//
// The SDK initialization steps are performed once when the port initializes.
// Except for the TLV storage layer initialization, all other initialization
// steps are performed whenever the Adapter is enabled.
//
// A note about power control: The CYW43439 has independent power control pins
// for the Bluetooth and WiFi radios, but the Raspberry Pi Pico W connects both
// to a single RP2040 GPIO pin. This renders independent power control of the
// radios impossible.


// The singleton _bleio.Adapter object, bound to _bleio.adapter
bleio_adapter_obj_t common_hal_bleio_adapter_obj;
// Ensure that adapter singleton is initialized only once before use.
static bool bleio_adapter_first_reset = true;

// Two resets are provided that perform different levels of reset of the Bluetooth
// stack. The user reset stops scanning and advertising and then restarts Bluetooth
// background processing. User reset in invoked by `cleanup_after_vm()`, which is
// called when the CircuitPython VM is reset.
void bleio_user_reset() {
    mp_printf(&mp_plat_print, "bleio_user_reset\n");

    // Suspend generic hci event handling.
    // TODO

    // Stop any user scanning or advertising.
    // common_hal_bleio_adapter_stop_scan(&common_hal_bleio_adapter_obj);
    // if (common_hal_bleio_adapter_obj.current_advertising_data != NULL)
    //     common_hal_bleio_adapter_stop_advertising(&common_hal_bleio_adapter_obj);

    // Resume generic hci event handling.
    // TODO

    // Re-start advertising of BLE workflow.
    // supervisor_bluetooth_background();
}

// The full reset performs a complete reset of the Bluetooth stack. It is invoked
// in a number of places, most notably from `supervisor_workflow_start()`, when
// restarting the board before exiting run_code_py(), when exiting `run_repl()`,
// during `cleanup_after_vm(), and early in `main()`.
//
// The first call to bleio_reset() comes from `supervisor_workflow_start()`, which
void bleio_reset() {
    mp_printf(&mp_plat_print, "bleio_reset\n");
    // The first adapter reset initializes the BLE adapter singleton object.
    if (bleio_adapter_first_reset) {
        common_hal_bleio_adapter_obj.base.type = &bleio_adapter_type;
        // common_hal_bleio_adapter_obj.scanning = false;
        // common_hal_bleio_adapter_obj.advertising = false;
        // common_hal_bleio_adapter_obj.user_advertising = false;
        // Both CYW43 adapter and BTstack have odd starting states due to SDK.
        common_hal_bleio_adapter_obj.adapter_state = ADAPTER_STATE_DISABLED;
        common_hal_bleio_adapter_obj.btstack_state = BTSTACK_STATE_FIRST_START;

        bleio_adapter_first_reset = false;
    }

    // supervisor_stop_bluetooth();
    // bleio_adapter_reset(&common_hal_bleio_adapter_obj);
    common_hal_bleio_adapter_set_enabled(&common_hal_bleio_adapter_obj, false);
    // bonding_reset();
    // supervisor_start_bluetooth();
}

void common_hal_bleio_gc_collect(void) {
    mp_printf(&mp_plat_print, "common_hal_bleio_gc_collect\n");
    bleio_adapter_gc_collect(&common_hal_bleio_adapter_obj);
}

void common_hal_bleio_init(void) {
    // TODO
    mp_printf(&mp_plat_print, "common_hal_bleio_init\n");
}
