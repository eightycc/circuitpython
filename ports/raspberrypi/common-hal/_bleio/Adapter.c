/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
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
#include "supervisor/shared/tick.h"
// #include "supervisor/usb.h"
#include "hci_dump_mp_printf.h"
#include "btstack_evts.h"
#include "btstack.h"
#include "btstack_tlv.h"
#include "pico/btstack_run_loop_async_context.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_hci_transport_cyw43.h"

#include "shared-module/os/__init__.h"
#include "shared-bindings/_bleio/__init__.h"
#include "shared-bindings/_bleio/Adapter.h"
// #include "shared-bindings/_bleio/Address.h"
// #include "shared-bindings/nvm/ByteArray.h"
#include "shared-bindings/_bleio/Connection.h"
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

static const uint16_t BTSTACK_GAP_DEVICE_NAME_HANDLE = 3;
#define BLE_GAP_SCAN_BUFFER_MAX (31)
#define BLE_GAP_SCAN_BUFFER_EXTENDED_MAX_SUPPORTED (255)

static bool adapter_event_handler(uint8_t *event, uint16_t event_size, void *context) {
    bleio_adapter_obj_t *self = (bleio_adapter_obj_t *)context;
    uint8_t event_type = hci_event_packet_get_type(event);
    switch (event_type) {
        // BTstack state change events.
        case BTSTACK_EVENT_STATE:
            uint8_t state = btstack_event_state_get_state(event);
            switch (state) {
                case HCI_STATE_OFF:
                    self->btstack_state = BTSTACK_STATE_OFF;
                    break;
                case HCI_STATE_INITIALIZING:
                    self->btstack_state = BTSTACK_STATE_INITIALIZING;
                    break;
                case HCI_STATE_WORKING:
                    self->btstack_state = BTSTACK_STATE_WORKING;
                    break;
                case HCI_STATE_HALTING:
                    self->btstack_state = BTSTACK_STATE_HALTING;
                    break;
                case HCI_STATE_SLEEPING:
                    self->btstack_state = BTSTACK_STATE_SLEEPING;
                    break;
                case HCI_STATE_FALLING_ASLEEP:
                    self->btstack_state = BTSTACK_STATE_FALLING_ASLEEP;
                    break;
                default:
                    return false;
            }
            return true;
        case BTSTACK_EVENT_POWERON_FAILED:
            self->btstack_state = BTSTACK_STATE_ERROR;
            // TODO: throw an exception?
            return true;

        // Advertising report events for possible forwarding to ScanResults.
        // BTstack may issue multiple advertising reports, one for each report in the HCI event.
        case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
            return true;
        case GAP_EVENT_ADVERTISING_REPORT:
            if (self->scan_results != NULL) {
                bd_addr_t report_address;
                gap_event_advertising_report_get_address(event, report_address);
                shared_module_bleio_scanresults_append(
                    self->scan_results,
                    supervisor_ticks_ms64(),
                    (gap_event_advertising_report_get_advertising_event_type(event) == 0x00)
                    || (gap_event_advertising_report_get_advertising_event_type(event) == 0x01),
                    gap_event_advertising_report_get_advertising_event_type(event) == 0x04,
                    gap_event_advertising_report_get_rssi(event),
                    report_address,
                    gap_event_advertising_report_get_address_type(event),
                    gap_event_advertising_report_get_data(event),
                    gap_event_advertising_report_get_data_length(event));
            }
            return true;
        default:
            return false;
    }
}

static void bleio_btstack_deinit(bleio_adapter_obj_t *self) {
    switch (self->btstack_state) {
        case BTSTACK_STATE_FIRST_START:
            // On first start the SDK leaves BTstack partially initialized. Advance to OFF state
            // and reset any partially initialized BTstack internal state.
            self->btstack_state = BTSTACK_STATE_OFF;
            hci_close();
            btstack_run_loop_deinit();
            btstack_memory_deinit();
            return;
        case BTSTACK_STATE_OFF:
            // BTstack is already off. Do nothing.
            return;
        case BTSTACK_STATE_INITIALIZING:
        case BTSTACK_STATE_FALLING_ASLEEP:
        case BTSTACK_STATE_HALTING:
            // Since we synchronize with BTstack state changes, these states should not occur.
            mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("BTstack deinit while changing state"));
            break;
        case BTSTACK_STATE_SLEEPING:
        case BTSTACK_STATE_WORKING:
        case BTSTACK_STATE_ERROR:
            // Power off HCI and wait until it is off or times out.
            // Although hci_close() also does this step, we shut down the HCI and synchronize with
            // BTstack here to (1) detect a hung HCI and (2) avoid a crash in bleio_evts_reset()
            // when it tries to remove the BTstack event handler.
            hci_power_control(HCI_POWER_OFF);
            uint64_t deadline = supervisor_ticks_ms64() + 1000;
            while (self->btstack_state != BTSTACK_STATE_OFF || deadline < supervisor_ticks_ms64()) {
                RUN_BACKGROUND_TASKS;
            }
            if (self->btstack_state != BTSTACK_STATE_OFF) {
                mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("BTstack failed to power off HCI"));
            }
            // Reset BTstack event passthru.
            bleio_evts_reset();
            break;
    }

    // De-init BTstack.
    sm_deinit();
    l2cap_deinit();
    hci_close();
    btstack_run_loop_deinit();
    btstack_memory_deinit();

    // Release GATT server database.
    self->gatts_db = NULL;
}

static void bleio_btstack_init(bleio_adapter_obj_t *self) {
    // De-init BTstack if it's already initialized.
    bleio_btstack_deinit(self);

    // Erase both TLV flash banks. This is a debug tool that can wear
    // out flash, so use with caution.
    // const btstack_tlv_t * tlv_impl = NULL;
    // btstack_tlv_flash_bank_t * tlv_context;
    // btstack_tlv_get_instance(&tlv_impl, &tlv_context);
    // if (tlv_context) {
    //    tlv_context->hal_flash_bank_impl->erase(tlv_context->hal_flash_bank_context, 0);
    //    tlv_context->hal_flash_bank_impl->erase(tlv_context->hal_flash_bank_context, 1);
    // }

    // Initialize BTstack using SDK. Includes a harmless TLV flash restart.
    // btstack_cyw43_init(cyw43_arch_async_context());

    // Initialize BTstack in the same fashion as the SDK except do not restart the TLV flash.
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_async_context_get_instance(cyw43_arch_async_context()));
    hci_init(hci_transport_cyw43_instance(), NULL);

    // Start BTstack HCI packet logging.
    hci_dump_init(hci_dump_mp_printf_get_instance());
    hci_dump_enable_packet_log(true);

    // Create our GATT server database.
    bleio_gatts_db_create(&self->gatts_db);
    // Add an entry for the adapter's device name.
    bleio_gatts_db_create_entry(self->gatts_db, BTSTACK_GAP_DEVICE_NAME_HANDLE, 32);

    // Initialize l2cap.
    l2cap_init();

    // Initialize BTstack security manager.
    sm_init();

    // Initialize BTstack event passthru to _bleio.
    bleio_evts_init();

    // Install BTstack adapter-level event handler.
    bleio_evt_add_event_handler(adapter_event_handler, self);

    // Power on HCI, this will trigger the HCI startup sequence.
    hci_power_control(HCI_POWER_ON);
    // Wait for HCI startup to complete. Timeout should not occur, but if it does we'll throw
    // an exception.
    uint64_t deadline = supervisor_ticks_ms64() + 1000;
    while (self->btstack_state != BTSTACK_STATE_WORKING || deadline < supervisor_ticks_ms64()) {
        RUN_BACKGROUND_TASKS;
    }
    if (self->btstack_state != BTSTACK_STATE_WORKING) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("BTstack failed to power on HCI"));
    }
}

// The adapter's GAP name defaults to "CIRCUITPYxxxx\0", where "xxxx" is the last four hex digits
// of the adapter's MAC address. Additionally, the default may be overridden by a CIRCUITPY_BLE_NAME
// environment variable.
char default_ble_name[] = { 'C', 'I', 'R', 'C', 'U', 'I', 'T', 'P', 'Y', 0, 0, 0, 0, 0};

static void bleio_adapter_reset_name(bleio_adapter_obj_t *self) {
    // setup the default name
    bd_addr_t own_addr;
    uint8_t addr_type;
    gap_le_get_own_address(&addr_type, own_addr);
    mp_int_t len = sizeof(default_ble_name) - 1;
    default_ble_name[len - 4] = nibble_to_hex_lower[own_addr[1] >> 4 & 0xf];
    default_ble_name[len - 3] = nibble_to_hex_lower[own_addr[1] & 0xf];
    default_ble_name[len - 2] = nibble_to_hex_lower[own_addr[0] >> 4 & 0xf];
    default_ble_name[len - 1] = nibble_to_hex_lower[own_addr[0] & 0xf];
    default_ble_name[len] = '\0'; // for now we add null for compatibility with C ASCIIZ strings

    #if CIRCUITPY_OS_GETENV
    char ble_name[32];

    os_getenv_err_t result = common_hal_os_getenv_str("CIRCUITPY_BLE_NAME", ble_name, sizeof(ble_name));
    if (result == GETENV_OK) {
        common_hal_bleio_adapter_set_name(self, ble_name);
        return;
    }
    #endif

    common_hal_bleio_adapter_set_name(self, (char *)default_ble_name);
}

bleio_connection_internal_t bleio_connections[BLEIO_TOTAL_CONNECTION_COUNT];


bool common_hal_bleio_adapter_get_enabled(bleio_adapter_obj_t *self) {
    return self->adapter_state == ADAPTER_STATE_ENABLED;
}

void common_hal_bleio_adapter_set_enabled(bleio_adapter_obj_t *self, bool enabled) {
    // Determine whether adapter is enabled or disabled.
    bool is_enabled = common_hal_bleio_adapter_get_enabled(self);

    // Exit if adapter is already in the desired state.
    if (is_enabled == enabled) {
        return;
    }

    // Change adapter state.
    if (enabled) {
        // Ideally, the adapter will be in a clean disabled state when this is called.
        // Otherwise, we'll need to clean up the adapter's state here.

        // Initialize BTstack.
        bleio_btstack_init(self);

        // Reset all connections.

        // Init co-operative background processing.
        // TODO: should be part of one time init.
        // self->background_callback.fun = bluetooth_adapter_background;
        // self->background_callback.context = self;

        self->adapter_state = ADAPTER_STATE_ENABLED;

        // Reset adapter's name.
        bleio_adapter_reset_name(self);

        // Install BTstack event handling for connection events.

        // Run background processing.
        // bluetooth_adapter_background(self);

    } else {
        // Disable BTstack event handling.
        // Reset scanning, advertising, and connections.
        bleio_adapter_reset(self);
        // De-init BTstack.
        bleio_btstack_deinit(self);
        self->adapter_state = ADAPTER_STATE_DISABLED;
    }
}

static void check_adapter_enabled(bleio_adapter_obj_t *self) {
    if (self->adapter_state != ADAPTER_STATE_ENABLED) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Adapter is not enabled"));
    }
}

static void check_device_address(uint8_t type, const bd_addr_t address) {
    // Check device address validity according to the Bluetooth Core Specification v5.2,
    // Vol 6, Part B, Section 1.3:
    //   Public - No check needed
    //   Random - Check high-order bits for 11, check random part <> all zeroes or all ones
    //   Resolvable private - Check high-order bits for 01, check random part <> all zeroes or all ones
    //   Non-resolvable private - Check high-order bits for 00, check random part <> all zeroes or all ones
    bool valid = true;
    switch (type) {
        case BD_ADDR_TYPE_LE_PUBLIC:
            return;
        case BD_ADDR_TYPE_LE_RANDOM:
            if ((address[0] & 0xC0) != 0xC0) {
                valid = false;
            }
            break;
        case BD_ADDR_TYPE_LE_PUBLIC_IDENTITY:
            if ((address[0] & 0xC0) != 0x40) {
                valid = false;
            }
            break;
        case BD_ADDR_TYPE_LE_RANDOM_IDENTITY:
            if ((address[0] & 0xC0) != 0x80) {
                valid = false;
            }
            break;
        default:
            mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Invalid device address type"));
    }
    // The random portion of a random address must not be all zeroes or all ones.
    uint8_t addr_sum = address[0] & 0x3F;
    uint8_t addr_compl_sum = ~(address[0] | 0xC0);
    for (int i = 1; i < NUM_BLEIO_ADDRESS_BYTES; i++) {
        addr_sum |= address[i];
        addr_compl_sum |= ~address[i];
    }
    if (addr_sum == 0x00 || addr_compl_sum == 0x00) {
        valid = false;
    }
    if (!valid) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Invalid device address"));
    }
}

static void check_scanning(bleio_adapter_obj_t *self) {
    if (self->scan_results != NULL) {
        if (!shared_module_bleio_scanresults_get_done(self->scan_results)) {
            mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Scan in progress"));
        }
        self->scan_results = NULL;
    }
}

static void check_advertising(bleio_adapter_obj_t *self) {
    if (self->current_advertising_data != NULL) {
        if (self->current_advertising_data == self->advertising_data) {
            mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Advertising in progress"));
        }
        common_hal_bleio_adapter_stop_advertising(self);
    }
}

bleio_address_obj_t *common_hal_bleio_adapter_get_address(bleio_adapter_obj_t *self) {
    check_adapter_enabled(self);

    bd_addr_t local_address;
    uint8_t addr_type;
    gap_le_get_own_address(&addr_type, local_address);

    bleio_address_obj_t *address = mp_obj_malloc(bleio_address_obj_t, &bleio_address_type);
    common_hal_bleio_address_construct(address, local_address, addr_type);
    return address;
}

// Crypto random synchronization callback.
static void random_addr_ready(void *arg) {
    *(volatile bool *)arg = true;
}

bool common_hal_bleio_adapter_set_address(bleio_adapter_obj_t *self, bleio_address_obj_t *address) {
    // Adapter must be enabled and not scanning or advertising.
    check_adapter_enabled(self);
    check_scanning(self);
    check_advertising(self);

    bd_addr_t local_address;
    mp_buffer_info_t bufinfo;
    if (!mp_get_buffer(address->bytes, &bufinfo, MP_BUFFER_READ)) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Malformed address buffer"));
    }
    memcpy(local_address, bufinfo.buf, NUM_BLEIO_ADDRESS_BYTES);
    // Validate the address.
    check_device_address(address->type, local_address);
    // Set the address.
    switch (address->type) {
        case BD_ADDR_TYPE_LE_PUBLIC:
            // Set adapter to its pre-programmed public address.
            // The factory-programmed public address is used. It cannot be overridden.
            gap_random_address_set_mode(GAP_RANDOM_ADDRESS_TYPE_OFF);
            return true;
        case BD_ADDR_TYPE_LE_RANDOM:
            if (local_address[0] & 0x02) {
                // Set adapter to the caller-provided random address.
                gap_random_address_set_mode(GAP_RANDOM_ADDRESS_TYPE_STATIC);
                gap_random_address_set(local_address);
            } else {
                // Set adapter to a randomly generated address.
                btstack_crypto_random_t request;
                volatile bool crypto_done = false;
                btstack_crypto_random_generate(&request, local_address, NUM_BLEIO_ADDRESS_BYTES, &random_addr_ready, (void *)&crypto_done);
                // Synchronize with random address generation completion.
                while (!crypto_done) {
                    RUN_BACKGROUND_TASKS;
                }
                // Set the L bit and clear the G bit (locally administered, individual address)
                local_address[0] = (local_address[0] | 0x02) & 0xFE;
                gap_random_address_set_mode(GAP_RANDOM_ADDRESS_TYPE_STATIC);
                gap_random_address_set(local_address);
            }
            return true;
        default:
            // Unimplemented and invalid address types.
            return false;
    }
}

mp_obj_str_t *common_hal_bleio_adapter_get_name(bleio_adapter_obj_t *self) {
    // Adapter must be enabled.
    check_adapter_enabled(self);
    const uint8_t *name = NULL;
    size_t name_len = 0;

    bleio_gatts_db_read(self->gatts_db, BTSTACK_GAP_DEVICE_NAME_HANDLE, &name, &name_len);

    return mp_obj_new_str((const char *)name, name_len);
}

void common_hal_bleio_adapter_set_name(bleio_adapter_obj_t *self, const char *name) {
    // Adapter must be enabled and not scanning or advertising.
    check_adapter_enabled(self);
    check_scanning(self);
    check_advertising(self);

    size_t len = strlen(name);
    bleio_gatts_db_write(self->gatts_db, BTSTACK_GAP_DEVICE_NAME_HANDLE, (const uint8_t *)name, len);
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

static btstack_timer_source_t scan_timer;

static void scan_timer_handler(btstack_timer_source_t *ts) {
    UNUSED(ts);
    common_hal_bleio_adapter_stop_scan(&common_hal_bleio_adapter_obj);
}

mp_obj_t common_hal_bleio_adapter_start_scan(bleio_adapter_obj_t *self, uint8_t *prefixes, size_t prefix_length,
    bool extended, mp_int_t buffer_size, mp_float_t timeout, mp_float_t interval, mp_float_t window,
    mp_int_t minimum_rssi, bool active) {
    // Adapter must be enabled and not scanning or advertising.
    check_adapter_enabled(self);
    check_scanning(self);
    check_advertising(self);

    // Construct a new ScanResults object.
    self->scan_results = shared_module_bleio_new_scanresults(buffer_size, prefixes, prefix_length, minimum_rssi);
    if (self->scan_results == NULL) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Failed to allocate ScanResults object and buffer"));
    }

    // Set up timeout. Arg timeout is in floating-point seconds, convert to integer milliseconds.
    // For timeout <= 0, scan will run until stopped, usually by _stop_scan.
    if (timeout > 0) {
        btstack_run_loop_set_timer(&scan_timer, timeout * 1000.0 + 0.5);
        btstack_run_loop_set_timer_handler(&scan_timer, scan_timer_handler);
        btstack_run_loop_add_timer(&scan_timer);
    }

    // Set up scan parameters and start scanning.
    // Args interval and window are in floating-point seconds, convert to integer units of 0.625 ms.
    // BTstack requires a smaller interval and therefore a smaller window than the range check in
    // bleio_adapter_start_scan(). Rather than throw an exception, we'll limit the values here.
    uint16_t scan_interval = interval * 1600.0 + 0.5;
    scan_interval = (scan_interval <= 0x4000)? scan_interval : 0x4000;
    uint16_t scan_window = window * 1600.0 + 0.5;
    scan_window = (scan_window <= scan_interval)? scan_window : scan_interval;

    gap_set_scan_parameters(active? 1 : 0, scan_interval, scan_window);
    gap_start_scan();

    return MP_OBJ_FROM_PTR(self->scan_results);
}

void common_hal_bleio_adapter_stop_scan(bleio_adapter_obj_t *self) {
    // Note: This function may be called from an interrupt context due to
    // scan_timer_handler on BTstack timer expiration.

    // Is adapter enabled.
    if (self->adapter_state != ADAPTER_STATE_ENABLED) {
        return;
    }
    // Is scan in progress?
    if (self->scan_results == NULL || shared_module_bleio_scanresults_get_done(self->scan_results)) {
        return;
    }
    // Stop scanning.
    gap_stop_scan();
    // Stop the scan timer.
    btstack_run_loop_remove_timer(&scan_timer);
    // Mark the scan results as done.
    shared_module_bleio_scanresults_set_done(self->scan_results, true);
    self->scan_results = NULL;
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
    if (self->adapter_state != ADAPTER_STATE_ENABLED) {
        return;
    }
}

void bleio_adapter_gc_collect(bleio_adapter_obj_t *adapter) {
    // We divide by size_t so that we can scan each 32-bit aligned value to see
    // if it is a pointer. This allows us to change the structs without worrying
    // about collecting new pointers.
    gc_collect_root((void **)adapter, sizeof(bleio_adapter_obj_t) / (sizeof(size_t)));
    gc_collect_root((void **)bleio_connections, sizeof(bleio_connections) / (sizeof(size_t)));
}

void bleio_adapter_reset(bleio_adapter_obj_t *adapter) {
    // Stop scanning and advertising.
    common_hal_bleio_adapter_stop_scan(adapter);
    common_hal_bleio_adapter_stop_advertising(adapter);
    // Sever all connections.
    // TODO
    // Block until all connections are closed or timeout.
    adapter->scan_results = NULL;
    // adapter->current_advertising_data = NULL;
    // adapter->advertising_data = NULL;
    // adapter->scan_response_data = NULL;
}
