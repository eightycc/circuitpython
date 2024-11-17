// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019-2020 Jim Mussared
// SPDX-FileCopyrightText: Copyright (c) 2018 Ayke van Laethem
// SPDX-FileCopyrightText: Copyright (c) 2018 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

#include "btstack.h"

#include "shared-bindings/_bleio/__init__.h"
#include "shared-bindings/_bleio/Attribute.h"
#include "shared-bindings/_bleio/Connection.h"

// BTstack delivers GATT client responses as events routed to this handler.
static void gattc_read_response_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    // Filter out all but characteristic value query status or result.
    uint8_t event_type = hci_event_packet_get_type(packet);
    uint16_t conn_handle;
    switch (event_type) {
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT:
            conn_handle = gatt_event_characteristic_value_query_result_get_handle(packet);
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            conn_handle = gatt_event_query_complete_get_handle(packet);
            break;
        default:
            return;
    }
    // Test connection handle validity.
    // if (conn_handle == 0) {
    //    return;
    // }

    // Locate the CP internal connection object associated with this response
    bleio_connection_internal_t *connection = bleio_conn_handle_to_connection(conn_handle);

    switch (event_type) {
        case GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT: {
            // Unpack event.
            uint16_t value_handle = gatt_event_characteristic_value_query_result_get_value_handle(packet);
            uint16_t len = gatt_event_characteristic_value_query_result_get_value_length(packet);
            const uint8_t *data = gatt_event_characteristic_value_query_result_get_value(packet);

            // Locate read context.
            if (connection->read_context == NULL) {
                return;
            }
            bleio_gattc_read_t *read = connection->read_context;

            // Verify that the value handle matches the one we requested.
            if (value_handle != read->value_handle) {
                return;
            }

            // Truncate read if necessary.
            len = MIN(len, read->len);
            // Copy data to user buffer.
            memcpy(read->buf, data, len);
            read->len = len;
            // Mark read as complete with no status.
            read->att_status = 0;
            read->done = true;
            break;
        }
        case GATT_EVENT_QUERY_COMPLETE: {
            // Unpack event.
            uint16_t status = gatt_event_query_complete_get_att_status(packet);
            // Locate read context.
            if (connection->read_context == NULL) {
                return;
            }
            bleio_gattc_read_t *read = connection->read_context;
            // Mark read as complete with status.
            read->att_status = status;
            read->done = true;
            break;
        }
    }
}

size_t common_hal_bleio_gattc_read(uint16_t handle, uint16_t conn_handle, uint8_t *buf, size_t len) {
    common_hal_bleio_check_connected(conn_handle);
    bleio_connection_internal_t *connection = bleio_conn_handle_to_connection(conn_handle);

    // Finding a non-NULL read_context indicates an internal logic error.
    if (connection->read_context != NULL) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Bluetooth internal logic error"));
    }

    // Construct context for read response events on stack.
    bleio_gattc_read_t read;
    read.buf = buf;
    read.len = len;
    read.att_status = 0;
    read.done = false;
    connection->read_context = &read;

    // Initiate GATT client read.
    uint8_t err = gatt_client_read_value_of_characteristic_using_value_handle(&gattc_read_response_handler, conn_handle, handle);
    if (err != ERROR_CODE_SUCCESS) {
        connection->read_context = NULL;
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("Bluetooth error %x reading value"), err);
    }

    // Wait for read to complete. BTstack has a hard-coded timeout of 30 seconds after which it will
    // abort the read and return ATT_ERROR_TIMEOUT via the event handler. Likewise, other ATT errors
    // will be returned via the event handler.

    // TODO: Consider adding additional timeout logic here in case BTstack gets stuck.
    while (!read.done) {
        RUN_BACKGROUND_TASKS;
    }

    // Remove read context from connection.
    connection->read_context = NULL;

    // Analyze ATT read completion status, raise exception if error.
    if (read.att_status != ATT_ERROR_SUCCESS) {
        mp_raise_bleio_BluetoothError(MP_ERROR_TEXT("ATT error %x reading value"), read.att_status);
    }

    return read.len;
}

void common_hal_bleio_gattc_write(uint16_t handle, uint16_t conn_handle,
    mp_buffer_info_t *bufinfo, bool write_no_response) {
}

size_t common_hal_bleio_gatts_read(uint16_t handle, uint16_t conn_handle, uint8_t *buf, size_t len) {
    // TODO: Implement.
    return 0;
}

void common_hal_bleio_gatts_write(uint16_t handle, uint16_t conn_handle, mp_buffer_info_t *bufinfo) {

}

// BTstack does not provide a GATT server database, so we adapt the implementation from Micropython.

void bleio_gatts_db_create(bleio_gatts_db_t *db) {
    *db = m_new(mp_map_t, 1);
}

void bleio_gatts_db_create_entry(bleio_gatts_db_t db, uint16_t handle, size_t len) {
    bleio_gatts_db_entry_t *entry = m_new(bleio_gatts_db_entry_t, 1);
    entry->buf = m_new(uint8_t, len);
    mp_map_elem_t *elem = mp_map_lookup(db, MP_OBJ_NEW_SMALL_INT(handle), MP_MAP_LOOKUP_ADD_IF_NOT_FOUND);
    entry->alloc_sz = len;
    entry->len = 0;
    entry->append = false;
    elem->value = MP_OBJ_FROM_PTR(entry);
}

bleio_gatts_db_entry_t *bleio_gatts_db_lookup(bleio_gatts_db_t db, uint16_t handle) {
    mp_map_elem_t *elem = mp_map_lookup(db, MP_OBJ_NEW_SMALL_INT(handle), MP_MAP_LOOKUP);
    if (!elem) {
        return NULL;
    }
    return MP_OBJ_TO_PTR(elem->value);
}

void bleio_gatts_db_read(bleio_gatts_db_t db, uint16_t handle, const uint8_t **value, size_t *value_len) {
    bleio_gatts_db_entry_t *entry = bleio_gatts_db_lookup(db, handle);
    if (entry) {
        *value = entry->buf;
        *value_len = entry->len;
    } else {
        // For empty handles, return NULL.
        *value = NULL;
        *value_len = 0;
    }
}

void bleio_gatts_db_write(bleio_gatts_db_t db, uint16_t handle, const uint8_t *value, size_t value_len) {
    bleio_gatts_db_entry_t *entry = bleio_gatts_db_lookup(db, handle);
    if (entry) {
        if (value_len > entry->alloc_sz) {
            uint8_t *buf = m_new(uint8_t, value_len);
            entry->buf = buf;
            entry->alloc_sz = value_len;
        }

        memcpy(entry->buf, value, value_len);
        entry->len = value_len;
    }
}
