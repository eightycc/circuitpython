/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
 * Copyright (c) 2019 Dan Halbert for Adafruit Industries
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

#include <stdbool.h>
#include <stdio.h>

#include "btstack_evts.h"
#include "btstack.h"

#include "py/mpstate.h"
#include "py/runtime.h"
#include "py/gc.h"

// We register a single event handler with BTstack. Registration with BTstack happens once
// and is invoked by btstack_evts_init().

static bool btstack_evt_handler_registered = false;
static btstack_packet_callback_registration_t btstack_evt_handler_entry;

// Handle an HCI event from BTstack by calling registered CP event handlers. Event handlers
// are called in the order they were registered until one returns true or the list is
// exhausted.
//  packet_type: HCI packet type
//  channel: HCI channel
//  packet: HCI packet
//  size: HCI packet size
static void btstack_evt_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    // We care only for HCI events.
    if (packet_type != HCI_EVENT_PACKET) {
        return;
    }
    // Notify our CP event handlers until event is handled or list is exhausted.
    bleio_evt_handler_entry_t *handler = MP_STATE_VM(bleio_evt_handler_entries);
    while (handler != NULL) {
        if (handler->callback(packet, size, handler->context)) {
            return;
        }
        handler = handler->next;
    }
}

// Register our event handler with BTstack.
// hci_xxx_event_handler() functions guard against double registration and null removal.
static void register_btstack_evt_handler(void) {
    btstack_evt_handler_entry.callback = &btstack_evt_handler;
    hci_add_event_handler(&btstack_evt_handler_entry);
    btstack_evt_handler_registered = true;
}

static void remove_btstack_evt_handler(void) {
    if (btstack_evt_handler_registered) {
        // This call is fatal if the handler is not registered.
        hci_remove_event_handler(&btstack_evt_handler_entry);
        btstack_evt_handler_registered = false;
    }
}

void bleio_evts_init(void) {
    // Ignore double init (maybe an assert?)
    if (btstack_evt_handler_registered) {
        return;
    }
    // Initialize our list of CP event handlers.
    MP_STATE_VM(bleio_evt_handler_entries) = NULL;
    // Register our event handler with BTstack.
    register_btstack_evt_handler();
}

void bleio_evts_reset(void) {
    // Remove our event handler from BTstack.
    remove_btstack_evt_handler();
    // Remove all CP event handlers.
    MP_STATE_VM(bleio_evt_handler_entries) = NULL;
}

void bleio_evt_remove_heap_handlers(void) {
    bleio_evt_handler_entry_t *it = MP_STATE_VM(bleio_evt_handler_entries);
    while (it != NULL) {
        // If the context is on the heap, then delete the handler.
        if (gc_ptr_on_heap(it->context)) {
            bleio_evt_remove_event_handler(it->callback, it->context);
        }
        it = it->next;
    }
}

void bleio_evt_add_event_handler_entry(bleio_evt_handler_entry_t *entry, bleio_evt_handler_t callback, void *context) {
    bleio_evt_handler_entry_t *it = MP_STATE_VM(bleio_evt_handler_entries);
    bleio_evt_handler_entry_t **prev = &MP_STATE_VM(bleio_evt_handler_entries);
    while (it != NULL) {
        // If event handler and its corresponding context are already on the list, don't add again.
        if ((it->callback == callback) && (it->context == context)) {
            return;
        }
        prev = &(it->next);
        it = it->next;
    }
    // Add new handler to the tail of the list.
    entry->next = *prev;
    entry->context = context;
    entry->callback = callback;
    *prev = entry;
}

void bleio_evt_add_event_handler(bleio_evt_handler_t callback, void *context) {
    bleio_evt_handler_entry_t *it = MP_STATE_VM(bleio_evt_handler_entries);
    while (it != NULL) {
        // If event handler and its corresponding context are already on the list, don't add again.
        if ((it->callback == callback) && (it->context == context)) {
            return;
        }
        it = it->next;
    }

    // Add a new handler to the tail of the list
    bleio_evt_handler_entry_t *handler = m_new(bleio_evt_handler_entry_t, 1);
    bleio_evt_add_event_handler_entry(handler, callback, context);
}

void bleio_evt_remove_event_handler(bleio_evt_handler_t callback, void *context) {
    bleio_evt_handler_entry_t *it = MP_STATE_VM(bleio_evt_handler_entries);
    bleio_evt_handler_entry_t **prev = &MP_STATE_VM(bleio_evt_handler_entries);
    while (it != NULL) {
        if ((it->callback == callback) && (it->context == context)) {
            // Splice out the matching handler.
            *prev = it->next;
            // Clear next of the removed node so it's clearly not in a list.
            it->next = NULL;
            return;
        }
        prev = &(it->next);
        it = it->next;
    }
}

// List of active BTstack event handlers.
MP_REGISTER_ROOT_POINTER(bleio_evt_handler_entry_t * bleio_evt_handler_entries);
