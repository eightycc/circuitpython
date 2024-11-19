// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

#include <stdbool.h>
#include <stdio.h>

#include "btstack_evts.h"
#include "btstack.h"

#include "py/mpstate.h"
#include "py/runtime.h"
#include "py/gc.h"

// Because BTstack events don't provide a means of passing a user context pointer to
// event handlers, we use a two handler scheme. The top-level event handler is
// registered with BTstack. For each BTstack HCI event the top-level handler
// dispatches CP event handlers in the order they were registered. Once a CP event
// handler has handled an event it returns true and the top-level handler stops
// dispatching the event to other CP event handlers.
//
// CP event handlers have one of two different lifetimes. Handlers created dynamically
// on the CP heap live from creation until they are either individually removed by
// bleio_evt_remove_event_handler() or until bleio_user_reset() invokes
// bleio_evt_remove_heap_handlers() to remove them all in preparation for a heap
// reset. Handlers created statically on the CP stack live until they are either
// removed individually or until bleio_reset() invokes bleio_evt_reset() to remove
// all CP event handlers in preparation for a full reset of the Bluetooth stack.
//
// We register our top-level event handler with BTstack once to begin the lifetime
// of our registration. The btstack_evt_handler_registered static guards against
// registration of an already registered handler.

static bool btstack_evt_handler_registered = false;
static btstack_packet_callback_registration_t btstack_evt_handler_entry;

// Handle an HCI event from BTstack by calling registered CP event handlers.
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
static void register_btstack_evt_handler(void) {
    btstack_evt_handler_entry.callback = &btstack_evt_handler;
    hci_add_event_handler(&btstack_evt_handler_entry);
    btstack_evt_handler_registered = true;
}

static void deregister_btstack_evt_handler(void) {
    if (btstack_evt_handler_registered) {
        // This call is fatal if the handler is not registered.
        hci_remove_event_handler(&btstack_evt_handler_entry);
        btstack_evt_handler_registered = false;
    }
}

void bleio_evt_init(void) {
    // bleio_evt_init() not serially reusable unless unregistered first.
    assert(!btstack_evt_handler_registered);
    // Initialize our list of CP event handlers.
    MP_STATE_VM(bleio_evt_handler_entries) = NULL;
    // Register our event handler with BTstack.
    register_btstack_evt_handler();
}

void bleio_evt_deinit(void) {
    // Remove our event handler from BTstack.
    deregister_btstack_evt_handler();
    // Remove all CP event handlers.
    MP_STATE_VM(bleio_evt_handler_entries) = NULL;
}

// The following functions apply to CP event handlers:

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
