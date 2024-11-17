// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>

// BTstack event handler.
//
// BTstack delivers events to a set of registered event handlers. BTstack will
// call all of the registered event handlers for each event. We interpose our
// own single event handler to allow CP handlers to filter out handled events
// and to allow us to bind custom context to the CP handler.
//
// BTstack events are delivered synchronously with respect to VM execution.

// Event callback function type.
//   Arg 1 is a pointer to the event
//   Arg 2 is the size of the event
//   Arg 3 is a pointer to the context for this callback
//   Returns true if event was handled
typedef bool (*bleio_evt_handler_t)(uint8_t *, uint16_t, void *);

typedef struct bleio_evt_handler_entry {
    struct bleio_evt_handler_entry *next;
    bleio_evt_handler_t callback;
    void *context;
} bleio_evt_handler_entry_t;

void bleio_evts_init(void);
void bleio_evts_reset(void);
void bleio_evt_remove_heap_handlers(void);
void bleio_evt_add_event_handler(bleio_evt_handler_t callback, void *context);
void bleio_evt_add_event_handler_entry(bleio_evt_handler_entry_t *entry, bleio_evt_handler_t callback, void *context);
void bleio_evt_remove_event_handler(bleio_evt_handler_t callback, void *context);
