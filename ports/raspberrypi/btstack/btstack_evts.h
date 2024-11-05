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
