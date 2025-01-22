// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2022 Damien P. George
//
// SPDX-License-Identifier: MIT
#pragma once

// The board-level config will be included here, so it can set some CYW43 values.
#include "py/mpconfig.h"
#include "py/mperrno.h"
#include "py/mphal.h"

#include "supervisor/port.h"

#include "sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h"

#define CYW43_NETUTILS                  (1)

#if CIRCUITPY_USB_DEVICE
#include "supervisor/usb.h"
#define CYW43_EVENT_POLL_HOOK usb_background();
#else
#define CYW43_EVENT_POLL_HOOK
#endif

#include <stdio.h>
#include "supervisor/shared/serial.h"
#include "shared-module/ramlog/ramlog.h"
#define CYW43_PRINTF(...)               CIRCUITPY_RAMLOG_PRINTF(__VA_ARGS__)
#define CYW43_VERBOSE_DEBUG             (1)
#define CYW43_VDEBUG(...)               CYW43_PRINTF(__VA_ARGS__)
#define CYW43_HEXDUMP(prefix, buf, len) CIRCUITPY_RAMLOG_HEXDUMP(prefix, buf, len)

void cyw43_post_poll_hook(void);
extern volatile int cyw43_has_pending;

static inline void cyw43_yield(void) {
    uint32_t my_interrupts = save_and_disable_interrupts();
    if (!cyw43_has_pending) {
        port_idle_until_interrupt();
    }
    restore_interrupts(my_interrupts);
}
