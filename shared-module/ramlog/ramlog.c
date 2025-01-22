// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Bob Abeles
//
// SPDX-License-Identifier: MIT

#include <stdarg.h>
#include <string.h>

#include "py/mpconfig.h"
#include "py/mphal.h"

#include "supervisor/port.h"
#include "supervisor/shared/serial.h"
#include "shared-module/ramlog/ramlog.h"

// A logging service that writes to a circular buffer in RAM. It is useful for low-level
// CP debugging of the core and external libraries. The interface follows mp-print
// conventions so it can be easily adapted to log messages from external libraries.
// Because it writes to RAM, its timing may not interfere as much with code under test
// as writing to a file or serial uart.
//
// A companion Python module, ramlog, gives Python code access to the facility. This
// is useful for controlling logging and for extracting logs for analysis. To enable
// logging from Python code, the RamLog singleton that represents the RAM logging
// service adheres to the stream protocol, so it can be connected to adafruit-logging
// as a stream handler.
//
// Once a log is captured, the stream protocol can be used to extract the log to
// a file or some other destination.

static void ramlog_print_callback(void *env, const char *str, size_t len);
const mp_print_t ramlog_print = {NULL, ramlog_print_callback};

// Making this struct volatile guarantees that the compiler will not reorder
// accesses to its fields or optimize them away.
volatile ramlog_t *_ramlog = NULL;

// Combines current time in ticks (1/1024 second) and subticks (1/32768 second)
// into a single 64-bit value representing current time in subticks. The
// effective range for a full 64-bit timestamp is about 18 million years.
static uint64_t get_time_subticks(void) {
    uint8_t subticks;
    uint64_t ticks = port_get_raw_ticks(&subticks);
    return ticks * 32 + subticks;
}

bool ramlog_init(uint32_t buf_sz) {
    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    if (_ramlog != NULL) {
        ramlog_deinit();
    }
    // The ramlog_t object and the buffer are allocated outside the garbage
    // collected heap to preserve the log across vm restarts.
    size_t alloc_sz = sizeof(ramlog_t) + buf_sz;
    _ramlog = port_malloc(alloc_sz, false);
    if (_ramlog == NULL) {
        MICROPY_END_ATOMIC_SECTION(state_save);
        return false;
    }
    _ramlog->log_buf_sz = buf_sz;
    _ramlog->next_idx = 0;
    _ramlog->logging_enabled = false;
    _ramlog->wrap_enabled = false;
    _ramlog->wrapped = false;
    _ramlog->prev_nl = true;
    _ramlog->last_log_time = get_time_subticks();
    _ramlog->magic = RAM_LOG_MAGIC;
    _ramlog->c_magic = ~RAM_LOG_MAGIC;
    MICROPY_END_ATOMIC_SECTION(state_save);
    return true;
}

void ramlog_deinit(void) {
    if (_ramlog == NULL) {
        return;
    }
    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    assert(_ramlog->magic == RAM_LOG_MAGIC);
    // clear magic to prevent accidental recovery of a deleted log
    _ramlog->magic = _ramlog->c_magic = 0;
    port_free((ramlog_t *)_ramlog);
    MICROPY_END_ATOMIC_SECTION(state_save);
}

// After a reset that doesn't involve a power cycle, the log may persist in RAM
// or PSRAM. Free areas of the tlsf heap are scanned for a ramlog_t magic number
// and its complement. If found, the ramlog_t is re-allocated at its original
// location and logging is disabled.
bool ramlog_recover(void) {
    // scan free blocks on the tlsf heap for a ramlog_t magic number
    // TODO: this may be pointless if the heap is zeroed on reset
    return false;
}

void ramlog_enable(bool enable) {
    if (_ramlog == NULL) {
        return;
    }
    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    assert(_ramlog->magic == RAM_LOG_MAGIC);
    _ramlog->logging_enabled = enable;
    MICROPY_END_ATOMIC_SECTION(state_save);
}

bool ramlog_get_enable(void) {
    if (_ramlog == NULL || !_ramlog->logging_enabled) {
        return false;
    }
    return true;
}

void ramlog_wrap_enable(bool enable) {
    if (_ramlog == NULL) {
        return;
    }
    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    assert(_ramlog->magic == RAM_LOG_MAGIC);
    _ramlog->wrap_enabled = enable;
    MICROPY_END_ATOMIC_SECTION(state_save);
}

bool ramlog_get_wrap_enable(void) {
    if (_ramlog == NULL || !_ramlog->wrap_enabled) {
        return false;
    }
    return true;
}

void ramlog_reset(void) {
    if (_ramlog == NULL) {
        return;
    }
    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    assert(_ramlog->magic == RAM_LOG_MAGIC);
    _ramlog->next_idx = 0;
    _ramlog->logging_enabled = false;
    _ramlog->wrap_enabled = false;
    _ramlog->wrapped = false;
    _ramlog->prev_nl = true;
    _ramlog->last_log_time = get_time_subticks();
    MICROPY_END_ATOMIC_SECTION(state_save);
}

// All output to the log goes through this function. The str arg is fully
// formatted by mp_print machinery. The string is "cooked" while it is
// entered into the log buffer, i.e., '\n' is replaced by '\r\n'.
static inline bool log_buf_putc(char c) {
    _ramlog->log_buf[_ramlog->next_idx++] = c;
    if (_ramlog->next_idx >= _ramlog->log_buf_sz) {
        _ramlog->next_idx = 0;
        _ramlog->wrapped = true;
        if (!_ramlog->wrap_enabled) {
            _ramlog->logging_enabled = false;
            return false;
        }
    }
    return true;
}

static void ramlog_print_callback(void *env, const char *str, size_t len) {
    (void)env;
    // To support simultaneous logging from IRQ level and user level,
    // interrupts are disabled while manipulating the log buffer. This
    // avoids corruption in the event of a race. This does not prevent
    // checkering of IRQ/user log entries, a problem that can be solved
    // by user code creating a critical section around calls to write
    // to the log.
    if (_ramlog == NULL || !_ramlog->logging_enabled) {
        return;
    }

    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();
    assert(_ramlog->magic == RAM_LOG_MAGIC);

    while (len > 0) {
        if (_ramlog->prev_nl) {
            uint64_t now = get_time_subticks();
            _ramlog->last_log_time = now;
            // Emit 63-bit time stamp. Timestamp is encoded as 7-bit digits and
            // stored into 9 log buffer bytes in big-endian order. The high
            // order bit of each buffer byte is set to 1 to force digits into
            // the range 0x80 to 0xff. This guarantees that no '\n' characters
            // will appear in the timestamp.
            int bits = 63;
            while (bits > 0) {
                uint8_t ch = ((now >> 57) & 0x7f) | 0x80;
                if (!log_buf_putc(ch)) {
                    MICROPY_END_ATOMIC_SECTION(state_save);
                    return;
                }
                bits -= 7;
                now <<= 7;
            }
            _ramlog->prev_nl = false;
        }
        size_t print_len = 0;
        while (print_len < len) {
            uint8_t ch = str[print_len++];
            if (!log_buf_putc(ch)) {
                MICROPY_END_ATOMIC_SECTION(state_save);
                return;
            }
            if (ch == '\n') {
                _ramlog->prev_nl = true;
                break;
            }
        }
        len -= print_len;
    }

    MICROPY_END_ATOMIC_SECTION(state_save);
}

// RAM logging calls are wrapped in a critical section to ensure the integrity
// of the log buffer and prevent message checkering between IRQ and user code.
// Macros should invoke these wrapper functions.
size_t ramlog_printf(const char *fmt, ...) {
    if (_ramlog == NULL || !_ramlog->logging_enabled) {
        return 0;
    }

    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();

    assert(_ramlog->magic == RAM_LOG_MAGIC);

    va_list args;
    va_start(args, fmt);
    size_t len = mp_vprintf(&ramlog_print, fmt, args);
    va_end(args);

    MICROPY_END_ATOMIC_SECTION(state_save);
    return len;
}

void ramlog_print_hexdump(const char *prefix, const uint8_t *buf, size_t len) {
    if (_ramlog == NULL || !_ramlog->logging_enabled) {
        return;
    }

    mp_uint_t state_save = MICROPY_BEGIN_ATOMIC_SECTION();

    assert(_ramlog->magic == RAM_LOG_MAGIC);

    print_hexdump(&ramlog_print, prefix, buf, len);

    MICROPY_END_ATOMIC_SECTION(state_save);
}
