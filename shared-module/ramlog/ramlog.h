// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Bob Abeles
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "py/mpconfig.h"
#include "py/mpprint.h"

// A 32-bit value that identifies the ramlog_t object in memory. It is the string
// 'raml' in little-endian order. Along with its complement, used to find a
// ramlog_t object in unallocated memory during recovery.
#define RAM_LOG_MAGIC (0x6c6d6172)

// The ramlog_t object encapsulates a RAM log circular buffer and its controls.
typedef struct _ramlog_t {
    uint32_t magic;             // Magic id number
    uint32_t c_magic;           // ...and its complement
    uint64_t last_log_time;     // Last log message time in subticks (1/32768 second)
    mp_uint_t log_buf_sz;       // Size of the log buffer in bytes
    mp_uint_t next_idx;         // Index of next byte to write in the log buffer
    bool logging_enabled;
    bool wrap_enabled;          // Wrap log buffer around when full
    bool wrapped;               // Log buffer has wrapped around since last reset
    bool prev_nl;               // Previous log byte was a '\n'
    uint8_t log_buf[];
} ramlog_t;

extern volatile ramlog_t *_ramlog;

extern const mp_print_t ramlog_print;

size_t ramlog_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void ramlog_print_hexdump(const char *prefix, const uint8_t *buf, size_t len);

bool ramlog_init(uint32_t buf_sz);
void ramlog_deinit(void);
bool ramlog_recover(void);
void ramlog_enable(bool enable);
bool ramlog_get_enable(void);
void ramlog_wrap_enable(bool enable);
bool ramlog_get_wrap_enable(void);
void ramlog_reset(void);
