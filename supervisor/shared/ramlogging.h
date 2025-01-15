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

#define RAM_LOG_MAGIC (0x6c6d6172)

typedef struct _ram_log_t {
    uint32_t magic;
    uint32_t c_magic;
    // Time is measured in subticks of 1/32768 second (approx. 30.5us) each.
    uint64_t last_log_time;
    // uint8_t *log_buf;
    mp_uint_t log_buf_sz;
    mp_uint_t next_idx;
    bool logging_enabled;
    bool wrap_enabled;
    bool wrapped;
    bool prev_nl;
    uint8_t log_buf[];
} ram_log_t;

extern volatile ram_log_t *_ram_log;

extern const mp_print_t ram_log_print;

size_t ram_log_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
void ram_log_print_hexdump(const char *prefix, const uint8_t *buf, size_t len);

bool ram_log_init(uint32_t buf_sz);
void ram_log_deinit(void);
bool ram_log_recover(void);
void ram_log_enable(bool enable);
bool ram_log_get_enable(void);
void ram_log_wrap_enable(bool enable);
bool ram_log_get_wrap_enable(void);
void ram_log_reset(void);
