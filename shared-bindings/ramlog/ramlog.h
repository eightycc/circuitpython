// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Bob Abeles
//
// SPDX-License-Identifier: MIT

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "py/obj.h"

extern const mp_obj_type_t ramlog_ramlog_type;

#define RAM_LOG_WRITE_BUF_SIZE 132
#define RAM_LOG_TIMESTAMP_BUF_SIZE 64

typedef struct _ramlog_ramlog_obj_t {
    mp_obj_base_t base;

    uint64_t prev_log_time;

    size_t read_pos;
    size_t read_limit;
    size_t read_start_offset;
    size_t timestamp_pos;
    uint8_t timestamp_buf[RAM_LOG_TIMESTAMP_BUF_SIZE + 1];
    bool timestamp_emitting;
    bool prev_nl;
    bool read_eof;

    size_t write_pos;
    uint8_t write_buf[RAM_LOG_WRITE_BUF_SIZE + 1];
} ramlog_ramlog_obj_t;

extern ramlog_ramlog_obj_t ramlog_ramlog_obj;
