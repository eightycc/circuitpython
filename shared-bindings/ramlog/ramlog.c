// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Bob Abeles
//
// SPDX-License-Identifier: MIT

#include <string.h>

#include "py/obj.h"
#include "py/objproperty.h"
#include "py/runtime.h"
#include "py/stream.h"
#include "shared-bindings/ramlog/__init__.h"
#include "shared-bindings/ramlog/ramlog.h"
#include "shared-module/ramlog/ramlog.h"

ramlog_ramlog_obj_t ramlog_ramlog_obj = {{&ramlog_ramlog_type}};

static void init_read_pos(ramlog_ramlog_obj_t *self);
static void pos_oldest_complete_message(ramlog_ramlog_obj_t *self);

//| class RamLog:
//|     """RamLog"""
//|
//|     def __init__(self) -> None:
//|         """This class represents CircuitPython's RAM logging facility. It is a singleton and will always return the same instance."""
//|         ...
static mp_obj_t ramlog_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // No arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return constant object
    return (mp_obj_t)&ramlog_ramlog_obj;
}

//|     def init(self, size: int) -> None:
//|         """Initialize the RAM log with a buffer of the given size."""
//|         ...
static mp_obj_t ramlog_ramlog_init(mp_obj_t self_in, mp_obj_t size) {
    if (!ramlog_init(mp_obj_get_int(size))) {
        mp_raise_msg(&mp_type_RuntimeError, MP_ERROR_TEXT("Failed to allocate RAM log buffer"));
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(ramlog_ramlog_init_obj, ramlog_ramlog_init);

//|     def deinit(self) -> None:
//|         """Deinitialize the RAM log."""
//|         ...
static mp_obj_t ramlog_ramlog_deinit(mp_obj_t self_in) {
    ramlog_deinit();
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ramlog_ramlog_deinit_obj, ramlog_ramlog_deinit);

//|     def reset(self) -> None:
//|         """Reset the RAM log."""
//|         ...
static mp_obj_t ramlog_ramlog_reset(mp_obj_t self_in) {
    ramlog_reset();
    ramlog_ramlog_obj_t *self = MP_OBJ_TO_PTR(self_in);
    init_read_pos(self);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_1(ramlog_ramlog_reset_obj, ramlog_ramlog_reset);

//|     def log_enable(self, enable: Bool) -> Bool:
//|         """Enable or disable logging."""
//|         ...
static mp_obj_t ramlog_ramlog_get_log_enable(mp_obj_t self_in) {
    bool log_enable = ramlog_get_enable();
    return mp_obj_new_bool(log_enable);
}
MP_DEFINE_CONST_FUN_OBJ_1(ramlog_ramlog_get_log_enable_obj, ramlog_ramlog_get_log_enable);

static mp_obj_t ramlog_ramlog_set_log_enable(mp_obj_t self_in, mp_obj_t log_enable) {
    ramlog_ramlog_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ramlog_enable(mp_obj_is_true(log_enable));
    if (!mp_obj_is_true(log_enable)) {
        init_read_pos(self);
    }
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(ramlog_ramlog_set_log_enable_obj, ramlog_ramlog_set_log_enable);

MP_PROPERTY_GETSET(ramlog_ramlog_log_enable_obj,
    (mp_obj_t)&ramlog_ramlog_get_log_enable_obj,
    (mp_obj_t)&ramlog_ramlog_set_log_enable_obj);

//|     def wrap_enable(self, enable: Bool) -> Bool:
//|         """Enable or disable log wrap."""
//|         ...
//|
static mp_obj_t ramlog_ramlog_get_wrap_enable(mp_obj_t self_in) {
    bool wrap_enable = ramlog_get_wrap_enable();
    return mp_obj_new_bool(wrap_enable);
}
MP_DEFINE_CONST_FUN_OBJ_1(ramlog_ramlog_get_wrap_enable_obj, ramlog_ramlog_get_wrap_enable);

static mp_obj_t ramlog_ramlog_set_wrap_enable(mp_obj_t self_in, mp_obj_t wrap_enable) {
    ramlog_wrap_enable(mp_obj_is_true(wrap_enable));
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(ramlog_ramlog_set_wrap_enable_obj, ramlog_ramlog_set_wrap_enable);

MP_PROPERTY_GETSET(ramlog_ramlog_wrap_enable_obj,
    (mp_obj_t)&ramlog_ramlog_get_wrap_enable_obj,
    (mp_obj_t)&ramlog_ramlog_set_wrap_enable_obj);

static void init_read_pos(ramlog_ramlog_obj_t *self) {
    self->read_pos = 0;
    self->read_start_offset = 0;
    self->timestamp_pos = 0;
    self->timestamp_emitting = false;
    self->prev_nl = true;
    self->read_eof = false;
    self->prev_log_time = 0;
    self->read_limit = _ramlog->log_buf_sz;
    pos_oldest_complete_message(self);
    self->read_limit = (_ramlog->wrapped)?
        _ramlog->log_buf_sz - self->read_start_offset :
        _ramlog->next_idx;
}

// Increment the read position. Sets EOF indicator when final log byte is passed.
inline static void increment_read_pos(ramlog_ramlog_obj_t *self) {
    if (self->read_eof) {
        return;
    }
    if (++self->read_pos >= self->read_limit) {
        self->read_eof = true;
    }
}

inline static mp_uint_t physical_read_pos(ramlog_ramlog_obj_t *self) {
    if (_ramlog->wrapped && _ramlog->wrap_enabled) {
        return ((_ramlog->next_idx + self->read_start_offset + self->read_pos) < _ramlog->log_buf_sz)?
               _ramlog->next_idx + self->read_start_offset + self->read_pos :
               _ramlog->next_idx + self->read_start_offset + self->read_pos - _ramlog->log_buf_sz;
    }
    return self->read_pos;
}

// Decode timestamp from 9 x 7-bit encoded bytes to a 63-bit integer.
inline static uint64_t decode_timestamp(ramlog_ramlog_obj_t *self) {
    size_t bits = 63;
    uint64_t timestamp = 0;
    while (bits > 0) {
        if (self->read_eof) {
            return 0;
        }
        uint8_t ts_byte = _ramlog->log_buf[physical_read_pos(self)];
        increment_read_pos(self);
        assert(ts_byte & 0x80);
        timestamp = (timestamp << 7) | (ts_byte & 0x7f);
        bits -= 7;
    }
    return timestamp;
}

// Get next byte from the log. Returns formatted timestamp bytes as necessary.
inline static uint8_t ramlog_getc(ramlog_ramlog_obj_t *self) {
    if (self->read_eof) {
        return 0;
    }
    // return formatted timestamp byte
    if (self->timestamp_emitting) {
        if (self->timestamp_buf[self->timestamp_pos] == '\0') {
            self->timestamp_emitting = false;
        } else {
            return self->timestamp_buf[self->timestamp_pos++];
        }
    }
    // return log message byte
    uint8_t log_byte = _ramlog->log_buf[physical_read_pos(self)];
    increment_read_pos(self);
    return log_byte;
}

// Return the position of the first timestamp byte of the oldest complete
// message in the buffer.
static void pos_oldest_complete_message(ramlog_ramlog_obj_t *self) {
    // If buffer has never wrapped or it is stopped at its end, oldest complete message is at 0
    if (!(_ramlog->wrapped && _ramlog->wrap_enabled)) {
        if (self->read_pos == _ramlog->next_idx) {
            self->read_eof = true;
        }
        // self->read_pos = 0;
        // self->read_start_offset = 0;
        // self->prev_nl = true;
        return;
    }
    // self->read_start_offset = _ramlog->next_idx;
    while (ramlog_getc(self) != '\n' && !self->read_eof) {
        ;
    }
    if (!self->read_eof) {
        self->read_start_offset = self->read_pos;
        self->read_pos = 0;
        self->prev_nl = true;
    }
}

static void timestamp_print_cb(void *env, const char *str, size_t len) {
    ramlog_ramlog_obj_t *self = (ramlog_ramlog_obj_t *)env;
    if (len > 0) {
        size_t remaining = len;
        while (remaining-- > 0) {
            assert(self->timestamp_pos < RAM_LOG_TIMESTAMP_BUF_SIZE);
            self->timestamp_buf[self->timestamp_pos++] = *str++;
        }
    }
}

static void format_timestamp(ramlog_ramlog_obj_t *self) {
    // if (self->timestamp_emitting) {
    //    return;
    // }
    assert(!self->timestamp_emitting);
    uint64_t timestamp = decode_timestamp(self);
    if (self->read_eof) {
        return;
    }
    self->timestamp_emitting = true;
    self->prev_nl = false;
    // Convert timestamp in subticks (1/32768 second) to microseconds. Because the
    // conversion multiplies by 10^6, timestamp range is reduced from approx. 17 million
    // years to approx. 17 years.
    uint64_t timestamp_us = timestamp * 1000000 / 32768;
    uint64_t delta = timestamp_us - self->prev_log_time;
    self->prev_log_time = timestamp_us;
    mp_uint_t ts_us = timestamp_us % 1000;
    timestamp_us /= 1000;
    mp_uint_t ts_ms = timestamp_us % 1000;
    timestamp_us /= 1000;
    mp_uint_t ts_s = timestamp_us % 60;
    mp_uint_t ts_m = timestamp_us / 60;
    mp_uint_t del_us = delta % 1000;
    delta /= 1000;
    mp_uint_t del_ms = delta % 1000;
    delta /= 1000;
    mp_uint_t del_s = delta % 60;
    mp_uint_t del_m = delta / 60;

    // Use mp-printf to format timestamp as a string.
    self->timestamp_pos = 0;
    mp_print_t timestamp_print = {self, timestamp_print_cb};
    if (ts_m > 0) {
        mp_printf(&timestamp_print, "%lu:%02lu.%03lu.%03lu",
            ts_m, ts_s, ts_ms, ts_us);
    } else {
        mp_printf(&timestamp_print, "%02lu.%03lu.%03lu",
            ts_s, ts_ms, ts_us);
    }
    if (del_m > 0) {
        mp_printf(&timestamp_print, "(%lu:%02lu.%03lu.%03lu): ", del_m, del_s, del_ms, del_us);
    } else {
        mp_printf(&timestamp_print, "(%02lu.%03lu.%03lu): ", del_s, del_ms, del_us);
    }
    self->timestamp_buf[self->timestamp_pos] = '\0';
    self->timestamp_pos = 0;
}

// Read the next byte(s) from the current ramlog_t including the RAM log buffer.
// If there is no current ramlog_t or logging is active, return 0 indicating EOF.
static mp_uint_t ramlog_ramlog_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode) {
    ramlog_ramlog_obj_t *self = MP_OBJ_TO_PTR(self_in);

    if (_ramlog == NULL || _ramlog->logging_enabled || self->read_eof) {
        return 0;
    }

    mp_uint_t buf_remaining_sz = size;
    while (buf_remaining_sz-- > 0) {
        if (self->prev_nl) {
            format_timestamp(self);
            self->prev_nl = false;
        }
        if (self->read_eof) {
            break;
        }
        uint8_t ch = ramlog_getc(self);
        *(uint8_t *)buf++ = ch;
        if (ch == '\n') {
            self->prev_nl = true;
        }
    }

    return size - buf_remaining_sz;
}

static mp_uint_t ramlog_ramlog_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode) {
    if (_ramlog == NULL || !_ramlog->logging_enabled) {
        return 0;
    }
    // Print to log. RAM logging will insert a timestamp before each '\n' terminated message.
    // Because the write may be a fragment of a message, we accumulate message bytes in a
    // buffer until we encounter a '\n' or run out of buffer space. If the buffer is full,
    // the remaining bytes are printed on additional time-stamped lines as necessary.
    ramlog_ramlog_obj_t *self = MP_OBJ_TO_PTR(self_in);
    size_t remaining = size;
    while (remaining-- > 0) {
        uint8_t ch = *(const uint8_t *)buf++;
        if (self->write_pos >= RAM_LOG_WRITE_BUF_SIZE) {
            // Buffer is full. Print the buffer and reset.
            self->write_buf[self->write_pos] = '\n';
            ramlog_printf("%.*s", self->write_pos, self->write_buf);
            self->write_pos = 0;
        }
        self->write_buf[self->write_pos++] = ch;
        if (ch == '\n') {
            // Print the buffer and reset.
            ramlog_printf("%.*s", self->write_pos, self->write_buf);
            self->write_pos = 0;
        }
    }
    return size;
}

static mp_uint_t ramlog_ramlog_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    if (_ramlog == NULL || _ramlog->logging_enabled) {
        return 0;
    }

    ramlog_ramlog_obj_t *self = MP_OBJ_TO_PTR(self_in);
    switch (request) {
        case MP_STREAM_SEEK: {
            struct mp_stream_seek_t *s = (struct mp_stream_seek_t *)(uintptr_t)arg;
            mp_uint_t ref = 0;
            switch (s->whence) {
                case MP_SEEK_CUR:
                    ref = self->read_pos;
                    break;
                case MP_SEEK_END:
                    ref = sizeof(ramlog_t) + _ramlog->log_buf_sz;
                    break;
            }
            mp_uint_t new_pos = s->offset + ref;
            if (s->whence != MP_SEEK_SET && s->offset < 0) {
                if (new_pos > ref) {
                    new_pos = 0;
                }
            } else if (new_pos < ref) {
                *errcode = MP_EINVAL;
                return MP_STREAM_ERROR;
            }
            s->offset = self->read_pos = new_pos;
            return 0;
        }
        case MP_STREAM_FLUSH:
            return 0;
        case MP_STREAM_CLOSE:
            return 0;
        default:
            *errcode = MP_EINVAL;
            return MP_STREAM_ERROR;
    }
    return 0;
}

static const mp_stream_p_t ramlog_ramlog_stream_p = {
    .read = ramlog_ramlog_read,
    .write = ramlog_ramlog_write,
    .ioctl = ramlog_ramlog_ioctl,
    .is_text = false,
};

static const mp_rom_map_elem_t ramlog_ramlog_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    { MP_ROM_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&mp_stream_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_seek), MP_ROM_PTR(&mp_stream_seek_obj) },
    { MP_ROM_QSTR(MP_QSTR_tell), MP_ROM_PTR(&mp_stream_tell_obj) },
    { MP_ROM_QSTR(MP_QSTR_flush), MP_ROM_PTR(&mp_stream_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_stream_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&ramlog_ramlog_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&ramlog_ramlog_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&ramlog_ramlog_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_log_enable), MP_ROM_PTR(&ramlog_ramlog_log_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_wrap_enable), MP_ROM_PTR(&ramlog_ramlog_wrap_enable_obj) },
};
static MP_DEFINE_CONST_DICT(ramlog_ramlog_locals_dict, ramlog_ramlog_locals_dict_table);

MP_DEFINE_CONST_OBJ_TYPE(
    ramlog_ramlog_type,
    MP_QSTR_RamLog,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, ramlog_make_new,
    locals_dict, &ramlog_ramlog_locals_dict,
    protocol, &ramlog_ramlog_stream_p
    );
