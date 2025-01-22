// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2025 Bob Abeles
//
// SPDX-License-Identifier: MIT

#include "py/obj.h"
#include "py/runtime.h"
#include "shared-bindings/ramlog/__init__.h"
#include "shared-bindings/ramlog/ramlog.h"

///| """RAM Logging
///|
///| The `ramlog` module provides access to CircuitPython's RAM logging facility. This is a
///| low-level logging service that writes messages to a circular buffer in RAM. It is useful
///| for debugging situations where logging to a file or serial UART is not practical because
///| logging I/O delays interfere with the code under test.
///|

static const mp_rom_map_elem_t ramlog_module_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ramlog) },
    { MP_ROM_QSTR(MP_QSTR_RamLog), MP_ROM_PTR(&ramlog_ramlog_type) },
};

static MP_DEFINE_CONST_DICT(ramlog_module_globals, ramlog_module_globals_table);

const mp_obj_module_t ramlog_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&ramlog_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR_ramlog, ramlog_module);
