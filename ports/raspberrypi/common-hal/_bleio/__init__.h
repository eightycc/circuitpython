/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Bob Abeles
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

// #include <stdbool.h>

// Subset of BTstack interfaces required by header files. We keep this list small to avoid
// name collisions with other header files.
// #include "bluetooth.h"

// #include "py/obj.h"
// #include "py/objlist.h"
// #include "py/objtuple.h"
// #include "py/ringbuf.h"
// #include "supervisor/background_callback.h"

// For convenience, we include all of _bleio's shared bindings which in turn will
// include all of our port's hal implementation includes.
// #include "shared_bindings/_bleio/__init__.h"
// #include "shared-bindings/_bleio/UUID.h"
// #include "shared-bindings/_bleio/ScanResults.h"
// #include "shared-bindings/_bleio/Adapter.h"
// #include "shared-bindings/_bleio/Address.h"
// #include "shared-bindings/_bleio/Attribute.h"
// #include "shared-bindings/_bleio/Characteristic.h"
// #include "shared-bindings/_bleio/CharacteristicBuffer.h"
// #include "shared-bindings/_bleio/Connection.h"
// #include "shared-bindings/_bleio/Descriptor.h"
// #include "shared-bindings/_bleio/PacketBuffer.h"
// #include "shared-bindings/_bleio/ScanEntry.h"
// #include "shared-bindings/_bleio/Service.h"