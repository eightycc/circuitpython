// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
// SPDX-FileCopyrightText: Copyright (c) 2019 Dan Halbert for Adafruit Industries
// SPDX-FileCopyrightText: Copyright (c) 2018 Artur Pacholec
// SPDX-FileCopyrightText: Copyright (c) 2016 Glenn Ruben Bakke
//
// SPDX-License-Identifier: MIT

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
