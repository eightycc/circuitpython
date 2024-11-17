// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
//
// SPDX-License-Identifier: MIT

#define BTSTACK_FILE__ "hci_dump_mp_printf.c"

#include <stdarg.h>
#include <stddef.h>

#include "py/mpprint.h"
#include "supervisor/shared/tick.h"

#include "btstack.h"
#include "hci_dump_mp_printf.h"

// Implementation of low-level BTstack debug dump messaging for CircuitPython.

static void mp_printf_hexdump(const uint8_t *data, uint16_t size) {
    for (uint16_t i = 0; i < size; i++) {
        if (i % 16 == 0) {
            mp_printf(&mp_plat_print, "  %4x: ", i);
        } else {
            if (i % 4 == 0) {
                mp_printf(&mp_plat_print, ":");
            } else {
                mp_printf(&mp_plat_print, " ");
            }
        }

        mp_printf(&mp_plat_print, "%02x", data[i]);

        if ((i % 16 == 15) || (i == size - 1)) {
            if ((i + 1) % 16 != 0) {
                for (uint16_t j = (i + 1) % 16; j <= 15; j++) {
                    mp_printf(&mp_plat_print, "   ");
                }
            }
            mp_printf(&mp_plat_print, " : ");

            uint16_t chars = i % 16 + 1;
            for (uint16_t j = 0; j < chars; j++) {
                uint16_t idx = i - i % 16 + j;
                uint8_t c = data[idx];
                if (c < 32 || c > 126) {
                    c = '.';
                }
                mp_printf(&mp_plat_print, "%c", c);
            }
            mp_printf(&mp_plat_print, "\n");
        }
    }

    mp_printf(&mp_plat_print, "\n");
}

static void hci_dump_mp_printf_timestamp(void) {
    uint64_t timestamp = supervisor_ticks_ms64();
    mp_printf(&mp_plat_print, "%06u: ", timestamp & 0xFFFFFF);
}

static void hci_dump_mp_printf_packet(uint8_t packet_type, uint8_t in, uint8_t *packet, uint16_t len) {
    switch (packet_type) {
        case HCI_COMMAND_DATA_PACKET:
            mp_printf(&mp_plat_print, "CMD => ");
            break;
        case HCI_EVENT_PACKET:
            mp_printf(&mp_plat_print, "EVT <= ");
            break;
        case HCI_ACL_DATA_PACKET:
            #ifdef HCI_DUMP_STDOUT_MAX_SIZE_ACL
            if (len > HCI_DUMP_STDOUT_MAX_SIZE_ACL) {
                mp_printf(&mp_plat_print, "LOG -- ACL %s, size %u\n", in ? "in" : "out", len);
                len = HCI_DUMP_STDOUT_MAX_SIZE_ACL;
                break;
            }
            #endif
            if (in) {
                mp_printf(&mp_plat_print, "ACL <= ");
            } else {
                mp_printf(&mp_plat_print, "ACL => ");
            }
            break;
        case HCI_SCO_DATA_PACKET:
            #ifdef HCI_DUMP_STDOUT_MAX_SIZE_SCO
            if (len > HCI_DUMP_STDOUT_MAX_SIZE_SCO) {
                mp_printf(&mp_plat_print, "LOG -- SCO %s, size %u\n", in ? "in" : "out", len);
                len = HCI_DUMP_STDOUT_MAX_SIZE_SCO;
                break;
            }
            #endif
            if (in) {
                mp_printf(&mp_plat_print, "SCO <= ");
            } else {
                mp_printf(&mp_plat_print, "SCO => ");
            }
            break;
        case HCI_ISO_DATA_PACKET:
            #ifdef HCI_DUMP_STDOUT_MAX_SIZE_ISO
            if (len > HCI_DUMP_STDOUT_MAX_SIZE_ISO) {
                mp_printf(&mp_plat_print, "LOG -- ISO %s, size %u\n", in ? "in" : "out", len);
                len = HCI_DUMP_STDOUT_MAX_SIZE_ISO;
                break;
            }
            #endif
            if (in) {
                mp_printf(&mp_plat_print, "ISO <= ");
            } else {
                mp_printf(&mp_plat_print, "ISO => ");
            }
            break;
        case LOG_MESSAGE_PACKET:
            mp_printf(&mp_plat_print, "LOG -- %s\n", (char *)packet);
            return;
        default:
            mp_printf(&mp_plat_print, "??? <= ");
            break;
    }
    mp_printf(&mp_plat_print, "\n");
    mp_printf_hexdump(packet, len);
}

static void hci_dump_mp_printf_log_packet(uint8_t packet_type, uint8_t in, uint8_t *packet, uint16_t len) {
    hci_dump_mp_printf_timestamp();
    hci_dump_mp_printf_packet(packet_type, in, packet, len);
}

static void hci_dump_mp_printf_log_message(int log_level, const char *format, va_list argptr) {
    mp_vprintf(&mp_plat_print, format, argptr);
    mp_printf(&mp_plat_print, "\n");
}

const hci_dump_t *hci_dump_mp_printf_get_instance(void) {
    static const hci_dump_t hci_dump_mp_printf = {
        /* void (*reset)(void); */
        NULL,
        /* void (*log_packet)(uint8_t packet_type, uint8_t in, uint8_t *packet, uint16_t len); */
        &hci_dump_mp_printf_log_packet,
        /* void (*log_message)(int log_level, const char * format, va_list argptr); */
        &hci_dump_mp_printf_log_message,
    };
    return &hci_dump_mp_printf;
}
