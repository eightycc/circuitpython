// This file is part of the CircuitPython project: https://circuitpython.org
//
// SPDX-FileCopyrightText: Copyright (c) 2024 Bob Abeles
//
// SPDX-License-Identifier: MIT

#define MICROPY_HW_BOARD_NAME "Pimoroni Pico Plus 2 W"
#define MICROPY_HW_MCU_NAME "rp2350b"

#define CIRCUITPY_DIGITALIO_HAVE_INVALID_PULL (1)
#define CIRCUITPY_DIGITALIO_HAVE_INVALID_DRIVE_MODE (1)

#define MICROPY_HW_LED_STATUS   (&pin_CYW0)

#define CIRCUITPY_BOARD_I2C         (1)
#define CIRCUITPY_BOARD_I2C_PIN     {{.scl = &pin_GPIO5, .sda = &pin_GPIO4}}

#define CIRCUITPY_PSRAM_CHIP_SELECT (&pin_GPIO47)

// Uncomment to enable REPL and debug output over UART0 (TX/RX) pins
#define CIRCUITPY_CONSOLE_UART_RX (&pin_GPIO1)
#define CIRCUITPY_CONSOLE_UART_TX (&pin_GPIO0)
#define CIRCUITPY_CONSOLE_UART_TIMESTAMP (1)

// RAM logging configuration
#define CIRCUITPY_RAMLOG_AUTOSIZE (1024 * 1024)
#define CIRCUITPY_RAMLOG_AUTOALLOCATE (1)
#define CIRCUITPY_RAMLOG_RECOVER (1)
#define CIRCUITPY_RAMLOG_AUTOSTART (1)
