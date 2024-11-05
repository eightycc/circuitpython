# Raspberry Pi BLE Notes

# Table of Contents
- [Raspberry Pi BLE Notes](#raspberry-pi-ble-notes)
- [Table of Contents](#table-of-contents)
  - [Raspberry Pi Pico-W Hardware](#raspberry-pi-pico-w-hardware)
    - [CYW43439 Board Connections](#cyw43439-board-connections)
  - [Raspberry Pi Port](#raspberry-pi-port)
    - [Port Configuration](#port-configuration)
  - [Raspberry Pi SDK](#raspberry-pi-sdk)
  - [CYW43 Driver](#cyw43-driver)
    - [Driver Configuration](#driver-configuration)
      - [Configuration Oddities](#configuration-oddities)
  - [BTstack](#btstack)
    - [BTstack Interface](#btstack-interface)
      - [Events](#events)
        - [Events Emitted by BTstack](#events-emitted-by-btstack)
      - [Timers](#timers)
  - [Debugging](#debugging)
    - [Bluetooth Sniffer](#bluetooth-sniffer)
    - [Pico Probe](#pico-probe)
  - [Pico SDK BTStack Interface](#pico-sdk-btstack-interface)
      - [`btstack_chipset_cyw43.[c,h]`](#btstack_chipset_cyw43ch)
      - [`btstack_cyw43.[c,h]`](#btstack_cyw43ch)
      - [`btstack_hci_transport_cyw43.[c,h]`](#btstack_hci_transport_cyw43ch)
  - [CircuitPython `_bleio` Shared Bindings](#circuitpython-_bleio-shared-bindings)
      - [`__init__.[c,h]`](#__init__ch)
      - [`Adapter.[c,h]`](#adapterch)
      - [`Address.[c,h]`](#addressch)
      - [`Attribute.[c,h]`](#attributech)
      - [`Characteristic.[c,h]`](#characteristicch)
      - [`CharacteristicBuffer.[c,h]`](#characteristicbufferch)
      - [`Connection.[c,h]`](#connectionch)
      - [`Descriptor.[c,h]`](#descriptorch)
      - [`PacketBuffer.[c,h]`](#packetbufferch)
      - [`ScanEntry.[c,h]`](#scanentrych)
      - [`ScanResults.[c,h]`](#scanresultsch)
      - [`Service.[c,h]`](#servicech)
      - [`UUID.[c,h]`](#uuidch)
  - [`_bleio` and `wifi` Shared Bindings](#_bleio-and-wifi-shared-bindings)
  - [MicroPython Bluetooth Implementation ](#micropython-bluetooth-implementation-)
    - [BTStack Events Handled](#btstack-events-handled)
      - [`modbluetooth.[c,h]`](#modbluetoothch)
      - [`modbluetooth_btstack.[c,h]`](#modbluetooth_btstackch)
  - [nRF Implementation](#nrf-implementation)
    - [Events](#events-1)
      - [`__init__.[c,h]`](#__init__ch-1)
      - [`Adapter.[c,h]`](#adapterch-1)
      - [`Attribute.[c,h]`](#attributech-1)
      - [`bonding.[c,h]`](#bondingch)
      - [`Characteristic.[c,h]`](#characteristicch-1)
      - [`CharacteristicBuffer.[c,h]`](#characteristicbufferch-1)
      - [`Connection.[c,h]`](#connectionch-1)
      - [`Descriptor.[c,h]`](#descriptorch-1)
      - [`PacketBuffer.[c,h]`](#packetbufferch-1)
      - [`Service.[c,h]`](#servicech-1)
      - [`UUID.[c,h]`](#uuidch-1)
      - [`ble_drv.[c,h]`](#ble_drvch)

## Raspberry Pi Pico-W Hardware

### CYW43439 Board Connections

| Wire | RP2040 | CYW43439 | RT6154A | Notes |
|-|--------|----------|-------|--|
| WL_GPIO0 | | GPIO0 | | replaces GPIO25<br>Green LED D2 |
| WL_GPIO1 | | GPIO1 | PS/SYNC | replaces GPIO23<br>100K pull low |
| WL_GPIO2 | | GPIO2 | | replaces GPIO24<br>VBUS / 2 |
| WL_ON | GPIO23 | WL_REG_ON<br> BT_REG_ON | | |
| WL_D | GPIO24 | SDIO_CMD<br>470R SDIO_DATA0<br>10K SDIO_DATA1<br>SDIO_DATA2 | | gSPI DI<br>gSPI DO<br>gSPI IRQ<br>gSPI NC|
| WL_CS | GPIO25 | SDIO_DATA3 | | gSPI CS|
| WL_CLK | GPIO29_ADC3 | SDIO_CLK | | gSPI SCLK<br>10K pull low |
## Raspberry Pi Port

### Port Configuration
| Option | Setting | Final Setter |
|--------|---------|-------|
| CIRCUITPY_BLEIO | 1 | `raspberry_pi_pico_w/mpconfigboard.mk` |
| CIRCUITPY_BLEIO_HCI | 0 | `raspberry_pi_pico_w/mpconfigboard.mk` |

## Raspberry Pi SDK

## CYW43 Driver

### Driver Configuration

The Raspberry Pi port configures the CYW43 driver by front-ending `sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h` with `ports/raspberrypi/sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h`.

CYW43_driver configuration is done in the following includes:

`ports/raspberrypi/lib/cyw43-driver/src/cyw43_config.h`:
```c
// Import port-specific configuration file.
#ifdef CYW43_CONFIG_FILE
#include CYW43_CONFIG_FILE
#else
#include <cyw43_configport.h>
#endif
```

`ports/raspberrypi/cyw43_configport.h`:
```c
#include "sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h"
```

`ports/raspberrypi/sdk/src/rp2_common/pico_cyw43_driver/include/cyw43_configport.h`

| Option | Setting | Final Setter |
|--------|---------|-------|
| CYW43_CONFIG_FILE | *undefined* | |
| CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE | `wb43439A0_7_95_49_00_combined.h` | `cyw43_configport.h` |
| CYW43_CLEAR_SDIO_INT | 0 | `cyw43_config.h` |
| CYW43_DEBUG(...) | `NDEBUG? (void)0 : CYW43_PRINTF(__VA_ARGS__)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_AP_ADDRESS | `LWIP_MAKEU32(192, 168, 4, 1)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_AP_GATEWAY | `LWIP_MAKEU32(192, 168, 4, 1)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_DNS | `LWIP_MAKEU32(8, 8, 8, 8)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_MASK | `LWIP_MAKEU32(255, 255, 255, 0)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_STA_ADDRESS | `LWIP_MAKEU32(0, 0, 0, 0)` | `cyw43_config.h` |
| CYW43_DEFAULT_IP_STA_GATEWAY | `LWIP_MAKEU32(192, 168, 0, 1)` | `cyw43_config.h` |
| CYW43_EVENT_POLL_HOOK | `usb_background()` | local `cyw43_configport.h` |
| CYW43_ENABLE_BLUETOOTH | 1 | local `cyw43_configport.h` |
| CYW43_FAIL_FAST_CHECK(res) | `res` | `cyw43_config.h` |
| CYW43_GPIO | 1 | `cyw43_configport.h` |
| CYW43_GPIO_IRQ_HANDLER_PRIORITY | 0x40 | `cyw43_driver.h` |
| CYW43_HAL_MAC_WLAN0 | 0 | `cyw43_configport.h` |
| CYW43_HAL_PIN_MODE_INPUT | GPIO_IN | `cyw43_configport.h` |
| CYW43_HAL_PIN_MODE_OUTPUT | GPIO_OUT | `cyw43_configport.h` |
| CYW43_HOST_NAME | `PicoW` | `cyw43_configport.h` |
| CYW43_INFO(...) | `CYW43_PRINTF(__VA_ARGS__)` | `cyw43_config.h` |
| CYW43_IOCTL_TIMEOUT_US | 100000 | `cyw43_configport.h` |
| CYW43_LOGIC_DEBUG | 0 | `cyw43_configport.h` |
| CYW43_LWIP | 1 | `cyw43_config.h` |
| CYW43_NO_NETUTILS | 1 | `cyw43_configport.h` |
| CYW43_NETUTILS | 1 | local `cyw43_configport.h` |
| CYW43_NUM_GPIOS | `CYW43_WL_GPIO_COUNT` | `cyw43_configport.h` |
| CYW43_PIN_WL_HOST_WAKE | 24 | local `cyw43_configport.h` |
| CYW43_PIN_WL_RFSW_VDD | *undefined* | |
| CYW43_PIN_WL_SDIO_1 | *undefined* | |
| CYW43_POST_POLL_HOOK | `cyw43_post_poll_hook();` | `cyw43_configport.h` |
| CYW43_PRINTF(...) | `printf(__VA_ARGS__)` | `cyw43_config.h` |
| CYW43_RESOURCE_ATTRIBUTE | `__attribute__((aligned(4)))` | `cyw43_config.h` |
| CYW43_SPI_PIO | 1 | `cyw43_configport.h` |
| CYW43_SPI_PIO_PREFERRED_PIO | 1 | `cyw43_bus_pio_spi.c` |
| CYW43_SLEEP_CHECK_MS | 50 | `cyw43_driver.c` |
| CYW43_SLEEP_MAX | 50 | `cyw43_config.h` |
| CYW43_USE_HEX_BTFW | *undefined* | |
| CYW43_USE_OTP_MAC | 1 | `cyw43_configport.h` |
| CYW43_USE_STATS | 0 | `cyw43_configport.h` |
| CYW43_USE_SPI | 1 | `cyw43_configport.h` |
| CYW43_VDEBUG(...) | `(void)0` | `cyw43_config.h` |
| CYW43_VERBOSE_DEBUG | 0 | ```cyw43_config.h``` |
| CYW43_WARN(...) | `CYW43_PRINTF("[CYW43] " __VA_ARGS__)` | `cyw43_config.h` |
| CYW43_WIFI_NVRAM_INCLUDE_FILE | `wifi_nvram_43439.h` | `cyw43_configport.h` |
| CYW43_WL_GPIO_COUNT | 3 | `mpconfigboard.mk` |
| CYW43_WL_GPIO_LED_PIN | 0 | `mpconfigboard.mk` |
| CYW43_WL_GPIO_VBUS_PIN | *undefined* | |
| CYW43_WL_HOST_WAKE | 24 | `mpconfigboard.mk` |
| CYW43_WL_REG_ON | 23 | `mpconfigboard.mk` |
| CYW43_WL_USES_VSYS_PIN | *undefined* | |

#### Configuration Oddities

A number of `CYW43_*` configuration options are defined in `ports/raspberrypi/sdk/src/boards/include/boards/pico_w.h`, but this file is not included by CP builds. The following options are affected: `CYW43_PIN_WL_HOST_WAKE`, `CYW43_PIN_WL_REG_ON`, `CYW43_WL_GPIO_COUNT`, `CYW43_WL_GPIO_LED_PIN`, `CYW43_WL_GPIO_VBUS_PIN`, `CYW43_USES_VSYS_PIN`.

Some of these options are defined in `ports/raspberrypi/boards/raspberry_pi_pico_w/mpconfigboard.mk` and in other board's `mpconfigboard.mk` files. Maybe not ideal.

Some options are passed into the build via `CFLAGS_CYW43` in `ports/raspberrypi/Makefile`. Again, maybe not ideal. These options are:
| Option | Setting |
|--------|---------|
|CYW43_ENABLE_BLUETOOTH|1|
|CYW43_LOGIC_DEBUG|0|
|CYW43_LWIP|1|
|CYW43_USE_SPI|*defined*|
|CYW43_USE_STATS|0|
|IGNORE_GPIO25|*defined*|
|IGNORE_GPIO23|*defined*|
|IGNORE_GPIO24|*defined*|
|PICO_BUILD|*defined*|
|PICO_CYW43_ARCH_POLL|0|
|PICO_CYW43_ARCH_THREADSAFE_BACKGROUND|1|

## BTstack

### BTstack Interface

The Raspberry Pi Pico SDK interfaces with BTstack using the CYW43 driver.

#### Events

BTstack notifies _bleio of events by issuing callbacks to registered event handlers. Internally, BTstack invokes all registered event handlers for any event. This results in overhead growing arithmetically as event handlers are added. To avoid this additional overhead, _bleio registers a single event handler that categorizes all events and invokes only the appropriate secondary event handlers.

##### Events Emitted by BTstack

| Event | Description | Emitter |
|-------|-------------|---------|
|GAP_EVENT_PAIRING_STARTED|Pairing started.|hci_pairing_started|
|GAP_EVENT_PAIRING_COMPLETE|Pairing complete.|hci_pairing_complete|
|GAP_EVENT_ADVERTISING_REPORT|Advertising report.|le_handle_advertisement_report|
|GAP_EVENT_ADVERTISING_REPORT|Advertising report.|le_handle_extended_advertisement_report|
|GAP_EVENT_EXTENDED_ADVERTISING_REPORT|Extended advertising report.|le_handle_extended_advertisement_report|
|GAP_EVENT_RSSI_MEASUREMENT|RSSI measurement.|handle_command_complete_event|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_ADVERTISING_SET_INSTALLED|Meta GAP event.|handle_command_complete_event|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_ADVERTISING_SET_REMOVED|Meta GAP event.|handle_command_complete_event|
|GAP_EVENT_INQUIRY_COMPLETE|Inquiry complete.|handle_command_complete_event|
|GAP_EVENT_LOCAL_OOB_DATA|Local OOB data.|handle_command_complete_event|
|GAP_EVENT_INQUIRY_COMPLETE|Inquiry complete.|handle_command_complete_event|
|Packet in event|Notify upper stack|handle_command_complete_event|
|GAP_EVENT_INQUIRY_RESULT|Inquiry result.|gap_inquiry_explode|
|BTSTACK_EVENT_STATE|BTstack state.|hci_emit_state|
|HCI_EVENT_CONNECTION_COMPLETE|Connection complete.|hci_emit_connection_complete|
|L2CAP_EVENT_TIMEOUT_CHECK|Timeout check.|hci_emit_l2cap_check_timeout|
|HCI_EVENT_LE_META<br>HCI_SUBEVENT_LE_CONNECTION_COMPLETE|LE meta event.|hci_emit_le_connection_complete|
|HCI_EVENT_TRANSPORT_PACKET_SENT|Transport packet sent.|hci_emit_transport_packet_sent|
|HCI_EVENT_DISCONNETION_COMPLETE|Disconnection complete.|hci_emit_disconnection_complete|
|BTSTACK_EVENT_NR_CONNECTIONS_CHANGED|Number of connections changed.|hci_emit_nr_connections_changed|
|BTSTACK_EVENT_POWERON_FAILED|Power on failed.|hci_emit_hci_open_failed|
|GAP_EVENT_DEDICATED_BONDING_COMPLETED|Dedicated bonding completed.|hci_emit_dedicated_bonding_completed|
|GAP_EVENT_SECURITY_LEVEL|Security level.|hci_emit_security_level|
|BTSTACK_EVENT_SCAN_MODE_CHANGED|Scan mode changed.|hci_emit_scan_mode_changed|
|l2cap_trigger_run_event|Trigger run event.|gap_request_connection_parameter_update|
|GAP_EVENT_INQUIRY_COMPLETE|Inquiry complete.|gap_inquiry_stop|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_BIG_CREATED|Meta GAP event.|hci_emit_big_created|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_CIG_CREATED|Meta GAP event.|hci_emit_cig_created|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_CIS_CREATED|Meta GAP event.|hci_emit_cis_created|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_BIG_TERMINATED|Meta GAP event.|hci_emit_big_terminated|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_BIG_SYNC_CREATED|Meta GAP event.|hci_emit_big_sync_created|
|HCI_EVENT_META_GAP<br>GAP_SUBEVENT_BIG_SYNC_STOPPED|Meta GAP event.|hci_emit_big_sync_stopped|
|HCI_EVENT_BIS_CAN_SEND_NOW|BIS can send now.|hci_emit_bis_can_send_now|
|HCI_EVENT_CIS_CAN_SEND_NOW|CIS can send now.|hci_emit_cis_can_send_now|
|HCI_STATE_WORKING|HCI state.|hci_init_done|
|power state|Power state.|hci_power_control|
|HCI_STATE_HALTING|HCI halting.|hci_halting_run|
|HCI_STATE_SLEEPING|HCI sleeping.|hci_falling_asleep_run|

#### Timers

## Debugging

### Bluetooth Sniffer

### Pico Probe

Connect the pico probe to the RP2040's SWD pins and its debug UART port (GPIO0 and GPIO1). The probe and the RP2040 UUT share a common ground but are powered separately by their USB ports.

| Probe Pico | UUT Pico | Description |
|------------|----------|----------|
| GND        | GND      | GND      |
| GP2        | SWCLK    | SWCLK    |
| GP3        | SWDIO    | SWDIO    |
| GP4        | GP1      | UART0 TX |
| GP5        | GP0      | UART0 RX |

OpenOCD is a locally built version of OpenOCD, `Open On-Chip Debugger 0.11.0-g8e3c38f (2023-08-30-15:11)`.

Connect the debug UART by running:
```
minicom -D /dev/ttyACMx -b 115200
```

Connect OpenOCD by running:
```
openocd -f interface/cmsis-dap.cfg -c "adapter speed 5000" -f target/rp2040.cfg -s tcl
```

```
Open On-Chip Debugger 0.11.0-g8e3c38f (2023-08-30-15:11)
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
adapter speed: 5000 kHz

Info : auto-selecting first available session transport "swd". To override use 'transport select <transport>'.
Info : Hardware thread awareness created
Info : Hardware thread awareness created
Info : RP2040 Flash Bank Command
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : Using CMSIS-DAPv2 interface with VID:PID=0x2e8a:0x000c, serial=E6603828235F8435
Info : CMSIS-DAP: SWD  Supported
Info : CMSIS-DAP: FW Version = 2.0.0
Info : CMSIS-DAP: Interface Initialised (SWD)
Info : SWCLK/TCK = 0 SWDIO/TMS = 0 TDI = 0 TDO = 0 nTRST = 0 nRESET = 0
Info : CMSIS-DAP: Interface ready
Info : clock speed 5000 kHz
Info : SWD DPIDR 0x0bc12477
Info : SWD DLPIDR 0x00000001
Info : SWD DPIDR 0x0bc12477
Info : SWD DLPIDR 0x10000001
Info : rp2040.core0: hardware has 4 breakpoints, 2 watchpoints
Info : rp2040.core1: hardware has 4 breakpoints, 2 watchpoints
Info : starting gdb server for rp2040.core0 on 3333
Info : Listening on port 3333 for gdb connections
```

```bash
rabeles@ub2004:~/.../build-raspberry_pi_pico_w$ arm-none-eabi-gdb firmware.elf
GNU gdb (Arm GNU Toolchain 13.2.rel1 (Build arm-13.7)) 13.2.90.20231008-git
Copyright (C) 2023 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=x86_64-pc-linux-gnu --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<https://bugs.linaro.org/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from firmware.elf...
(gdb) target remote localhost:3333
Remote debugging using localhost:3333
warning: multi-threaded target stopped without sending a thread-id, using first non-exited thread
0x1005ada4 in x509_get_crt_ext (p=0x100000, end=0x0, crt=0x5, cb=0xffffffff, p_ctx=0x1) at ../../lib/mbedtls/library/x509_crt.c:912
912	    if (*p == end) {
(gdb) monitor reset init
target halted due to debug-request, current mode: Thread
xPSR: 0xf1000000 pc: 0x000000ea msp: 0x20041f00
target halted due to debug-request, current mode: Thread
xPSR: 0xf1000000 pc: 0x000000ea msp: 0x20041f00
(gdb) load
Loading section .boot2, size 0x100 lma 0x10000000
Loading section .text, size 0xba4f8 lma 0x10000100
Loading section .rodata, size 0x7a8c8 lma 0x100ba5f8
Loading section .binary_info, size 0x1c lma 0x10134ec0
Loading section .data, size 0x43ec lma 0x10134edc
Loading section .uninitialized, size 0x104 lma 0x101392c8
Loading section .itcm, size 0x2ff8 lma 0x101393cc
Loading section .dtcm_data, size 0x400 lma 0x1013c3c4
Loading section .scratch_x, size 0x4d0 lma 0x1013c7c4
Start address 0x100001e8, load size 1297556
Transfer rate: 64 KB/sec, 14744 bytes/write.
(gdb) monitor reset init
target halted due to debug-request, current mode: Thread
xPSR: 0xf1000000 pc: 0x000000ea msp: 0x20041f00
target halted due to debug-request, current mode: Thread
xPSR: 0xf1000000 pc: 0x000000ea msp: 0x20041f00
(gdb) cont
Continuing.
```

## Pico SDK BTStack Interface

#### `btstack_chipset_cyw43.[c,h]`

```c
static void chipset_set_bd_addr_command(
    bd_addr_t addr,
    uint8_t *hci_cmd_buffer);
```

```c
static const btstack_chipset_t btstack_chipset_cyw43 = {
    .name = "CYW43",
    .init = NULL,
    .next_command = NULL,
    .set_baudrate_command = NULL,
    .set_bd_addr_command = chipset_set_bd_addr_command,
};
```

```c
const btstack_chipset_t * btstack_chipset_cyw43_instance(void);
```

Returns the `btstack_chipset_cyw43` structure. Passed by `hci_transport_cyw43_open` to `hci_set_chipset` to set the HCI chipset for BTStack.

---

#### `btstack_cyw43.[c,h]`

```c
static void setup_tlv(void);
```
Sets up flash storage for TLV (Tag Length Value) persistent binding data.

```c
bool btstack_cyw43_init(
    async_context_t *context);
```

Invoked by `cyw43_arch_init()` to initialize bluetooth:
* Call `btstack_memory_init()` to initialize memory pools.
* Call `btstack_run_loop_init()` to initialize run loop.
* Call `hci_init(hci_transport_cyw43_instance(), NULL)` to initialize HCI.
```c
void btstack_cyw43_deinit(
    __unused async_context_t *context);
```
Invoked by `cyw43_arch_deinit()` to deinitialize bluetooth:
* Call `hci_power_control(HCI_POWER_OFF)` to power off bluetooth.
* Call `hci_close()` to close bluetooth HCI.
* Call `btstack_run_loop_deinit()` to deinitialize run loop.
* Call `btstack_memory_deinit()` to deinitialize memory pools.

---
#### `btstack_hci_transport_cyw43.[c,h]`

```c
static void hci_transport_data_source_process(
    btstack_data_source_t *ds,
    btstack_data_source_callback_type_t callback_type);
```

```c
static void hci_transport_cyw43_init(
    const void *transport_config);
```

```c
static int hci_transport_cyw43_open(void);
```

```c
static int hci_transport_cyw43_close(void);
```

```c
static void hci_transport_cyw43_register_packet_handler(
    void (*handler)(uint8_t packet_type, uint8_t *packet, uint16_t size));
```

```c
static int hci_transport_cyw43_can_send_now(
    uint8_t packet_type);
```

```c
static int hci_transport_cyw43_send_packet(
    uint8_t packet_type,
    uint8_t *packet,
    int size);
```

```c
static const hci_transport_t hci_transport_cyw43 = {
        /* const char * name; */                                        "CYW43",
        /* void   (*init) (const void *transport_config); */            &hci_transport_cyw43_init,
        /* int    (*open)(void); */                                     &hci_transport_cyw43_open,
        /* int    (*close)(void); */                                    &hci_transport_cyw43_close,
        /* void   (*register_packet_handler)(void (*handler)(...); */   &hci_transport_cyw43_register_packet_handler,
        /* int    (*can_send_packet_now)(uint8_t packet_type); */       &hci_transport_cyw43_can_send_now,
        /* int    (*send_packet)(...); */                               &hci_transport_cyw43_send_packet,
        /* int    (*set_baudrate)(uint32_t baudrate); */                NULL,
        /* void   (*reset_link)(void); */                               NULL,
        /* void   (*set_sco_config)(uint16_t voice_setting, int num_connections); */ NULL,
};
```

```c
const hci_transport_t *hci_transport_cyw43_instance(void);
```

```c
static void hci_transport_cyw43_process(void);
```

```c
void cyw43_bluetooth_hci_process(void);
```

## CircuitPython `_bleio` Shared Bindings
---
#### `__init__.[c,h]`

```c
const mp_obj_module_t bleio_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bleio_module_globals,
};

MP_REGISTER_MODULE(MP_QSTR__bleio, bleio_module);
```

```c
#if CIRCUITPY_BLEIO_HCI
// Make the module dictionary be in RAM, so that _bleio.adapter can be set.
// Use a local macro to define how table entries should be converted.
#define OBJ_FROM_PTR MP_OBJ_FROM_PTR
STATIC mp_map_elem_t bleio_module_globals_table[] = {
#else
#define OBJ_FROM_PTR MP_ROM_PTR
STATIC const mp_rom_map_elem_t bleio_module_globals_table[] = {
    #endif
    // Name must be the first entry so that the exception printing below is correct.
    { MP_ROM_QSTR(MP_QSTR___name__),             MP_ROM_QSTR(MP_QSTR__bleio) },
    { MP_ROM_QSTR(MP_QSTR_Adapter),              OBJ_FROM_PTR(&bleio_adapter_type) },
    { MP_ROM_QSTR(MP_QSTR_Address),              OBJ_FROM_PTR(&bleio_address_type) },
    { MP_ROM_QSTR(MP_QSTR_Attribute),            OBJ_FROM_PTR(&bleio_attribute_type) },
    { MP_ROM_QSTR(MP_QSTR_Connection),           OBJ_FROM_PTR(&bleio_connection_type) },
    { MP_ROM_QSTR(MP_QSTR_Characteristic),       OBJ_FROM_PTR(&bleio_characteristic_type) },
    { MP_ROM_QSTR(MP_QSTR_CharacteristicBuffer), OBJ_FROM_PTR(&bleio_characteristic_buffer_type) },
    { MP_ROM_QSTR(MP_QSTR_Descriptor),           OBJ_FROM_PTR(&bleio_descriptor_type) },
    { MP_ROM_QSTR(MP_QSTR_PacketBuffer),         OBJ_FROM_PTR(&bleio_packet_buffer_type) },
    { MP_ROM_QSTR(MP_QSTR_ScanEntry),            OBJ_FROM_PTR(&bleio_scanentry_type) },
    { MP_ROM_QSTR(MP_QSTR_ScanResults),          OBJ_FROM_PTR(&bleio_scanresults_type) },
    { MP_ROM_QSTR(MP_QSTR_Service),              OBJ_FROM_PTR(&bleio_service_type) },
    { MP_ROM_QSTR(MP_QSTR_UUID),                 OBJ_FROM_PTR(&bleio_uuid_type) },

    #if CIRCUITPY_BLEIO_HCI
    // For HCI, _bleio.adapter is settable, and starts as None.
    { MP_ROM_QSTR(MP_QSTR_adapter),              mp_const_none },
    { MP_ROM_QSTR(MP_QSTR_set_adapter),          (mp_obj_t)&bleio_set_adapter_obj },
    #else
    // For non-HCI _bleio.adapter is a fixed singleton, and is not settable.
    // _bleio.set_adapter will raise NotImplementedError.
    { MP_ROM_QSTR(MP_QSTR_adapter),              MP_ROM_PTR(&common_hal_bleio_adapter_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_adapter),          MP_ROM_PTR(&bleio_set_adapter_obj) },
    #endif

    // Errors
    { MP_ROM_QSTR(MP_QSTR_BluetoothError),       OBJ_FROM_PTR(&mp_type_bleio_BluetoothError) },
    { MP_ROM_QSTR(MP_QSTR_RoleError),            OBJ_FROM_PTR(&mp_type_bleio_RoleError) },
    { MP_ROM_QSTR(MP_QSTR_SecurityError),        OBJ_FROM_PTR(&mp_type_bleio_SecurityError) },

    // Initialization
    { MP_ROM_QSTR(MP_QSTR___init__),             OBJ_FROM_PTR(&bleio___init___obj) },
};
```

```c
MP_DEFINE_BLEIO_EXCEPTION(BluetoothError, Exception)

NORETURN void mp_raise_bleio_BluetoothError(
    mp_rom_error_text_t fmt, ...);
```

```c
MP_DEFINE_BLEIO_EXCEPTION(RoleError, bleio_BluetoothError)

NORETURN void mp_raise_bleio_RoleError(
    mp_rom_error_text_t msg);
```

```c
MP_DEFINE_BLEIO_EXCEPTION(SecurityError, bleio_BluetoothError)

NORETURN void mp_raise_bleio_SecurityError(
    mp_rom_error_text_t fmt, ...);
```

```c
STATIC mp_obj_t bleio___init__(void);
```

Invokes `common_hal_bleio_adapter_set_enabled` to set the adapter's enabled state to `true`.

```c
mp_obj_t bleio_set_adapter(
    mp_obj_t adapter_obj);

MP_DEFINE_CONST_FUN_OBJ_1(bleio_set_adapter_obj, bleio_set_adapter);
```
---
#### `Adapter.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_adapter_type,
    MP_QSTR_Adapter,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, bleio_adapter_make_new,
    locals_dict, &bleio_adapter_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_adapter_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_enabled), MP_ROM_PTR(&bleio_adapter_enabled_obj) },
    { MP_ROM_QSTR(MP_QSTR_address), MP_ROM_PTR(&bleio_adapter_address_obj) },
    { MP_ROM_QSTR(MP_QSTR_name),    MP_ROM_PTR(&bleio_adapter_name_obj) },

    { MP_ROM_QSTR(MP_QSTR_start_advertising), MP_ROM_PTR(&bleio_adapter_start_advertising_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_advertising),  MP_ROM_PTR(&bleio_adapter_stop_advertising_obj) },
    { MP_ROM_QSTR(MP_QSTR_advertising),   MP_ROM_PTR(&bleio_adapter_advertising_obj) },

    { MP_ROM_QSTR(MP_QSTR_start_scan), MP_ROM_PTR(&bleio_adapter_start_scan_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_scan),  MP_ROM_PTR(&bleio_adapter_stop_scan_obj) },

    { MP_ROM_QSTR(MP_QSTR_connect),    MP_ROM_PTR(&bleio_adapter_connect_obj) },

    { MP_ROM_QSTR(MP_QSTR_connected),   MP_ROM_PTR(&bleio_adapter_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR_connections), MP_ROM_PTR(&bleio_adapter_connections_obj) },

    { MP_ROM_QSTR(MP_QSTR_erase_bonding), MP_ROM_PTR(&bleio_adapter_erase_bonding_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_adapter_locals_dict, bleio_adapter_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_adapter_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bleio_adapter_get_enabled(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_enabled_obj,
    bleio_adapter_get_enabled);
```

```c
static mp_obj_t bleio_adapter_set_enabled(
    mp_obj_t self,
    mp_obj_t value);

STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_adapter_set_enabled_obj,
    bleio_adapter_set_enabled);

MP_PROPERTY_GETSET(bleio_adapter_enabled_obj,
    (mp_obj_t)&bleio_adapter_get_enabled_obj,
    (mp_obj_t)&bleio_adapter_set_enabled_obj);
```

```c
STATIC mp_obj_t bleio_adapter_get_address(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_address_obj,
    bleio_adapter_get_address);
```

```c
STATIC mp_obj_t bleio_adapter_set_address(
    mp_obj_t self,
    mp_obj_t new_address);

MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_adapter_set_address_obj,
    bleio_adapter_set_address);

MP_PROPERTY_GETSET(bleio_adapter_address_obj,
    (mp_obj_t)&bleio_adapter_get_address_obj,
    (mp_obj_t)&bleio_adapter_set_address_obj);
```

```c
STATIC mp_obj_t _bleio_adapter_get_name(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_name_obj,
    _bleio_adapter_get_name);
```

```c
STATIC mp_obj_t bleio_adapter_set_name(
    mp_obj_t self,
    mp_obj_t new_name);

MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_adapter_set_name_obj,
    bleio_adapter_set_name);

MP_PROPERTY_GETSET(bleio_adapter_name_obj,
    (mp_obj_t)&bleio_adapter_get_name_obj,
    (mp_obj_t)&bleio_adapter_set_name_obj);
```

```c
STATIC mp_obj_t bleio_adapter_start_advertising(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_adapter_start_advertising_obj,
    1,
    bleio_adapter_start_advertising);
```

```c
STATIC mp_obj_t bleio_adapter_stop_advertising(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_stop_advertising_obj,
    bleio_adapter_stop_advertising);
```

```c
STATIC mp_obj_t bleio_adapter_start_scan(
    size_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_adapter_start_scan_obj,
    1,
    bleio_adapter_start_scan);
```

```c
STATIC mp_obj_t bleio_adapter_stop_scan(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_stop_scan_obj,
    bleio_adapter_stop_scan);
```

```c
STATIC mp_obj_t bleio_adapter_get_advertising(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_advertising_obj,
    bleio_adapter_get_advertising);

MP_PROPERTY_GETTER(bleio_adapter_advertising_obj,
    (mp_obj_t)&bleio_adapter_get_advertising_obj);
```

```c
STATIC mp_obj_t bleio_adapter_get_connected(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_connected_obj,
    bleio_adapter_get_connected);

MP_PROPERTY_GETTER(bleio_adapter_connected_obj,
    (mp_obj_t)&bleio_adapter_get_connected_obj);
```

```c
STATIC mp_obj_t bleio_adapter_get_connections(
    mp_obj_t self);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_get_connections_obj,
    bleio_adapter_get_connections);

MP_PROPERTY_GETTER(bleio_adapter_connections_obj,
    (mp_obj_t)&bleio_adapter_get_connections_obj);
```

```c
STATIC mp_obj_t bleio_adapter_connect(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_adapter_connect_obj,
    1,
    bleio_adapter_connect);
```

```c
STATIC mp_obj_t bleio_adapter_erase_bonding(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_adapter_erase_bonding_obj,
    bleio_adapter_erase_bonding);
```
---
#### `Address.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_address_type,
    MP_QSTR_Address,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, bleio_address_make_new,
    print, bleio_address_print,
    locals_dict, &bleio_address_locals_dict,
    unary_op, bleio_address_unary_op,
    binary_op, bleio_address_binary_op
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_address_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_address_bytes),                 MP_ROM_PTR(&bleio_address_address_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_type),                          MP_ROM_PTR(&bleio_address_type_obj) },
    // These match the BLE_GAP_ADDR_TYPES values used by the nRF library.
    { MP_ROM_QSTR(MP_QSTR_PUBLIC),                        MP_ROM_INT(0) },
    { MP_ROM_QSTR(MP_QSTR_RANDOM_STATIC),                 MP_ROM_INT(1) },
    { MP_ROM_QSTR(MP_QSTR_RANDOM_PRIVATE_RESOLVABLE),     MP_ROM_INT(2) },
    { MP_ROM_QSTR(MP_QSTR_RANDOM_PRIVATE_NON_RESOLVABLE), MP_ROM_INT(3) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_address_locals_dict, bleio_address_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_address_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bleio_address_get_address_bytes(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_address_get_address_bytes_obj,
    bleio_address_get_address_bytes);

MP_PROPERTY_GETTER(bleio_address_address_bytes_obj,
    (mp_obj_t)&bleio_address_get_address_bytes_obj);
```

```c
STATIC mp_obj_t bleio_address_get_type(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_address_get_type_obj,
    bleio_address_get_type);

MP_PROPERTY_GETTER(bleio_address_type_obj,
    (mp_obj_t)&bleio_address_get_type_obj);
```

```c
STATIC mp_obj_t bleio_address_binary_op(
    mp_binary_op_t op,
    mp_obj_t lhs_in,
    mp_obj_t rhs_in);
```

```c
STATIC mp_obj_t bleio_address_unary_op(
    mp_unary_op_t op,
    mp_obj_t self_in);
```

```c
STATIC void bleio_address_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

#### `Attribute.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_attribute_type,
    MP_QSTR_Attribute,
    MP_TYPE_FLAG_NONE,
    locals_dict, &bleio_attribute_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_attribute_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_NO_ACCESS),              MP_ROM_INT(SECURITY_MODE_NO_ACCESS) },
    { MP_ROM_QSTR(MP_QSTR_OPEN),                   MP_ROM_INT(SECURITY_MODE_OPEN) },
    { MP_ROM_QSTR(MP_QSTR_ENCRYPT_NO_MITM),        MP_ROM_INT(SECURITY_MODE_ENC_NO_MITM) },
    { MP_ROM_QSTR(MP_QSTR_ENCRYPT_WITH_MITM),      MP_ROM_INT(SECURITY_MODE_ENC_WITH_MITM) },
    { MP_ROM_QSTR(MP_QSTR_LESC_ENCRYPT_WITH_MITM), MP_ROM_INT(SECURITY_MODE_LESC_ENC_WITH_MITM) },
    { MP_ROM_QSTR(MP_QSTR_SIGNED_NO_MITM),         MP_ROM_INT(SECURITY_MODE_SIGNED_NO_MITM) },
    { MP_ROM_QSTR(MP_QSTR_SIGNED_WITH_MITM),       MP_ROM_INT(SECURITY_MODE_SIGNED_WITH_MITM) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_attribute_locals_dict, bleio_attribute_locals_dict_table);
```

#### `Characteristic.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_characteristic_type,
    MP_QSTR_Characteristic,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    print, bleio_characteristic_print,
    locals_dict, &bleio_characteristic_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_characteristic_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_add_to_service), MP_ROM_PTR(&bleio_characteristic_add_to_service_obj) },
    { MP_ROM_QSTR(MP_QSTR_descriptors),    MP_ROM_PTR(&bleio_characteristic_descriptors_obj) },
    { MP_ROM_QSTR(MP_QSTR_properties),     MP_ROM_PTR(&bleio_characteristic_properties_obj) },
    { MP_ROM_QSTR(MP_QSTR_uuid),           MP_ROM_PTR(&bleio_characteristic_uuid_obj) },
    { MP_ROM_QSTR(MP_QSTR_value),          MP_ROM_PTR(&bleio_characteristic_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cccd),       MP_ROM_PTR(&bleio_characteristic_set_cccd_obj) },
    { MP_ROM_QSTR(MP_QSTR_BROADCAST),         MP_ROM_INT(CHAR_PROP_BROADCAST) },
    { MP_ROM_QSTR(MP_QSTR_INDICATE),          MP_ROM_INT(CHAR_PROP_INDICATE) },
    { MP_ROM_QSTR(MP_QSTR_NOTIFY),            MP_ROM_INT(CHAR_PROP_NOTIFY) },
    { MP_ROM_QSTR(MP_QSTR_READ),              MP_ROM_INT(CHAR_PROP_READ) },
    { MP_ROM_QSTR(MP_QSTR_WRITE),             MP_ROM_INT(CHAR_PROP_WRITE) },
    { MP_ROM_QSTR(MP_QSTR_WRITE_NO_RESPONSE), MP_ROM_INT(CHAR_PROP_WRITE_NO_RESPONSE) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_characteristic_locals_dict, bleio_characteristic_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_characteristic_add_to_service(
    size_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_characteristic_add_to_service_fun_obj,
    1,
    bleio_characteristic_add_to_service);

STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(
    bleio_characteristic_add_to_service_obj,
    MP_ROM_PTR(&bleio_characteristic_add_to_service_fun_obj));
```

```c
STATIC mp_obj_t bleio_characteristic_get_properties(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_properties_obj,
    bleio_characteristic_get_properties);

MP_PROPERTY_GETTER(bleio_characteristic_properties_obj,
    (mp_obj_t)&bleio_characteristic_get_properties_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_get_uuid(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_uuid_obj,
    bleio_characteristic_get_uuid);

MP_PROPERTY_GETTER(bleio_characteristic_uuid_obj,
    (mp_obj_t)&bleio_characteristic_get_uuid_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_get_value(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_value_obj,
    bleio_characteristic_get_value);
```

```c
STATIC mp_obj_t bleio_characteristic_set_value(
    mp_obj_t self_in,
    mp_obj_t value_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_characteristic_set_value_obj,
    bleio_characteristic_set_value);

MP_PROPERTY_GETSET(bleio_characteristic_value_obj,
    (mp_obj_t)&bleio_characteristic_get_value_obj,
    (mp_obj_t)&bleio_characteristic_set_value_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_get_max_length(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_max_length_obj,
    bleio_characteristic_get_max_length);

MP_PROPERTY_GETTER(bleio_characteristic_max_length_obj,
    (mp_obj_t)&bleio_characteristic_get_max_length_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_get_descriptors(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_descriptors_obj,
    bleio_characteristic_get_descriptors);

MP_PROPERTY_GETTER(bleio_characteristic_descriptors_obj,
    (mp_obj_t)&bleio_characteristic_get_descriptors_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_get_service(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_get_service_obj,
    bleio_characteristic_get_service);

MP_PROPERTY_GETTER(bleio_characteristic_service_obj,
    (mp_obj_t)&bleio_characteristic_get_service_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_set_cccd(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_characteristic_set_cccd_obj,
    1,
    bleio_characteristic_set_cccd);
```

```c
STATIC void bleio_characteristic_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

#### `CharacteristicBuffer.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_characteristic_buffer_type,
    MP_QSTR_CharacteristicBuffer,
    MP_TYPE_FLAG_ITER_IS_ITERNEXT | MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, bleio_characteristic_buffer_make_new,
    locals_dict, &bleio_characteristic_buffer_locals_dict,
    iter, mp_stream_unbuffered_iter,
    protocol, &characteristic_buffer_stream_p
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_characteristic_buffer_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit),        MP_ROM_PTR(&bleio_characteristic_buffer_deinit_obj) },

    // Standard stream methods.
    { MP_OBJ_NEW_QSTR(MP_QSTR_read),     MP_ROM_PTR(&mp_stream_read_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_readline), MP_ROM_PTR(&mp_stream_unbuffered_readline_obj)},
    { MP_OBJ_NEW_QSTR(MP_QSTR_readinto), MP_ROM_PTR(&mp_stream_readinto_obj) },
    // CharacteristicBuffer is currently read-only.
    // { MP_OBJ_NEW_QSTR(MP_QSTR_write),    MP_ROM_PTR(&mp_stream_write_obj) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_reset_input_buffer), MP_ROM_PTR(&bleio_characteristic_buffer_reset_input_buffer_obj) },
    // Properties
    { MP_ROM_QSTR(MP_QSTR_in_waiting), MP_ROM_PTR(&bleio_characteristic_buffer_in_waiting_obj) },
};

STATIC MP_DEFINE_CONST_DICT(
    bleio_characteristic_buffer_locals_dict,
    bleio_characteristic_buffer_locals_dict_table);
```

```c
STATIC const mp_stream_p_t characteristic_buffer_stream_p = {
    MP_PROTO_IMPLEMENT(MP_QSTR_protocol_stream)
    .read = bleio_characteristic_buffer_read,
    .write = bleio_characteristic_buffer_write,
    .ioctl = bleio_characteristic_buffer_ioctl,
    .is_text = false,
    // Disallow readinto() size parameter.
    .pyserial_readinto_compatibility = true,
};
```

```c
STATIC void raise_error_if_not_connected(
    bleio_characteristic_buffer_obj_t *self);
```

```c
STATIC mp_obj_t bleio_characteristic_buffer_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC void check_for_deinit(
    bleio_characteristic_buffer_obj_t *self);
```

```c
STATIC mp_uint_t bleio_characteristic_buffer_read(
    mp_obj_t self_in,
    void *buf_in,
    mp_uint_t size,
    int *errcode);
```

```c
STATIC mp_uint_t bleio_characteristic_buffer_write(
    mp_obj_t self_in,
    const void *buf_in,
    mp_uint_t size,
    int *errcode);
```

```c
STATIC mp_uint_t bleio_characteristic_buffer_ioctl(
    mp_obj_t self_in,
    mp_uint_t request,
    mp_uint_t arg,
    int *errcode);
```

```c
STATIC mp_obj_t bleio_characteristic_buffer_obj_get_in_waiting(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_buffer_get_in_waiting_obj,
    bleio_characteristic_buffer_obj_get_in_waiting);

MP_PROPERTY_GETTER(bleio_characteristic_buffer_in_waiting_obj,
    (mp_obj_t)&bleio_characteristic_buffer_get_in_waiting_obj);
```

```c
STATIC mp_obj_t bleio_characteristic_buffer_obj_reset_input_buffer(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_buffer_reset_input_buffer_obj,
    bleio_characteristic_buffer_obj_reset_input_buffer);
```

```c
STATIC mp_obj_t bleio_characteristic_buffer_deinit(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_characteristic_buffer_deinit_obj,
    bleio_characteristic_buffer_deinit);
```

#### `Connection.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_connection_type,
    MP_QSTR_Connection,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    locals_dict, &bleio_connection_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_connection_locals_dict_table[] = {
    // Methods
    { MP_ROM_QSTR(MP_QSTR_pair),                     MP_ROM_PTR(&bleio_connection_pair_obj) },
    { MP_ROM_QSTR(MP_QSTR_disconnect),               MP_ROM_PTR(&bleio_connection_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_discover_remote_services), MP_ROM_PTR(&bleio_connection_discover_remote_services_obj) },

    // Properties
    { MP_ROM_QSTR(MP_QSTR_connected),           MP_ROM_PTR(&bleio_connection_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR_paired),              MP_ROM_PTR(&bleio_connection_paired_obj) },
    { MP_ROM_QSTR(MP_QSTR_connection_interval), MP_ROM_PTR(&bleio_connection_connection_interval_obj) },
    { MP_ROM_QSTR(MP_QSTR_max_packet_length),   MP_ROM_PTR(&bleio_connection_max_packet_length_obj) },
};
```

```c
void bleio_connection_ensure_connected(
    bleio_connection_obj_t *self);
```

```c
STATIC mp_obj_t bleio_connection_disconnect(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_connection_disconnect_obj,
    bleio_connection_disconnect);
```

```c
STATIC mp_obj_t bleio_connection_pair(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_connection_pair_obj,
    1,
    bleio_connection_pair);
```

```c
STATIC mp_obj_t bleio_connection_discover_remote_services(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_connection_discover_remote_services_obj,
    1,
    bleio_connection_discover_remote_services);
```

```c
STATIC mp_obj_t bleio_connection_get_connected(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_connection_get_connected_obj,
    bleio_connection_get_connected);

MP_PROPERTY_GETTER(bleio_connection_connected_obj,
    (mp_obj_t)&bleio_connection_get_connected_obj);
```

```c
STATIC mp_obj_t bleio_connection_get_paired(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_connection_get_paired_obj,
    bleio_connection_get_paired);

MP_PROPERTY_GETTER(bleio_connection_paired_obj,
    (mp_obj_t)&bleio_connection_get_paired_obj);
```

```c
STATIC mp_obj_t bleio_connection_get_connection_interval(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_connection_get_connection_interval_obj,
    bleio_connection_get_connection_interval);
```

```c
STATIC mp_obj_t bleio_connection_set_connection_interval(
    mp_obj_t self_in,
    mp_obj_t interval_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_connection_set_connection_interval_obj,
    bleio_connection_set_connection_interval);

MP_PROPERTY_GETSET(bleio_connection_connection_interval_obj,
    (mp_obj_t)&bleio_connection_get_connection_interval_obj,
    (mp_obj_t)&bleio_connection_set_connection_interval_obj);

MP_PROPERTY_GETTER(bleio_connection_max_packet_length_obj,
    (mp_obj_t)&bleio_connection_get_max_packet_length_obj);
```

```c
STATIC mp_obj_t bleio_connection_get_max_packet_length(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_connection_get_max_packet_length_obj,
    bleio_connection_get_max_packet_length);
```

#### `Descriptor.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_descriptor_type,
    MP_QSTR_Descriptor,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    print, bleio_descriptor_print,
    locals_dict, &bleio_descriptor_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_descriptor_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_add_to_characteristic), MP_ROM_PTR(&bleio_descriptor_add_to_characteristic_obj) },
    { MP_ROM_QSTR(MP_QSTR_uuid), MP_ROM_PTR(&bleio_descriptor_uuid_obj) },
    { MP_ROM_QSTR(MP_QSTR_characteristic), MP_ROM_PTR(&bleio_descriptor_characteristic_obj) },
    { MP_ROM_QSTR(MP_QSTR_value), MP_ROM_PTR(&bleio_descriptor_value_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_descriptor_locals_dict, bleio_descriptor_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_descriptor_add_to_characteristic(
    size_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_descriptor_add_to_characteristic_fun_obj,
    1,
    bleio_descriptor_add_to_characteristic);
STATIC MP_DEFINE_CONST_CLASSMETHOD_OBJ(
    bleio_descriptor_add_to_characteristic_obj,
    MP_ROM_PTR(&bleio_descriptor_add_to_characteristic_fun_obj));
```

```c
STATIC mp_obj_t bleio_descriptor_get_uuid(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_descriptor_get_uuid_obj,
    bleio_descriptor_get_uuid);

MP_PROPERTY_GETTER(bleio_descriptor_uuid_obj,
    (mp_obj_t)&bleio_descriptor_get_uuid_obj);
```

```c
STATIC mp_obj_t bleio_descriptor_get_characteristic(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_descriptor_get_characteristic_obj,
    bleio_descriptor_get_characteristic);

MP_PROPERTY_GETTER(bleio_descriptor_characteristic_obj,
    (mp_obj_t)&bleio_descriptor_get_characteristic_obj);
```

```c
STATIC mp_obj_t bleio_descriptor_get_value(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_descriptor_get_value_obj,
    bleio_descriptor_get_value);
```

```c
STATIC mp_obj_t bleio_descriptor_set_value(
    mp_obj_t self_in,
    mp_obj_t value_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_descriptor_set_value_obj,
    bleio_descriptor_set_value);

MP_PROPERTY_GETSET(bleio_descriptor_value_obj,
    (mp_obj_t)&bleio_descriptor_get_value_obj,
    (mp_obj_t)&bleio_descriptor_set_value_obj);
```

```c
STATIC void bleio_descriptor_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

#### `PacketBuffer.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_packet_buffer_type,
    MP_QSTR_PacketBuffer,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, bleio_packet_buffer_make_new,
    locals_dict, &bleio_packet_buffer_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_packet_buffer_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_deinit),                     MP_ROM_PTR(&bleio_packet_buffer_deinit_obj) },

    // Standard stream methods.
    { MP_OBJ_NEW_QSTR(MP_QSTR_readinto),               MP_ROM_PTR(&bleio_packet_buffer_readinto_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_write),                  MP_ROM_PTR(&bleio_packet_buffer_write_obj) },

    { MP_OBJ_NEW_QSTR(MP_QSTR_incoming_packet_length), MP_ROM_PTR(&bleio_packet_buffer_incoming_packet_length_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_outgoing_packet_length), MP_ROM_PTR(&bleio_packet_buffer_outgoing_packet_length_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_packet_buffer_locals_dict, bleio_packet_buffer_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_packet_buffer_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC void check_for_deinit(
    bleio_packet_buffer_obj_t *self);
```

```c
STATIC mp_obj_t bleio_packet_buffer_readinto(
    mp_obj_t self_in,
    mp_obj_t buffer_obj);

STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bleio_packet_buffer_readinto_obj,
    bleio_packet_buffer_readinto);
```

```c
STATIC mp_obj_t bleio_packet_buffer_write(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_packet_buffer_write_obj,
    1,
    bleio_packet_buffer_write);
```

```c
STATIC mp_obj_t bleio_packet_buffer_deinit(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_packet_buffer_deinit_obj,
    bleio_packet_buffer_deinit);
```

```c
STATIC mp_obj_t bleio_packet_buffer_get_incoming_packet_length(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_packet_buffer_get_incoming_packet_length_obj,
    bleio_packet_buffer_get_incoming_packet_length);

MP_PROPERTY_GETTER(bleio_packet_buffer_incoming_packet_length_obj,
    (mp_obj_t)&bleio_packet_buffer_get_incoming_packet_length_obj);
```

```c
STATIC mp_obj_t bleio_packet_buffer_get_outgoing_packet_length(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_packet_buffer_get_outgoing_packet_length_obj,
    bleio_packet_buffer_get_outgoing_packet_length);

MP_PROPERTY_GETTER(bleio_packet_buffer_outgoing_packet_length_obj,
    (mp_obj_t)&bleio_packet_buffer_get_outgoing_packet_length_obj);
```

#### `ScanEntry.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_scanentry_type,
    MP_QSTR_ScanEntry,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    locals_dict, &bleio_scanentry_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_scanentry_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_address),             MP_ROM_PTR(&bleio_scanentry_address_obj) },
    { MP_ROM_QSTR(MP_QSTR_advertisement_bytes), MP_ROM_PTR(&bleio_scanentry_advertisement_bytes_obj) },
    { MP_ROM_QSTR(MP_QSTR_rssi),                MP_ROM_PTR(&bleio_scanentry_rssi_obj) },
    { MP_ROM_QSTR(MP_QSTR_connectable),         MP_ROM_PTR(&bleio_scanentry_connectable_obj) },
    { MP_ROM_QSTR(MP_QSTR_scan_response),       MP_ROM_PTR(&bleio_scanentry_scan_response_obj) },
    { MP_ROM_QSTR(MP_QSTR_matches),             MP_ROM_PTR(&bleio_scanentry_matches_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_scanentry_locals_dict, bleio_scanentry_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_scanentry_matches(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(
    bleio_scanentry_matches_obj,
    1,
    bleio_scanentry_matches);
```

```c
STATIC mp_obj_t bleio_scanentry_get_address(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_scanentry_get_address_obj,
    bleio_scanentry_get_address);

MP_PROPERTY_GETTER(bleio_scanentry_address_obj,
    (mp_obj_t)&bleio_scanentry_get_address_obj);
```

```c
STATIC mp_obj_t scanentry_get_advertisement_bytes(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_scanentry_get_advertisement_bytes_obj,
    scanentry_get_advertisement_bytes);

MP_PROPERTY_GETTER(bleio_scanentry_advertisement_bytes_obj,
    (mp_obj_t)&bleio_scanentry_get_advertisement_bytes_obj);
```

```c
STATIC mp_obj_t scanentry_get_rssi(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_scanentry_get_rssi_obj,
    scanentry_get_rssi);

MP_PROPERTY_GETTER(bleio_scanentry_rssi_obj,
    (mp_obj_t)&bleio_scanentry_get_rssi_obj);
```

```c
STATIC mp_obj_t scanentry_get_connectable(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_scanentry_get_connectable_obj,
    scanentry_get_connectable);

MP_PROPERTY_GETTER(bleio_scanentry_connectable_obj,
    (mp_obj_t)&bleio_scanentry_get_connectable_obj);
```

```c
STATIC mp_obj_t scanentry_get_scan_response(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(
    bleio_scanentry_get_scan_response_obj,
    scanentry_get_scan_response);

MP_PROPERTY_GETTER(bleio_scanentry_scan_response_obj,
    (mp_obj_t)&bleio_scanentry_get_scan_response_obj);
```

#### `ScanResults.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_scanresults_type,
    MP_QSTR_ScanResults,
    MP_TYPE_FLAG_ITER_IS_ITERNEXT,
    iter, scanresults_iternext
    );
```

```c
STATIC mp_obj_t scanresults_iternext(
    mp_obj_t self_in);
```

#### `Service.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_service_type,
    MP_QSTR_Service,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    make_new, bleio_service_make_new,
    print, bleio_service_print,
    locals_dict, &bleio_service_locals_dict
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_service_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_characteristics),   MP_ROM_PTR(&bleio_service_characteristics_obj) },
    { MP_ROM_QSTR(MP_QSTR_secondary),         MP_ROM_PTR(&bleio_service_secondary_obj) },
    { MP_ROM_QSTR(MP_QSTR_uuid),              MP_ROM_PTR(&bleio_service_uuid_obj) },
    { MP_ROM_QSTR(MP_QSTR_remote),            MP_ROM_PTR(&bleio_service_remote_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_service_locals_dict, bleio_service_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_service_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bleio_service_get_characteristics(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(bleio_service_get_characteristics_obj, bleio_service_get_characteristics);

MP_PROPERTY_GETTER(bleio_service_characteristics_obj,
    (mp_obj_t)&bleio_service_get_characteristics_obj);
```

```c
STATIC mp_obj_t bleio_service_get_remote(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(bleio_service_get_remote_obj, bleio_service_get_remote);

MP_PROPERTY_GETTER(bleio_service_remote_obj,
    (mp_obj_t)&bleio_service_get_remote_obj);
```

```c
STATIC mp_obj_t bleio_service_get_secondary(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(bleio_service_get_secondary_obj, bleio_service_get_secondary);

MP_PROPERTY_GETTER(bleio_service_secondary_obj,
    (mp_obj_t)&bleio_service_get_secondary_obj);
```

```c
STATIC mp_obj_t bleio_service_get_uuid(
    mp_obj_t self_in);

STATIC MP_DEFINE_CONST_FUN_OBJ_1(bleio_service_get_uuid_obj, bleio_service_get_uuid);

MP_PROPERTY_GETTER(bleio_service_uuid_obj,
    (mp_obj_t)&bleio_service_get_uuid_obj);
```

```c
STATIC void bleio_service_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

#### `UUID.[c,h]`

```c
MP_DEFINE_CONST_OBJ_TYPE(
    bleio_uuid_type,
    MP_QSTR_UUID,
    MP_TYPE_FLAG_HAS_SPECIAL_ACCESSORS,
    print, bleio_uuid_print,
    make_new, bleio_uuid_make_new,
    locals_dict, &bleio_uuid_locals_dict,
    unary_op, bleio_uuid_unary_op,
    binary_op, bleio_uuid_binary_op
    );
```

```c
STATIC const mp_rom_map_elem_t bleio_uuid_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_uuid16), MP_ROM_PTR(&bleio_uuid_uuid16_obj) },
    { MP_ROM_QSTR(MP_QSTR_uuid128), MP_ROM_PTR(&bleio_uuid_uuid128_obj) },
    { MP_ROM_QSTR(MP_QSTR_size), MP_ROM_PTR(&bleio_uuid_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_pack_into), MP_ROM_PTR(&bleio_uuid_pack_into_obj) },
};

STATIC MP_DEFINE_CONST_DICT(bleio_uuid_locals_dict, bleio_uuid_locals_dict_table);
```

```c
STATIC mp_obj_t bleio_uuid_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bleio_uuid_get_uuid16(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(bleio_uuid_get_uuid16_obj, bleio_uuid_get_uuid16);

MP_PROPERTY_GETTER(bleio_uuid_uuid16_obj,
    (mp_obj_t)&bleio_uuid_get_uuid16_obj);
```

```c
STATIC mp_obj_t bleio_uuid_get_uuid128(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(bleio_uuid_get_uuid128_obj, bleio_uuid_get_uuid128);

MP_PROPERTY_GETTER(bleio_uuid_uuid128_obj,
    (mp_obj_t)&bleio_uuid_get_uuid128_obj);
```

```c
STATIC mp_obj_t bleio_uuid_get_size(
    mp_obj_t self_in);

MP_DEFINE_CONST_FUN_OBJ_1(bleio_uuid_get_size_obj, bleio_uuid_get_size);

MP_PROPERTY_GETTER(bleio_uuid_size_obj,
    (mp_obj_t)&bleio_uuid_get_size_obj);
```

```c
STATIC mp_obj_t bleio_uuid_pack_into(
    mp_uint_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bleio_uuid_pack_into_obj, 1, bleio_uuid_pack_into);
```

```c
STATIC mp_obj_t bleio_uuid_unary_op(
    mp_unary_op_t op,
    mp_obj_t self_in);
```

```c
STATIC mp_obj_t bleio_uuid_binary_op(
    mp_binary_op_t op,
    mp_obj_t lhs_in,
    mp_obj_t rhs_in);
```

```c
void bleio_uuid_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

## `_bleio` and `wifi` Shared Bindings



## MicroPython Bluetooth Implementation <a name="mp_ble"></a>

---

### BTStack Events Handled

|Type|Usage|Notes|
|----|-----|-----|
| `HCI_EVENT_PACKET` | `btstack_packet_handler_att_server` | filter incoming packet type |
| `ATT_EVENT_CONNECTED` | `btstack_packet_handler_att_server` | |
| `ATT_EVENT_HANDLE_VALUE_INDICATION_COMPLETE` | `btstack_packet_handler_att_server` | |
| `ATT_EVENT_MTU_EXCHANGE_COMPLETE` | `btstack_packet_handler_att_server` | |
| `HCI_EVENT_LE_META` | `btstack_packet_handler_att_server` | |
| `HCI_EVENT_DISCONNECTION_COMPLETE` | `btstack_packet_handler_att_server` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_generic` | filter incoming packet type |
| `HCI_EVENT_LE_META`<br>`HCI_SUBEVENT_LE_CONNECTION_COMPLETE`| `btstack_packet_handler_generic` | |
| `HCI_EVENT_LE_META`<br>`HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE` | `btstack_packet_handler_generic` | |
| `BTSTACK_EVENT_STATE`<br>`HCI_STATE_WORKING` | `btstack_packet_handler_generic` | |
| `BTSTACK_EVENT_STATE`<br>`HCI_STATE_HALTING` | `btstack_packet_handler_generic` | |
| `BTSTACK_EVENT_STATE`<br>`HCI_STATE_OFF` | `btstack_packet_handler_generic` | |
| `BTSTACK_EVENT_POWERON_FAILED` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_TRANSPORT_PACKET_SENT` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_COMMAND_COMPLETE` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_COMMAND_STATUS` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_NUMBER_OF_COMPLETED_PACKETS` | `btstack_packet_handler_generic` | |
| `BTSTACK_EVENT_NR_CONNECTIONS_CHANGED` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_VENDOR_SPECIFIC` | `btstack_packet_handler_generic` | |
| `SM_EVENT_AUTHORIZATION_RESULT` | `btstack_packet_handler_generic` | |
| `SM_EVENT_PAIRING_COMPLETE` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_ENCRYPTION_CHANGE` | `btstack_packet_handler_generic` | |
| `HCI_EVENT_DISCONNECTION_COMPLETE` | `btstack_packet_handler_generic` | |
| `GAP_EVENT_ADVERTISING_REPORT` | `btstack_packet_handler_generic` | |
| `GATT_EVENT_MTU` | `btstack_packet_handler_generic` | |
| `GATT_EVENT_NOTIFICATION` | `btstack_packet_handler_generic` | |
| `GATT_EVENT_INDICATION` | `btstack_packet_handler_generic` | |
| `GATT_EVENT_CAN_WRITE_WITHOUT_RESPONSE` | `btstack_packet_handler_generic` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_discover_services` | filter incoming packet type |
| `GATT_EVENT_SERVICE_QUERY_RESULT` | `btstack_packet_handler_discover_services` | |
| `GATT_EVENT_QUERY_COMPLETE` | `btstack_packet_handler_discover_services` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_discover_characteristics` | filter incoming packet type |
| `GATT_EVENT_CHARACTERISTIC_QUERY_RESULT` | `btstack_packet_handler_discover_characteristics` | |
| `GATT_EVENT_QUERY_COMPLETE` | `btstack_packet_handler_discover_characteristics` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_discover_descriptors` | filter incoming packet type |
| `GATT_EVENT_ALL_CHARACTERISTIC_DESCRIPTORS_QUERY_RESULT` | `btstack_packet_handler_discover_descriptors` | |
| `GATT_EVENT_QUERY_COMPLETE` | `btstack_packet_handler_discover_descriptors` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_read` | filter incoming packet type |
| `GATT_EVENT_QUERY_COMPLETE` | `btstack_packet_handler_read` | |
| `GATT_EVENT_CHARACTERISTIC_VALUE_QUERY_RESULT` | `btstack_packet_handler_read` | |
||||
| `HCI_EVENT_PACKET` | `btstack_packet_handler_write_with_response` | filter incoming packet type |
| `GATT_EVENT_QUERY_COMPLETE` | `btstack_packet_handler_write_with_response` | |

#### `modbluetooth.[c,h]`

```c
typedef struct {
    mp_obj_base_t base;
    uint8_t type;
    uint8_t data[16];
} mp_obj_bluetooth_uuid_t;

extern const mp_obj_type_t mp_type_bluetooth_uuid;
```

```c
STATIC mp_obj_t bluetooth_uuid_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bluetooth_uuid_unary_op(
    mp_unary_op_t op,
    mp_obj_t self_in);
```

```c
STATIC mp_obj_t bluetooth_uuid_binary_op(
    mp_binary_op_t op,
    mp_obj_t lhs_in,
    mp_obj_t rhs_in);
```

```c
STATIC void bluetooth_uuid_print(
    const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind);
```

```c
STATIC mp_int_t bluetooth_uuid_get_buffer(
    mp_obj_t self_in,
    mp_buffer_info_t *bufinfo,
    mp_uint_t flags);
```

```c
STATIC void ringbuf_put_uuid(
    ringbuf_t *ringbuf,
    mp_obj_bluetooth_uuid_t *uuid);
```

```c
STATIC void ringbuf_get_uuid(
    ringbuf_t *ringbuf,
    mp_obj_bluetooth_uuid_t *uuid);
```

```c
MP_DEFINE_CONST_OBJ_TYPE(
    mp_type_bluetooth_uuid,
    MP_QSTR_UUID,
    MP_TYPE_FLAG_NONE,
    make_new, bluetooth_uuid_make_new,
    unary_op, bluetooth_uuid_unary_op,
    binary_op, bluetooth_uuid_binary_op,
    print, bluetooth_uuid_print,
    buffer, bluetooth_uuid_get_buffer
    );
```

```c
STATIC mp_obj_t bluetooth_ble_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *all_args);
```

```c
STATIC mp_obj_t bluetooth_ble_active(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(bluetooth_ble_active_obj, 1, 2, bluetooth_ble_active);
```

```c
STATIC mp_obj_t bluetooth_ble_config(
    size_t n_args,
    const mp_obj_t *args,
    mp_map_t *kwargs);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bluetooth_ble_config_obj, 1, bluetooth_ble_config);
```

```c
STATIC mp_obj_t bluetooth_ble_irq(
    mp_obj_t self_in,
    mp_obj_t handler_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(bluetooth_ble_irq_obj, bluetooth_ble_irq);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_advertise(
    size_t n_args,
    const mp_obj_t *pos_args,
    mp_map_t *kw_args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(bluetooth_ble_gap_advertise_obj, 1, bluetooth_ble_gap_advertise);
```

```c
STATIC int bluetooth_gatts_register_service(
    mp_obj_t uuid_in,
    mp_obj_t characteristics_in,
    uint16_t **handles,
    size_t *num_handles);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_register_services(
    mp_obj_t self_in,
    mp_obj_t services_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bluetooth_ble_gatts_register_services_obj,
    bluetooth_ble_gatts_register_services);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_connect(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gap_connect_obj,
    2,
    6,
    bluetooth_ble_gap_connect);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_scan(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gap_scan_obj,
    1,
    5,
    bluetooth_ble_gap_scan);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_disconnect(
    mp_obj_t self_in,
    mp_obj_t conn_handle_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bluetooth_ble_gap_disconnect_obj,
    bluetooth_ble_gap_disconnect);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_pair(
    mp_obj_t self_in,
    mp_obj_t conn_handle_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bluetooth_ble_gap_pair_obj,
    bluetooth_ble_gap_pair);
```

```c
STATIC mp_obj_t bluetooth_ble_gap_passkey(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gap_passkey_obj,
    4,
    4,
    bluetooth_ble_gap_passkey);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_read(
    mp_obj_t self_in,
    mp_obj_t value_handle_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bluetooth_ble_gatts_read_obj,
    bluetooth_ble_gatts_read);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_write(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gatts_write_obj,
    3,
    4,
    bluetooth_ble_gatts_write);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_notify_indicate(
    size_t n_args,
    const mp_obj_t *args,
    int gatts_op);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_notify(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gatts_notify_obj,
    3,
    4,
    bluetooth_ble_gatts_notify);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_indicate(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gatts_indicate_obj,
    3,
    4,
    bluetooth_ble_gatts_indicate);
```

```c
STATIC mp_obj_t bluetooth_ble_gatts_set_buffer(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gatts_set_buffer_obj,
    3,
    4,
    bluetooth_ble_gatts_set_buffer);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_discover_services(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gattc_discover_services_obj,
    2,
    3,
    bluetooth_ble_gattc_discover_services);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_discover_characteristics(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gattc_discover_characteristics_obj,
    4,
    5,
    bluetooth_ble_gattc_discover_characteristics);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_discover_descriptors(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gattc_discover_descriptors_obj,
    4,
    4,
    bluetooth_ble_gattc_discover_descriptors);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_read(
    mp_obj_t self_in,
    mp_obj_t conn_handle_in,
    mp_obj_t value_handle_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_3(
    bluetooth_ble_gattc_read_obj,
    bluetooth_ble_gattc_read);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_write(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_gattc_write_obj,
    4,
    5,
    bluetooth_ble_gattc_write);
```

```c
STATIC mp_obj_t bluetooth_ble_gattc_exchange_mtu(
    mp_obj_t self_in,
    mp_obj_t conn_handle_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_2(
    bluetooth_ble_gattc_exchange_mtu_obj,
    bluetooth_ble_gattc_exchange_mtu);
```

```c
STATIC mp_obj_t bluetooth_ble_l2cap_listen(
    mp_obj_t self_in,
    mp_obj_t psm_in,
    mp_obj_t mtu_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_3(
    bluetooth_ble_l2cap_listen_obj,
    bluetooth_ble_l2cap_listen);
```

```c
STATIC mp_obj_t bluetooth_ble_l2cap_connect(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_l2cap_connect_obj,
    4,
    4,
    bluetooth_ble_l2cap_connect);
```

```c
STATIC mp_obj_t bluetooth_ble_l2cap_disconnect(
    mp_obj_t self_in,
    mp_obj_t conn_handle_in,
    mp_obj_t cid_in);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_3(
    bluetooth_ble_l2cap_disconnect_obj,
    bluetooth_ble_l2cap_disconnect);
```

```c
STATIC mp_obj_t bluetooth_ble_l2cap_send(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_l2cap_send_obj,
    4,
    4,
    bluetooth_ble_l2cap_send);
```

```c
STATIC mp_obj_t bluetooth_ble_l2cap_recvinto(
    size_t n_args,
    const mp_obj_t *args);
```

```c
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(
    bluetooth_ble_l2cap_recvinto_obj,
    4,
    4,
    bluetooth_ble_l2cap_recvinto);
```

```c
STATIC const mp_rom_map_elem_t bluetooth_ble_locals_dict_table[] = {
    // General
    { MP_ROM_QSTR(MP_QSTR_active), MP_ROM_PTR(&bluetooth_ble_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_config), MP_ROM_PTR(&bluetooth_ble_config_obj) },
    { MP_ROM_QSTR(MP_QSTR_irq), MP_ROM_PTR(&bluetooth_ble_irq_obj) },
    // GAP
    { MP_ROM_QSTR(MP_QSTR_gap_advertise), MP_ROM_PTR(&bluetooth_ble_gap_advertise_obj) },
    #if MICROPY_PY_BLUETOOTH_ENABLE_CENTRAL_MODE
    { MP_ROM_QSTR(MP_QSTR_gap_connect), MP_ROM_PTR(&bluetooth_ble_gap_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_gap_scan), MP_ROM_PTR(&bluetooth_ble_gap_scan_obj) },
    #endif
    { MP_ROM_QSTR(MP_QSTR_gap_disconnect), MP_ROM_PTR(&bluetooth_ble_gap_disconnect_obj) },
    #if MICROPY_PY_BLUETOOTH_ENABLE_PAIRING_BONDING
    { MP_ROM_QSTR(MP_QSTR_gap_pair), MP_ROM_PTR(&bluetooth_ble_gap_pair_obj) },
    { MP_ROM_QSTR(MP_QSTR_gap_passkey), MP_ROM_PTR(&bluetooth_ble_gap_passkey_obj) },
    #endif
    // GATT Server
    { MP_ROM_QSTR(MP_QSTR_gatts_register_services), MP_ROM_PTR(&bluetooth_ble_gatts_register_services_obj) },
    { MP_ROM_QSTR(MP_QSTR_gatts_read), MP_ROM_PTR(&bluetooth_ble_gatts_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_gatts_write), MP_ROM_PTR(&bluetooth_ble_gatts_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_gatts_notify), MP_ROM_PTR(&bluetooth_ble_gatts_notify_obj) },
    { MP_ROM_QSTR(MP_QSTR_gatts_indicate), MP_ROM_PTR(&bluetooth_ble_gatts_indicate_obj) },
    { MP_ROM_QSTR(MP_QSTR_gatts_set_buffer), MP_ROM_PTR(&bluetooth_ble_gatts_set_buffer_obj) },
    #if MICROPY_PY_BLUETOOTH_ENABLE_GATT_CLIENT
    // GATT Client
    { MP_ROM_QSTR(MP_QSTR_gattc_discover_services), MP_ROM_PTR(&bluetooth_ble_gattc_discover_services_obj) },
    { MP_ROM_QSTR(MP_QSTR_gattc_discover_characteristics), MP_ROM_PTR(&bluetooth_ble_gattc_discover_characteristics_obj) },
    { MP_ROM_QSTR(MP_QSTR_gattc_discover_descriptors), MP_ROM_PTR(&bluetooth_ble_gattc_discover_descriptors_obj) },
    { MP_ROM_QSTR(MP_QSTR_gattc_read), MP_ROM_PTR(&bluetooth_ble_gattc_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_gattc_write), MP_ROM_PTR(&bluetooth_ble_gattc_write_obj) },
    { MP_ROM_QSTR(MP_QSTR_gattc_exchange_mtu), MP_ROM_PTR(&bluetooth_ble_gattc_exchange_mtu_obj) },
    #endif
    #if MICROPY_PY_BLUETOOTH_ENABLE_L2CAP_CHANNELS
    { MP_ROM_QSTR(MP_QSTR_l2cap_listen), MP_ROM_PTR(&bluetooth_ble_l2cap_listen_obj) },
    { MP_ROM_QSTR(MP_QSTR_l2cap_connect), MP_ROM_PTR(&bluetooth_ble_l2cap_connect_obj) },
    { MP_ROM_QSTR(MP_QSTR_l2cap_disconnect), MP_ROM_PTR(&bluetooth_ble_l2cap_disconnect_obj) },
    { MP_ROM_QSTR(MP_QSTR_l2cap_send), MP_ROM_PTR(&bluetooth_ble_l2cap_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_l2cap_recvinto), MP_ROM_PTR(&bluetooth_ble_l2cap_recvinto_obj) },
    #endif
    #if MICROPY_PY_BLUETOOTH_ENABLE_HCI_CMD
    { MP_ROM_QSTR(MP_QSTR_hci_cmd), MP_ROM_PTR(&bluetooth_ble_hci_cmd_obj) },
    #endif
};
STATIC MP_DEFINE_CONST_DICT(bluetooth_ble_locals_dict, bluetooth_ble_locals_dict_table);

STATIC MP_DEFINE_CONST_OBJ_TYPE(
    mp_type_bluetooth_ble,
    MP_QSTR_BLE,
    MP_TYPE_FLAG_NONE,
    make_new, bluetooth_ble_make_new,
    locals_dict, &bluetooth_ble_locals_dict
    );

STATIC const mp_rom_map_elem_t mp_module_bluetooth_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bluetooth) },
    { MP_ROM_QSTR(MP_QSTR_BLE), MP_ROM_PTR(&mp_type_bluetooth_ble) },
    { MP_ROM_QSTR(MP_QSTR_UUID), MP_ROM_PTR(&mp_type_bluetooth_uuid) },

    // TODO: Deprecate these flags (recommend copying the constants from modbluetooth.h instead).
    { MP_ROM_QSTR(MP_QSTR_FLAG_READ), MP_ROM_INT(MP_BLUETOOTH_CHARACTERISTIC_FLAG_READ) },
    { MP_ROM_QSTR(MP_QSTR_FLAG_WRITE), MP_ROM_INT(MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE) },
    { MP_ROM_QSTR(MP_QSTR_FLAG_NOTIFY), MP_ROM_INT(MP_BLUETOOTH_CHARACTERISTIC_FLAG_NOTIFY) },
    { MP_ROM_QSTR(MP_QSTR_FLAG_INDICATE), MP_ROM_INT(MP_BLUETOOTH_CHARACTERISTIC_FLAG_INDICATE) },
    { MP_ROM_QSTR(MP_QSTR_FLAG_WRITE_NO_RESPONSE), MP_ROM_INT(MP_BLUETOOTH_CHARACTERISTIC_FLAG_WRITE_NO_RESPONSE) },
};

STATIC MP_DEFINE_CONST_DICT(mp_module_bluetooth_globals, mp_module_bluetooth_globals_table);

const mp_obj_module_t mp_module_bluetooth = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&mp_module_bluetooth_globals,
};

// This module should not be extensible (as it is not a CPython standard
// library nor is it necessary to override from the filesystem), however it
// has previously been known as `ubluetooth`, so by making it extensible the
// `ubluetooth` alias will continue to work.
MP_REGISTER_EXTENSIBLE_MODULE(MP_QSTR_bluetooth, mp_module_bluetooth);
```

```c
STATIC void ringbuf_extract(
    ringbuf_t *ringbuf,
    mp_obj_tuple_t *data_tuple,
    size_t n_u16,
    size_t n_u8,
    mp_obj_array_t *bytes_addr,
    size_t n_i8,
    mp_obj_bluetooth_uuid_t *uuid,
    mp_obj_array_t *bytes_data);
```

```c
STATIC mp_obj_t bluetooth_ble_invoke_irq(
    mp_obj_t none_in);
```

```c
STATIC mp_obj_t invoke_irq_handler_run(
    uint16_t event,
    const mp_int_t *numeric,
    size_t n_unsigned,
    size_t n_signed,
    const uint8_t *addr,
    const mp_obj_bluetooth_uuid_t *uuid,
    const uint8_t **data,
    uint16_t *data_len,
    size_t n_data);
```

```c
STATIC mp_obj_t invoke_irq_handler_run_protected(
    uint16_t event,
    const mp_int_t *numeric,
    size_t n_unsigned,
    size_t n_signed,
    const uint8_t *addr,
    const mp_obj_bluetooth_uuid_t *uuid,
    const uint8_t **data,
    uint16_t *data_len,
    size_t n_data);
    ```

```c
STATIC mp_obj_t invoke_irq_handler(
    uint16_t event,
    const mp_int_t *numeric,
    size_t n_unsigned,
    size_t n_signed,
    const uint8_t *addr,
    const mp_obj_bluetooth_uuid_t *uuid,
    const uint8_t **data,
    uint16_t *data_len,
    size_t n_data);
```

```c
STATIC mp_obj_t invoke_irq_handler(
    uint16_t event,
    const mp_int_t *numeric,
    size_t n_unsigned,
    size_t n_signed,
    const uint8_t *addr,
    const mp_obj_bluetooth_uuid_t *uuid,
    const uint8_t **data,
    uint16_t *data_len,
    size_t n_data);
```

```c
void mp_bluetooth_gap_on_connected_disconnected(
    uint8_t event,
    uint16_t conn_handle,
    uint8_t addr_type,
    const uint8_t *addr);
```

```c
void mp_bluetooth_gap_on_connection_update(
    uint16_t conn_handle,
    uint16_t conn_interval,
    uint16_t conn_latency,
    uint16_t supervision_timeout,
    uint16_t status);
```

```c
void mp_bluetooth_gatts_on_encryption_update(
    uint16_t conn_handle,
    bool encrypted,
    bool authenticated,
    bool bonded,
    uint8_t key_size);
```

```c
bool mp_bluetooth_gap_on_get_secret(
    uint8_t type,
    uint8_t index,
    const uint8_t *key,
    uint16_t key_len,
    const uint8_t **value,
    size_t *value_len);
```

```c
bool mp_bluetooth_gap_on_set_secret(
    uint8_t type,
    const uint8_t *key,
    size_t key_len,
    const uint8_t *value,
    size_t value_len);
```

```c
void mp_bluetooth_gap_on_passkey_action(
    uint16_t conn_handle,
    uint8_t action,
    mp_int_t passkey);
```

```c
void mp_bluetooth_gatts_on_write(
    uint16_t conn_handle,
    uint16_t value_handle);
```

```c
void mp_bluetooth_gatts_on_indicate_complete(
    uint16_t conn_handle,
    uint16_t value_handle,
    uint8_t status);
```

```c
mp_int_t mp_bluetooth_gatts_on_read_request(
    uint16_t conn_handle,
    uint16_t value_handle);
```

```c
void mp_bluetooth_gatts_on_mtu_exchanged(
    uint16_t conn_handle,
    uint16_t value);
```

```c
mp_int_t mp_bluetooth_on_l2cap_accept(
    uint16_t conn_handle,
    uint16_t cid,
    uint16_t psm,
    uint16_t our_mtu,
    uint16_t peer_mtu);
```

```c
void mp_bluetooth_on_l2cap_connect(
    uint16_t conn_handle,
    uint16_t cid,
    uint16_t psm,
    uint16_t our_mtu,
    uint16_t peer_mtu);
```

```c
void mp_bluetooth_on_l2cap_disconnect(
    uint16_t conn_handle,
    uint16_t cid,
    uint16_t psm,
    uint16_t status);
```

```c
void mp_bluetooth_on_l2cap_send_ready(
    uint16_t conn_handle,
    uint16_t cid,
    uint8_t status);
```

```c
void mp_bluetooth_on_l2cap_recv(
    uint16_t conn_handle,
    uint16_t cid);
```

```c
void mp_bluetooth_gap_on_scan_complete(void);
```

```c
void mp_bluetooth_gap_on_scan_result(
    uint8_t addr_type,
    const uint8_t *addr,
    uint8_t adv_type,
    const int8_t rssi,
    const uint8_t *data,
    uint16_t data_len);
```

```c
void mp_bluetooth_gattc_on_primary_service_result(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle,
    mp_obj_bluetooth_uuid_t *service_uuid);
```

```c
void mp_bluetooth_gattc_on_characteristic_result(
    uint16_t conn_handle,
    uint16_t value_handle,
    uint16_t end_handle,
    uint8_t properties,
    mp_obj_bluetooth_uuid_t *characteristic_uuid);
```

```c
void mp_bluetooth_gattc_on_descriptor_result(
    uint16_t conn_handle,
    uint16_t handle,
    mp_obj_bluetooth_uuid_t *descriptor_uuid);
```

```c
void mp_bluetooth_gattc_on_discover_complete(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t status);
```

```c
void mp_bluetooth_gattc_on_data_available(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t value_handle,
    const uint8_t **data,
    uint16_t *data_len,
    size_t num);
```

```c
void mp_bluetooth_gattc_on_read_write_status(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t value_handle,
    uint16_t status);
```

```c
STATIC bool enqueue_irq(
    mp_obj_bluetooth_ble_t *o,
    size_t len,
    uint8_t event);
```

```c
STATIC void schedule_ringbuf(
    mp_uint_t atomic_state);
```

```c
void mp_bluetooth_gap_on_connected_disconnected(
    uint8_t event,
    uint16_t conn_handle,
    uint8_t addr_type,
    const uint8_t *addr);
```

```c
void mp_bluetooth_gap_on_connection_update(
    uint16_t conn_handle,
    uint16_t conn_interval,
    uint16_t conn_latency,
    uint16_t supervision_timeout,
    uint16_t status);
```

```c
void mp_bluetooth_gatts_on_write(
    uint16_t conn_handle,
    uint16_t value_handle);
```

```c
void mp_bluetooth_gatts_on_indicate_complete(
    uint16_t conn_handle,
    uint16_t value_handle,
    uint8_t status);
```

```c
mp_int_t mp_bluetooth_gatts_on_read_request(
    uint16_t conn_handle,
    uint16_t value_handle);
```

```c
void mp_bluetooth_gatts_on_mtu_exchanged(
    uint16_t conn_handle,
    uint16_t value);
```

```c
void mp_bluetooth_gap_on_scan_complete(void);
```

```c
void mp_bluetooth_gap_on_scan_result(
    uint8_t addr_type,
    const uint8_t *addr,
    uint8_t adv_type,
    const int8_t rssi,
    const uint8_t *data,
    uint16_t data_len);
```

```c
void mp_bluetooth_gattc_on_primary_service_result(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle,
    mp_obj_bluetooth_uuid_t *service_uuid);
```

```c
void mp_bluetooth_gattc_on_characteristic_result(
    uint16_t conn_handle,
    uint16_t value_handle,
    uint16_t end_handle,
    uint8_t properties,
    mp_obj_bluetooth_uuid_t *characteristic_uuid);
```

```c
void mp_bluetooth_gattc_on_descriptor_result(
    uint16_t conn_handle,
    uint16_t handle,
    mp_obj_bluetooth_uuid_t *descriptor_uuid);
```

```c
void mp_bluetooth_gattc_on_discover_complete(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t status);
```

```c
void mp_bluetooth_gattc_on_data_available(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t value_handle,
    const uint8_t **data,
    uint16_t *data_len,
    size_t num);
```

```c
void mp_bluetooth_gattc_on_read_write_status(
    uint8_t event,
    uint16_t conn_handle,
    uint16_t value_handle,
    uint16_t status);
```

```c
void mp_bluetooth_gatts_db_create_entry(
    mp_gatts_db_t db,
    uint16_t handle,
    size_t len);
```

```c
mp_bluetooth_gatts_db_entry_t *mp_bluetooth_gatts_db_lookup(
    mp_gatts_db_t db,
    uint16_t handle);
```

```c
int mp_bluetooth_gatts_db_read(
    mp_gatts_db_t db,
    uint16_t handle,
    const uint8_t **value,
    size_t *value_len);
```

```c
int mp_bluetooth_gatts_db_write(
    mp_gatts_db_t db,
    uint16_t handle,
    const uint8_t *value,
    size_t value_len);
```

```c
int mp_bluetooth_gatts_db_resize(
    mp_gatts_db_t db,
    uint16_t handle,
    size_t len,
    bool append);
```

```c
MP_REGISTER_ROOT_POINTER(mp_obj_t bluetooth);
```

---

#### `modbluetooth_btstack.[c,h]`

```c
typedef struct _mp_bluetooth_btstack_root_pointers_t {
    // This stores both the advertising data and the scan response data, concatenated together.
    uint8_t *adv_data;
    // Total length of both.
    size_t adv_data_alloc;

    // Characteristic (and descriptor) value storage.
    mp_gatts_db_t gatts_db;

    #if MICROPY_PY_BLUETOOTH_ENABLE_GATT_CLIENT
    // Registration for notify/indicate events.
    gatt_client_notification_t notification;

    // Active connections (only used for GATT clients).
    btstack_linked_list_t active_connections;
    #endif
} mp_bluetooth_btstack_root_pointers_t;

enum {
    MP_BLUETOOTH_BTSTACK_STATE_OFF,
    MP_BLUETOOTH_BTSTACK_STATE_STARTING,
    MP_BLUETOOTH_BTSTACK_STATE_ACTIVE,
    MP_BLUETOOTH_BTSTACK_STATE_HALTING,
    MP_BLUETOOTH_BTSTACK_STATE_TIMEOUT,
};

extern volatile int mp_bluetooth_btstack_state;
```

```c
STATIC int btstack_error_to_errno(
    int err);
```

```c
STATIC mp_obj_bluetooth_uuid_t create_mp_uuid(uint16_t uuid16, const uint8_t *uuid128);

typedef struct _mp_btstack_active_connection_t {
    btstack_linked_item_t *next; // Must be first field to match btstack_linked_item.

    uint16_t conn_handle;

    // Read/write.
    uint16_t pending_value_handle;

    // Write only. Buffer must be retained until the operation completes.
    uint8_t *pending_write_value;
    size_t pending_write_value_len;
} mp_btstack_active_connection_t;
```

```c
STATIC mp_btstack_active_connection_t *create_active_connection(
    uint16_t conn_handle);
```

```c
STATIC mp_btstack_active_connection_t *find_active_connection(
    uint16_t conn_handle);
```

```c
STATIC void remove_active_connection(
    uint16_t conn_handle);
```

```c
STATIC void btstack_packet_handler_att_server(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC uint8_t controller_static_addr[6] = {0};
STATIC bool controller_static_addr_available = false;

STATIC const uint8_t read_static_address_command_complete_prefix[] = { 0x0e, 0x1b, 0x01, 0x09, 0xfc };
```

```c
STATIC void btstack_packet_handler_generic(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC btstack_packet_callback_registration_t hci_event_callback_registration = {
    .callback = &btstack_packet_handler_generic
};
```

```c
STATIC void btstack_packet_handler_discover_services(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC void btstack_packet_handler_discover_characteristics(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC void btstack_packet_handler_discover_descriptors(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC void btstack_packet_handler_read(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC void btstack_packet_handler_write_with_response(
    uint8_t packet_type,
    uint16_t channel,
    uint8_t *packet,
    uint16_t size);
```

```c
STATIC btstack_timer_source_t btstack_init_deinit_timeout;
```

```c
STATIC void btstack_init_deinit_timeout_handler(
    btstack_timer_source_t *ds);
```

```c
STATIC void btstack_static_address_ready(
    void *arg);
```

```c
STATIC bool set_public_address(void);
```

```c
STATIC void set_random_address(void);
```

```c
STATIC void deinit_stack(void);
```

```c
int mp_bluetooth_init(void);
```

Invoked by `bluetooth_ble_active()` to initialize the Bluetooth stack.
* Invoke `mp_bluetooth_deinit()` to cleanup as necessary.
* Invoke `btstack_memory_init()` to initialize BTStack memory.
* Initialize `bluetooth_btstack_root_pointers` to all zeroes.
* Invoke `mp_bluetooth_gatts_db_create()` to initialize the GATT server database.
* Create a GATT server database entry for our GAP device name ("MPY BTSTACK").
* Invoke `mp_bluetooth_gap_set_device_name()` to set the GAP device name.
* Invoke `mp_bluetooth_btstack_port_init()` to initialize the BTStack port.
  * Invoke `btstack_run_loop_init()` to initialize the BTStack run loop.
  * Invoke `hci_init(hci_transport_cyw43_instance(), NULL)` to initialize the HCI layer.
* Invoke `l2cap_init()` to initialize L2CAP.
* Invoke `le_device_db_init()` to initialize the LE device database.
* Invoke `sm_init()` to initialize the Security Manager.
* Set up dummy ER/IR keys to suppress BTStack warning.
* Invoke `gatt_client_init()` to initialize the GATT client.
* Invoke `gatt_client_mtu_enable_auto_negotiation(false)` to require explicit MTU negotiation.
* Invoke `hci_add_event_handler(&hci_event_callback_registration)` to register the HCI event handler.
* Invoke `btstack_run_loop_set_timer(&btstack_init_deinit_timeout, BTSTACK_INIT_DEINIT_TIMEOUT_MS)` to set the init/deinit timeout.
* Invoke `btstack_run_loop_set_timer_handler(&btstack_init_deinit_timeout, btstack_init_deinit_timeout_handler)` to set the init/deinit timeout handler.
* Invoke `btstack_run_loop_add_timer(&btstack_init_deinit_timeout)` to add the init/deinit timeout timer.
* Invoke `mp_bluetooth_port_start()`:
  * Invoke `hci_power_control(HCI_POWER_ON)` to power on the controller.
* Wait for BTStack to start.
* Invoke `btstack_run_loop_remove_timer(&btstack_init_deinit_timeout)` to remove the init/deinit timeout timer.
* Handle timeout during BTStack startup and return error.
* If there is no public address, invoke `set_random_address()` to set a random address.
* Invoke `gatt_client_listen_for_characteristic_value_updates(&notification)` to register the GATT client notification handler.
* Invoke `mp_bluetooth_gatts_register_service_begin(false)` to register the GATT server service.
* Invoke `mp_bluetooth_gatts_register_service_end()`.

```c
void mp_bluetooth_deinit(void);
```
*

```c
bool mp_bluetooth_is_active(void);
```

```c
void mp_bluetooth_get_current_address(
    uint8_t *addr_type,
    uint8_t *addr);
```

```c
void mp_bluetooth_set_address_mode(
    uint8_t addr_mode);
```

```c
void mp_bluetooth_set_bonding(
    bool enabled);
```

```c
void mp_bluetooth_set_mitm_protection(
    bool enabled);
```

```c
void mp_bluetooth_set_le_secure(
    bool enabled);
```

```c
void mp_bluetooth_set_io_capability(
    uint8_t capability);
```

```c
size_t mp_bluetooth_gap_get_device_name(
    const uint8_t **buf);
```

```c
int mp_bluetooth_gap_set_device_name(
    const uint8_t *buf,
    size_t len);
```

```c
int mp_bluetooth_gap_advertise_start(
    bool connectable,
    int32_t interval_us,
    const uint8_t *adv_data,
    size_t adv_data_len,
    const uint8_t *sr_data,
    size_t sr_data_len);
```

```c
void mp_bluetooth_gap_advertise_stop(void);
```

```c
int mp_bluetooth_gatts_register_service_begin(
    bool append);
```

```c
STATIC uint16_t att_read_callback(
    hci_con_handle_t connection_handle,
    uint16_t att_handle,
    uint16_t offset,
    uint8_t *buffer,
    uint16_t buffer_size);
```

```c
STATIC int att_write_callback(
    hci_con_handle_t connection_handle,
    uint16_t att_handle,
    uint16_t transaction_mode,
    uint16_t offset,
    uint8_t *buffer,
    uint16_t buffer_size);
```

```c
STATIC inline uint16_t get_uuid16(
    const mp_obj_bluetooth_uuid_t *uuid);
```

```c
STATIC void get_characteristic_permissions(
    uint16_t flags,
    uint16_t *read_permission,
    uint16_t *write_permission);
```

```c
int mp_bluetooth_gatts_register_service(
    mp_obj_bluetooth_uuid_t *service_uuid,
    mp_obj_bluetooth_uuid_t **characteristic_uuids,
    uint16_t *characteristic_flags,
    mp_obj_bluetooth_uuid_t **descriptor_uuids,
    uint16_t *descriptor_flags,
    uint8_t *num_descriptors,
    uint16_t *handles,
    size_t num_characteristics);
```

```c
int mp_bluetooth_gatts_register_service_end(void);
```

```c
int mp_bluetooth_gatts_read(
    uint16_t value_handle,
    const uint8_t **value,
    size_t *value_len);
```

```c
int mp_bluetooth_gatts_write(
    uint16_t value_handle,
    const uint8_t *value,
    size_t value_len,
    bool send_update);
```

```c
typedef struct {
    btstack_context_callback_registration_t btstack_registration;
    int gatts_op;
    uint16_t conn_handle;
    uint16_t value_handle;
    size_t value_len;
    uint8_t value[];
} notify_indicate_pending_op_t;
```

```c
STATIC void btstack_notify_indicate_ready_handler(
    void *context);
```

```c
int mp_bluetooth_gatts_notify_indicate(
    uint16_t conn_handle,
    uint16_t value_handle,
    int gatts_op,
    const uint8_t *value,
    size_t value_len);
```

```c
int mp_bluetooth_gatts_set_buffer(
    uint16_t value_handle,
    size_t len,
    bool append);
```

```c
int mp_bluetooth_get_preferred_mtu(void);
```

```c
int mp_bluetooth_set_preferred_mtu(
    uint16_t mtu);
```

```c
int mp_bluetooth_gap_disconnect(
    uint16_t conn_handle);
```

```c
int mp_bluetooth_gap_pair(
    uint16_t conn_handle);
```

```c
int mp_bluetooth_gap_passkey(
    uint16_t conn_handle,
    uint8_t action,
    mp_int_t passkey);
```

```c
STATIC btstack_timer_source_t scan_duration_timeout;
```

```c
STATIC void scan_duration_timeout_handler(
    btstack_timer_source_t *ds);
```

```c
int mp_bluetooth_gap_scan_start(
    int32_t duration_ms,
    int32_t interval_us,
    int32_t window_us,
    bool active_scan);
```

```c
int mp_bluetooth_gap_scan_stop(void);
```

```c
int mp_bluetooth_gap_peripheral_connect(
    uint8_t addr_type,
    const uint8_t *addr,
    int32_t duration_ms,
    int32_t min_conn_interval_us,
    int32_t max_conn_interval_us);
```

```c
int mp_bluetooth_gap_peripheral_connect_cancel(void);
```

```c
int mp_bluetooth_gattc_discover_primary_services(
    uint16_t conn_handle,
    const mp_obj_bluetooth_uuid_t *uuid);
```

```c
int mp_bluetooth_gattc_discover_characteristics(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle,
    const mp_obj_bluetooth_uuid_t *uuid);
```

```c
int mp_bluetooth_gattc_discover_descriptors(
    uint16_t conn_handle,
    uint16_t start_handle,
    uint16_t end_handle);
```

```c
int mp_bluetooth_gattc_read(
    uint16_t conn_handle,
    uint16_t value_handle);
```

```c
int mp_bluetooth_gattc_write(
    uint16_t conn_handle,
    uint16_t value_handle,
    const uint8_t *value,
    size_t value_len,
    unsigned int mode);
```

```c
int mp_bluetooth_gattc_exchange_mtu(
    uint16_t conn_handle);
```

```c
int mp_bluetooth_l2cap_listen(
    uint16_t psm,
    uint16_t mtu);
```

```c
int mp_bluetooth_l2cap_connect(
    uint16_t conn_handle,
    uint16_t psm,
    uint16_t mtu);
```

```c
int mp_bluetooth_l2cap_disconnect(
    uint16_t conn_handle,
    uint16_t cid);
```

```c
int mp_bluetooth_l2cap_send(
    uint16_t conn_handle,
    uint16_t cid,
    const uint8_t *buf,
    size_t len,
    bool *stalled);
```

```c
int mp_bluetooth_l2cap_recvinto(
    uint16_t conn_handle,
    uint16_t cid,
    uint8_t *buf,
    size_t *len);
```

```c
MP_REGISTER_ROOT_POINTER(struct _mp_bluetooth_btstack_root_pointers_t *bluetooth_btstack_root_pointers);
```

## nRF Implementation

### Events

|module|event handler|_handler_entry_t|context|
|---|---|---|---|
|`__init__.c`|`_on_gattc_read_rsp_evt`|dynamic|stack, blocking|
|`Adapter.c`|`connection_on_ble_evt`|dynamic, heap<br>`connection->handler_entry`|static<br>`bleio_connections[]`|
|`Adapter.c`|`adapter_on_ble_evt`|static<br>`adapter->connection_handler_entry`|static<br>`bleio_adapter_obj_t`|
|`Adapter.c`|`scan_on_ble_evt`|dynamic|static<br>`bleio_adapter_obj_t->scan_results`|
|`Adapter.c`|`connect_on_ble_evt`|dynamic|stack, blocking|
|`Adapter.c`|`advertising_on_ble_evt`|static<br>`adapter->advertising_handler_entry`|static<br>`bleio_adapter_obj_t`|
|`CharacteristicBuffer.c`|`characteristic_buffer_on_ble_evt`|static (bt serial) or dynamic|heap<br>`bleio_characteristic_buffer_obj_t`|
|`Connection.c`|`discovery_on_ble_evt`|dynamic|static<br>`bleio_connection_internal_t`|
|`PacketBuffer.c`|`packet_buffer_on_ble_client_evt`|static or dynamic|heap<br>`bleio_packet_buffer_obj_t`|
|`PacketBuffer.c`|`packet_buffer_on_ble_server_evt`|static or dynamic|heap<br>`bleio_packet_buffer_obj_t`|

Connections are identified by a 16-bit connection handle. The connection handle is assigned by the BLE stack when a connection is established. The connection handle is used to identify the connection in all subsequent calls to the BLE stack.

---

#### `__init__.[c,h]`

```c
typedef struct {
    ble_gap_enc_key_t own_enc;
    ble_gap_enc_key_t peer_enc;
    ble_gap_id_key_t peer_id;
} bonding_keys_t;
```

```c
void check_nrf_error(
    uint32_t err_code);
```

Checks a return code from a call into NRF code for an error and raises a CP message or exception if an one is indicated.

```c
void check_gatt_status(
    uint16_t gatt_status);
```

Checks a GATT status code for an error and raises a CP message or exception if an one is indicated.

```c
void check_sec_status(
    uint8_t sec_status);
```

Checks a security status code for an error and raises a CP message or exception if an one is indicated.

```c
void bleio_user_reset();
```

Resets all user created BLE state in preparation for the heap disappearing.
It will maintain BLE workflow and connections. The following reset steps are taken:
* Stop scanning by calling `common_hal_bleio_adapter_stop_scan`.
* Stop advertising by calling `common_hal_bleio_adapter_stop_advertising`.
* Remove driver event handler by calling `ble_drv_remove_heap_handlers`.
* Restart background tasks by calling `supervisor_bluetooth_background`. This keeps the BLE workflow and its connections alive.

```c
void bleio_reset();
```

Completely resets the BLE stack including BLE connections.

* Stops BLE workflow by calling `supervisor_stop_bluetooth`.
* Resets the BLE adapter by calling `bleio_adapter_reset`.
* Marks the BLE adapter as disabled by calling `common_hal_bleio_adapter_set_enabled`.
* Reset bonding by calling `bonding_reset`. This does not remove the bonding information from flash.
* Restarts BLE by calling `supervisor_start_bluetooth`.

```c
void common_hal_bleio_check_connected(
    uint16_t conn_handle);
```

Throws a connection error if the connection handle is not valid. Validity is determined by comparing `conn_handle` to `BLE_CONN_HANDLE_INVALID`.

```c
size_t common_hal_bleio_gatts_read(
    uint16_t handle,
    uint16_t conn_handle,
    uint8_t *buf,
    size_t len);
```

Given a GATT handle and a connection handle, return the corresponding value of the characteristic or descriptor.

Invokes `sd_ble_gatts_value_get` to get the value of the characteristic or descriptor.

```c
void common_hal_bleio_gatts_write(
    uint16_t handle,
    uint16_t conn_handle,
    mp_buffer_info_t *bufinfo);
```

Given a GATT handle and a connection handle, write the value of the characteristic or descriptor.

Invokes `sd_ble_gatts_value_set` to set the value of the characteristic or descriptor.

```c
STATIC bool _on_gattc_read_rsp_evt(
    ble_evt_t *ble_evt,
    void *param);
```

Temporary event handler for GATT characteristics read response.

```c
size_t common_hal_bleio_gattc_read(uint16_t handle,
    uint16_t conn_handle,
    uint8_t *buf,
    size_t len);
```

Given a GATT handle and a connection handle, return the corresponding value of the characteristic.
Will block until HCI responds.

```c
void common_hal_bleio_gattc_write(
    uint16_t handle,
    uint16_t conn_handle,
    mp_buffer_info_t *bufinfo,
    bool write_no_response);
```

```c
void bleio_background(void);
```
Flushes bonding info to flash by invoking `bonding_background`.

```c
void common_hal_bleio_gc_collect(void);
```

Performs garbage collection for the adapter by invoking `bleio_adapter_gc_collect`.

---
#### `Adapter.[c,h]`

```c
extern bleio_connection_internal_t bleio_connections[BLEIO_TOTAL_CONNECTION_COUNT];

typedef struct {
    mp_obj_base_t base;
    // We create buffers and copy the advertising data so it will live for as long as we need.
    uint8_t *advertising_data;
    uint8_t *scan_response_data;
    // Pointer to current data.
    const uint8_t *current_advertising_data;
    bleio_scanresults_obj_t *scan_results;
    mp_obj_t name;
    mp_obj_tuple_t *connection_objs;
    ble_drv_evt_handler_entry_t connection_handler_entry;
    ble_drv_evt_handler_entry_t advertising_handler_entry;
    background_callback_t background_callback;
    bool user_advertising;
} bleio_adapter_obj_t;
```

```c
const nvm_bytearray_obj_t common_hal_bleio_nvm_obj = {
    .base = {
        .type = &nvm_bytearray_type,
    },
    .start_address = (uint8_t *)CIRCUITPY_BLE_CONFIG_START_ADDR,
    .len = CIRCUITPY_BLE_CONFIG_SIZE,
};
```

```c
STATIC void softdevice_assert_handler(
    uint32_t id,        // ignored
    uint32_t pc,        // ignored
    uint32_t info);     // ignored
```

Invokes `reset_into_safe_mode` with `SAFE_MODE_SDK_FATAL_ERROR`. This handler is passed into `sd_softdevice_enable` by `ble_stack_enable`.

```c
STATIC uint32_t ble_stack_enable(void);
```

Invoked by `common_hal_bleio_adapter_set_enabled` to enable the BLE stack. It takes the following steps:
* Invokes `sd_softdevice_enable` to enable the BLE stack.
* Invokes `sd_nvic_EnableIRQ` to enable the BLE IRQ.
* Invokes `ble_drv_reset` to reset event handlers and reset flash operation state.
* Initializes a `ble_cfg_t` and passes it to `sd_ble_cfg_set` to configure the BLE stack:
  * GAP connection configuration `BLE_CONN_CFG_GAP`:
  * GAP role count configuration `BLE_GAP_CFG_ROLE_COUNT`:
  * GATTS connection configuration `BLE_CONN_CFG_GATTS`:
  * GATT connection configuration `BLE_CONN_CFG_GATT`:
  * GATTS service changed configuration `BLE_GATTS_CFG_SERVICE_CHANGED`:
  * GATTS attribute table size configuration `BLE_GATTS_CFG_ATTR_TAB_SIZE`:
  * Vendor specific UUID count configuration `BLE_VS_UUID_COUNT`:
* Invokes `sd_ble_enable` to enable the BLE stack.
* Sets `BLE_COMMON_OPT_CONN_EVT_EXT` to enable extended connection events.
* Passes `ble_gap_conn_params_t` to `sd_ble_gap_ppcp_set` to set the preferred connection parameters. These are: `BLE_MIN_CONN_INTERVAL`, `BLE_MAX_CONN_INTERVAL`, `BLE_SLAVE_LATENCY`, `BLE_CONN_SUP_TIMEOUT`.
* Passes `BLE_APPERANCE_UNKNOWN` to `sd_ble_gap_appearance_set` to set the appearance.

```c
STATIC bool adapter_on_ble_evt(
    ble_evt_t *ble_evt,
    void *self_in);
```

Adapter's connect/disconnect event handler. Invoked by the BLE stack when an event occurs. It is registered with the BLE stack or removed by `common_hal_bleio_adapter_set_enabled`.

* Invokes `background_callback_add_core` to queue runs of `supervisor_bluetooth_background` and `bleio_background`.

Events handled are:
* `BLE_GAP_EVT_CONNECTED`:
  * Allocate an unused `bleio_connection_internal_t`.
  * Get a pointer to the `ble_gap_evt_connected_t` from the BLE event.
  * Initialize the connection:
    * Copy the connection handle from the event.
    * Set connection object to `mp_const_none`.
    * Set pair status to `PAIR_NOT_PAIRED`.
    * Set MTU to 0.
    * Copy connection parameters from the event.
* `BLE_GAP_EVT_DISCONNECTED`

```c
STATIC void get_address(
    bleio_adapter_obj_t *self,
    ble_gap_addr_t *address);
```

```c
STATIC void bleio_adapter_reset_name(
    bleio_adapter_obj_t *self);
```

```c
static void bluetooth_adapter_background(
    void *data);
```

```c
void common_hal_bleio_adapter_set_enabled(
    bleio_adapter_obj_t *self,
    bool enabled);
```



```c
bool common_hal_bleio_adapter_get_enabled(
    bleio_adapter_obj_t *self);
```

```c
bleio_address_obj_t *common_hal_bleio_adapter_get_address(
    bleio_adapter_obj_t *self);
```

```c
bool common_hal_bleio_adapter_set_address(
    bleio_adapter_obj_t *self,
    bleio_address_obj_t *address);
```

```c
uint16_t bleio_adapter_get_name(
    char *buf,
    uint16_t len);
```

```c
mp_obj_str_t *common_hal_bleio_adapter_get_name(
    bleio_adapter_obj_t *self);
```

```c
void common_hal_bleio_adapter_set_name(
    bleio_adapter_obj_t *self,
    const char *name);
```

```c
STATIC uint32_t _update_identities(
    bool is_central);
```

```c
STATIC bool scan_on_ble_evt(
    ble_evt_t *ble_evt,
    void *scan_results_in);
```

```c
mp_obj_t common_hal_bleio_adapter_start_scan(
    bleio_adapter_obj_t *self,
    uint8_t *prefixes,
    size_t prefix_length,
    bool extended, mp_int_t buffer_size,
    mp_float_t timeout,
    mp_float_t interval,
    mp_float_t window,
    mp_int_t minimum_rssi,
    bool active);
```

```c
void common_hal_bleio_adapter_stop_scan(
    bleio_adapter_obj_t *self);
```

```c
STATIC bool connect_on_ble_evt(
    ble_evt_t *ble_evt, void *info_in);
```

```c
STATIC void _convert_address(
    const bleio_address_obj_t *address,
    ble_gap_addr_t *sd_address);
```

```c
mp_obj_t common_hal_bleio_adapter_connect(
    bleio_adapter_obj_t *self,
    bleio_address_obj_t *address,
    mp_float_t timeout);
```

```c
STATIC void check_data_fit(
    size_t data_len,
    bool connectable);
```

```c
STATIC bool advertising_on_ble_evt(
    ble_evt_t *ble_evt,
    void *self_in);
```

```c
uint32_t _common_hal_bleio_adapter_start_advertising(
    bleio_adapter_obj_t *self,
    bool connectable,
    bool anonymous,
    uint32_t timeout,
    float interval,
    const uint8_t *advertising_data,
    uint16_t advertising_data_len,
    const uint8_t *scan_response_data,
    uint16_t scan_response_data_len,
    mp_int_t tx_power,
    const bleio_address_obj_t *directed_to);
```

```c
void common_hal_bleio_adapter_start_advertising(
    bleio_adapter_obj_t *self,
    bool connectable,
    bool anonymous,
    uint32_t timeout,
    mp_float_t interval,
    mp_buffer_info_t *advertising_data_bufinfo,
    mp_buffer_info_t *scan_response_data_bufinfo,
    mp_int_t tx_power,
    const bleio_address_obj_t *directed_to);
```

```c
void common_hal_bleio_adapter_stop_advertising(
    bleio_adapter_obj_t *self);
```

```c
bool common_hal_bleio_adapter_get_advertising(
    bleio_adapter_obj_t *self);
```

```c
bool common_hal_bleio_adapter_get_connected(
    bleio_adapter_obj_t *self);
```

```c
mp_obj_t common_hal_bleio_adapter_get_connections(
    bleio_adapter_obj_t *self);
```

```c
void common_hal_bleio_adapter_erase_bonding(
    bleio_adapter_obj_t *self)
```

```c
bool common_hal_bleio_adapter_is_bonded_to_central(
    bleio_adapter_obj_t *self);
```

```c
void bleio_adapter_gc_collect(
    bleio_adapter_obj_t *adapter);
```

```c
void bleio_adapter_reset(
    bleio_adapter_obj_t *adapter);
```

---

#### `Attribute.[c,h]`

```c
void bleio_attribute_gatts_set_security_mode(
    ble_gap_conn_sec_mode_t *perm,
    bleio_attribute_security_mode_t security_mode)
```

---

#### `bonding.[c,h]`

```c
void bonding_print_block(
    bonding_block_t *block);
```

```c
void bonding_print_keys(
    bonding_keys_t *keys);
```

```c
STATIC size_t compute_block_size(
    uint16_t data_length);
```

```c
void bonding_erase_storage(void);
```

```c
STATIC bonding_block_t *next_block(
    bonding_block_t *block);
```

```c
STATIC bonding_block_t *find_existing_block(
    bool is_central,
    bonding_block_type_t type,
    uint16_t ediv);
```

```c
size_t bonding_peripheral_bond_count(void);
```

```c
STATIC bonding_block_t *find_unused_block(
    uint16_t data_length);
```

```c
STATIC void invalidate_block(
    bonding_block_t *block);
```

```c
STATIC void write_block_header(
    bonding_block_t *dest_block,
    bonding_block_t *source_block_header);
```

```c
STATIC void write_block_data(
    bonding_block_t *dest_block,
    uint8_t *data,
    uint16_t data_length);
```

```c
STATIC void write_sys_attr_block(
    bleio_connection_internal_t *connection);
```

```c
STATIC void write_keys_block(
    bleio_connection_internal_t *connection);
```

```c
void bonding_clear_keys(
    bonding_keys_t *bonding_keys);
```

```c
void bonding_reset(void);
```

```c
void bonding_background(void);
```

```c
bool bonding_load_cccd_info(
    bool is_central,
    uint16_t conn_handle,
    uint16_t ediv);
```

```c
bool bonding_load_keys(
    bool is_central,
    uint16_t ediv,
    bonding_keys_t *bonding_keys);
```

```c
size_t bonding_load_identities(
    bool is_central,
    const ble_gap_id_key_t **keys,
    size_t max_length);
```

```c
const ble_gap_enc_key_t *bonding_load_peer_encryption_key(
    bool is_central,
    const ble_gap_addr_t *peer);
```

---

#### `Characteristic.[c,h]`

```c
STATIC uint16_t characteristic_get_cccd(uint16_t cccd_handle,
    uint16_t conn_handle);
```

```c
STATIC void characteristic_gatts_notify_indicate(uint16_t handle,
    uint16_t conn_handle,
    mp_buffer_info_t *bufinfo,
    uint16_t hvx_type);
```

```c
void common_hal_bleio_characteristic_construct(
    bleio_characteristic_obj_t *self,
    bleio_service_obj_t *service,
    uint16_t handle,
    bleio_uuid_obj_t *uuid,
    bleio_characteristic_properties_t props,
    bleio_attribute_security_mode_t read_perm,
    bleio_attribute_security_mode_t write_perm,
    mp_int_t max_length,
    bool fixed_length,
    mp_buffer_info_t *initial_value_bufinfo,
    const char *user_description);
```

```c
mp_obj_tuple_t *common_hal_bleio_characteristic_get_descriptors(
    bleio_characteristic_obj_t *self);
```

```c
bleio_service_obj_t *common_hal_bleio_characteristic_get_service(
    bleio_characteristic_obj_t *self);
```

```c
size_t common_hal_bleio_characteristic_get_value(
    bleio_characteristic_obj_t *self,
    uint8_t *buf, size_t len);
```

```c
size_t common_hal_bleio_characteristic_get_max_length(
    bleio_characteristic_obj_t *self);
```

```c
void common_hal_bleio_characteristic_set_value(
    bleio_characteristic_obj_t *self,
    mp_buffer_info_t *bufinfo);
```

```c
bleio_uuid_obj_t *common_hal_bleio_characteristic_get_uuid(
    bleio_characteristic_obj_t *self);
```

```c
bleio_characteristic_properties_t common_hal_bleio_characteristic_get_properties(
    bleio_characteristic_obj_t *self)
```

```c
void common_hal_bleio_characteristic_add_descriptor(
    bleio_characteristic_obj_t *self,
    bleio_descriptor_obj_t *descriptor);
```

```c
void common_hal_bleio_characteristic_set_cccd(
    bleio_characteristic_obj_t *self,
    bool notify,
    bool indicate);
```

---

#### `CharacteristicBuffer.[c,h]`

```c
STATIC void write_to_ringbuf(
    bleio_characteristic_buffer_obj_t *self,
    uint8_t *data,
    uint16_t len);
```

```c
STATIC bool characteristic_buffer_on_ble_evt(
    ble_evt_t *ble_evt,
    void *param);
```

```c
void _common_hal_bleio_characteristic_buffer_construct(
    bleio_characteristic_buffer_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    mp_float_t timeout,
    uint8_t *buffer,
    size_t buffer_size,
    void *static_handler_entry,
    bool watch_for_interrupt_char);
```

```c
void common_hal_bleio_characteristic_buffer_construct(
    bleio_characteristic_buffer_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    mp_float_t timeout,
    size_t buffer_size);
```

```c
uint32_t common_hal_bleio_characteristic_buffer_read(
    bleio_characteristic_buffer_obj_t *self,
    uint8_t *data,
    size_t len,
    int *errcode);
```

```c
uint32_t common_hal_bleio_characteristic_buffer_rx_characters_available(
    bleio_characteristic_buffer_obj_t *self);
```

```c
void common_hal_bleio_characteristic_buffer_clear_rx_buffer(
    bleio_characteristic_buffer_obj_t *self);
```

```c
bool common_hal_bleio_characteristic_buffer_deinited(
    bleio_characteristic_buffer_obj_t *self);
```

```c
void common_hal_bleio_characteristic_buffer_deinit(
    bleio_characteristic_buffer_obj_t *self);
```

```c
bool common_hal_bleio_characteristic_buffer_connected(
    bleio_characteristic_buffer_obj_t *self);
```

---

#### `Connection.[c,h]`

```c
bool connection_on_ble_evt(
    ble_evt_t *ble_evt,
    void *self_in);
```

```c
void bleio_connection_clear(
    bleio_connection_internal_t *self);
```

```c
bool common_hal_bleio_connection_get_paired(
    bleio_connection_obj_t *self);
```

```c
bool common_hal_bleio_connection_get_connected(
    bleio_connection_obj_t *self);
```

```c
void common_hal_bleio_connection_disconnect(
    bleio_connection_internal_t *self);
```

```c
void common_hal_bleio_connection_pair(
    bleio_connection_internal_t *self,
    bool bond)
```

```c
mp_float_t common_hal_bleio_connection_get_connection_interval(
    bleio_connection_internal_t *self);
```

```c
mp_int_t common_hal_bleio_connection_get_max_packet_length(
    bleio_connection_internal_t *self);
```

```c
void common_hal_bleio_connection_set_connection_interval(
    bleio_connection_internal_t *self,
    mp_float_t new_interval);
```

```c
STATIC bool discover_next_services(
    bleio_connection_internal_t *connection,
    uint16_t start_handle,
    ble_uuid_t *service_uuid);
```

```c
STATIC bool discover_next_characteristics(
    bleio_connection_internal_t *connection,
    bleio_service_obj_t *service,
    uint16_t start_handle);
```

```c
STATIC bool discover_next_descriptors(
    bleio_connection_internal_t *connection,
    bleio_characteristic_obj_t *characteristic,
    uint16_t start_handle,
    uint16_t end_handle);
```

```c
STATIC void on_primary_srv_discovery_rsp(
    ble_gattc_evt_prim_srvc_disc_rsp_t *response,
    bleio_connection_internal_t *connection);
```

```c
STATIC void on_char_discovery_rsp(
    ble_gattc_evt_char_disc_rsp_t *response,
    bleio_connection_internal_t *connection);
```

```c
STATIC void on_desc_discovery_rsp(
    ble_gattc_evt_desc_disc_rsp_t *response,
    bleio_connection_internal_t *connection);
```

```c
STATIC bool discovery_on_ble_evt(
    ble_evt_t *ble_evt,
    mp_obj_t payload);
```

```c
STATIC void discover_remote_services(
    bleio_connection_internal_t *self,
    mp_obj_t service_uuids_whitelist);
```

```c
mp_obj_tuple_t *common_hal_bleio_connection_discover_remote_services(
    bleio_connection_obj_t *self,
    mp_obj_t service_uuids_whitelist);
```

```c
uint16_t bleio_connection_get_conn_handle(
    bleio_connection_obj_t *self);
```

```c
mp_obj_t bleio_connection_new_from_internal(
    bleio_connection_internal_t *internal);
```

```c
bleio_connection_internal_t *bleio_conn_handle_to_connection(
    uint16_t conn_handle);
```

---

#### `Descriptor.[c,h]`

```c
void common_hal_bleio_descriptor_construct(
    bleio_descriptor_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    bleio_uuid_obj_t *uuid,
    bleio_attribute_security_mode_t read_perm,
    bleio_attribute_security_mode_t write_perm,
    mp_int_t max_length,
    bool fixed_length,
    mp_buffer_info_t *initial_value_bufinfo);
```

```c
bleio_uuid_obj_t *common_hal_bleio_descriptor_get_uuid(
    bleio_descriptor_obj_t *self);
```

```c
bleio_characteristic_obj_t *common_hal_bleio_descriptor_get_characteristic(
    bleio_descriptor_obj_t *self);
```

```c
size_t common_hal_bleio_descriptor_get_value(
    bleio_descriptor_obj_t *self,
    uint8_t *buf,
    size_t len);
```

```c
void common_hal_bleio_descriptor_set_value(
    bleio_descriptor_obj_t *self,
    mp_buffer_info_t *bufinfo);
```

---

#### `PacketBuffer.[c,h]`

```c
STATIC void write_to_ringbuf(
    bleio_packet_buffer_obj_t *self,
    uint8_t *data,
    uint16_t len);
```

```c
STATIC uint32_t queue_next_write(
    bleio_packet_buffer_obj_t *self);
```

```c
STATIC bool packet_buffer_on_ble_client_evt(
    ble_evt_t *ble_evt,
    void *param);
```

```c
STATIC bool packet_buffer_on_ble_server_evt(
    ble_evt_t *ble_evt,
    void *param);
```

```c
void _common_hal_bleio_packet_buffer_construct(
    bleio_packet_buffer_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    uint32_t *incoming_buffer,
    size_t incoming_buffer_size,
    uint32_t *outgoing_buffer1,
    uint32_t *outgoing_buffer2,
    size_t max_packet_size,
    void *static_handler_entry);
```

```c
void common_hal_bleio_packet_buffer_construct(
    bleio_packet_buffer_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    size_t buffer_size,
    size_t max_packet_size);
```

```c
mp_int_t common_hal_bleio_packet_buffer_readinto(
    bleio_packet_buffer_obj_t *self,
    uint8_t *data,
    size_t len);
```

```c
mp_int_t common_hal_bleio_packet_buffer_write(
    bleio_packet_buffer_obj_t *self,
    const uint8_t *data,
    size_t len,
    uint8_t *header,
    size_t header_len);
```

```c
mp_int_t common_hal_bleio_packet_buffer_get_incoming_packet_length(
    bleio_packet_buffer_obj_t *self);
```

```c
mp_int_t common_hal_bleio_packet_buffer_get_outgoing_packet_length(
    bleio_packet_buffer_obj_t *self);
```

```c
void common_hal_bleio_packet_buffer_flush(
    bleio_packet_buffer_obj_t *self);
```

```c
bool common_hal_bleio_packet_buffer_deinited(
    bleio_packet_buffer_obj_t *self);
```

```c
void common_hal_bleio_packet_buffer_deinit(
    bleio_packet_buffer_obj_t *self);
```

---

#### `Service.[c,h]`

```c
STATIC void _indicate_service_change(
    uint16_t start,
    uint16_t end);
```

```c
uint32_t _common_hal_bleio_service_construct(
    bleio_service_obj_t *self,
    bleio_uuid_obj_t *uuid,
    bool is_secondary,
    mp_obj_list_t *characteristic_list);
```

```c
void common_hal_bleio_service_construct(
    bleio_service_obj_t *self,
    bleio_uuid_obj_t *uuid,
    bool is_secondary);
```

```c
void bleio_service_from_connection(
    bleio_service_obj_t *self,
    mp_obj_t connection);
```

```c
bleio_uuid_obj_t *common_hal_bleio_service_get_uuid(
    bleio_service_obj_t *self);
```

```c
mp_obj_tuple_t *common_hal_bleio_service_get_characteristics(
    bleio_service_obj_t *self);
```

```c
bool common_hal_bleio_service_get_is_remote(
    bleio_service_obj_t *self);
```

```c
bool common_hal_bleio_service_get_is_secondary(
    bleio_service_obj_t *self);
```

```c
STATIC void _expand_range(
    uint16_t new_value,
    uint16_t *start,
    uint16_t *end);
```

```c
void common_hal_bleio_service_add_characteristic(
    bleio_service_obj_t *self,
    bleio_characteristic_obj_t *characteristic,
    mp_buffer_info_t *initial_value_bufinfo,
    const char *user_description);
```

---

#### `UUID.[c,h]`

```c
typedef struct {
    mp_obj_base_t base;
    // Use the native way of storing UUID's:
    // - ble_uuid_t.uuid is a 16-bit uuid.
    // - ble_uuid_t.type is BLE_UUID_TYPE_BLE if it's a 16-bit Bluetooth SIG UUID.
    //   or is BLE_UUID_TYPE_VENDOR_BEGIN and higher, which indexes into a table of registered
    //   128-bit UUIDs.
    ble_uuid_t nrf_ble_uuid;
} bleio_uuid_obj_t;
```

```c
void common_hal_bleio_uuid_construct(
    bleio_uuid_obj_t *self,
    mp_int_t uuid16,
    const uint8_t uuid128[16]);
```

```c
uint32_t common_hal_bleio_uuid_get_size(
    bleio_uuid_obj_t *self);
```

```c
uint32_t common_hal_bleio_uuid_get_uuid16(
    bleio_uuid_obj_t *self);
```

```c
void common_hal_bleio_uuid_get_uuid128(
    bleio_uuid_obj_t *self,
    uint8_t uuid128[16]);
```

```c
void common_hal_bleio_uuid_pack_into(
    bleio_uuid_obj_t *self,
    uint8_t *buf);
```

```c
void bleio_uuid_construct_from_nrf_ble_uuid(
    bleio_uuid_obj_t *self,
    ble_uuid_t *nrf_ble_uuid);
```

```c
void bleio_uuid_convert_to_nrf_ble_uuid(
    bleio_uuid_obj_t *self,
    ble_uuid_t *nrf_ble_uuid);
```

---

#### `ble_drv.[c,h]`

```c
typedef bool (*ble_drv_evt_handler_t)(ble_evt_t *, void *);

typedef enum {
    SD_FLASH_OPERATION_DONE,
    SD_FLASH_OPERATION_IN_PROGRESS,
    SD_FLASH_OPERATION_ERROR,
} sd_flash_operation_status_t;

// Flag indicating progress of internal flash operation.
extern volatile sd_flash_operation_status_t sd_flash_operation_status;

typedef struct ble_drv_evt_handler_entry {
    struct ble_drv_evt_handler_entry *next;
    void *param;
    ble_drv_evt_handler_t func;
} ble_drv_evt_handler_entry_t;
```

```c
nrf_nvic_state_t nrf_nvic_state = { 0 };

// Flag indicating progress of internal flash operation.
volatile sd_flash_operation_status_t sd_flash_operation_status;

__attribute__((aligned(4)))
static uint8_t m_ble_evt_buf[sizeof(ble_evt_t) + (BLE_GATTS_VAR_ATTR_LEN_MAX)];
```

```c
void ble_drv_reset();
```

```c
void ble_drv_remove_heap_handlers(void);
```

```c
void ble_drv_add_event_handler_entry(
    ble_drv_evt_handler_entry_t *entry,
    ble_drv_evt_handler_t func,
    void *param);
```

```c
void ble_drv_add_event_handler(
    ble_drv_evt_handler_t func,
    void *param);
```

```c
void ble_drv_remove_event_handler(
    ble_drv_evt_handler_t func,
    void *param);
```

```c
void SD_EVT_IRQHandler(void);
```

```c
MP_REGISTER_ROOT_POINTER(ble_drv_evt_handler_entry_t * ble_drv_evt_handler_entries);
```
