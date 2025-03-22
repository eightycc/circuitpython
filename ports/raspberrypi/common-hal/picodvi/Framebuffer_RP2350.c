/*
 * This file is part of the Micro Python project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Scott Shawcroft for Adafruit Industries
 * Copyright (c) 2025 Robert Abeles for Adafruit Industries
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

#include "bindings/picodvi/Framebuffer.h"

#include "py/gc.h"
#include "py/runtime.h"
#include "shared-bindings/board/__init__.h"
#include "shared-bindings/busio/I2C.h"
#include "shared-bindings/time/__init__.h"
#include "supervisor/port.h"

#include "src/common/pico_stdlib_headers/include/pico/stdlib.h"

// This is from: https://github.com/raspberrypi/pico-examples-rp2350/blob/a1/hstx/dvi_out_hstx_encoder/dvi_out_hstx_encoder.c

#include "sdk/src/rp2_common/hardware_dma/include/hardware/dma.h"
#include "sdk/src/rp2_common/hardware_xip_cache/include/hardware/xip_cache.h"
#include "sdk/src/rp2350/hardware_structs/include/hardware/structs/bus_ctrl.h"
#include "sdk/src/rp2350/hardware_structs/include/hardware/structs/hstx_ctrl.h"
#include "sdk/src/rp2350/hardware_structs/include/hardware/structs/hstx_fifo.h"
#include "sdk/src/rp2350/hardware_structs/include/hardware/structs/xip_ctrl.h"
// ----------------------------------------------------------------------------
// DVI constants
//
// References:
//  [1] DVI 1.0 specification: http://www.cs.unc.edu/Research/stc/FAQs/Video/dvi_spec-V1_0.pdf
//  [2] DMT Version 1.13: https://glenwing.github.io/docs/VESA-DMT-1.13.pdf
//  [3] VESA E-EDID Standard Release A, Rev. 2: https://glenwing.github.io/docs/VESA-EEDID-A2.pdf

// These are 10-bit TMDS control symbols used to encode vertical and horizontal
// synchronization on TMDS channel 0. Additionally, the bit patterns were chosen
// to facilitate synchronization with the display's TMDS receiver by having a
// large number of level transitions.
#define TMDS_CTRL_00 0x354u // C0=0, C1=0, symbol=10'b11_0101_0100
#define TMDS_CTRL_01 0x0abu // C0=0, C1=1, symbol=10'b00_1010_1011
#define TMDS_CTRL_10 0x154u // C0=1, C1=0, symbol=10'b10_1010_1000
#define TMDS_CTRL_11 0x2abu // C0=1, C1=1, symbol=10'b10_1010_1011

// These are the 30-bit TMDS words that will be sent on channel 0 and decoded
// by the display to form vertical and horizontal synchronization pulses.
#define SYNC_V0_H0 (TMDS_CTRL_00 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V0_H1 (TMDS_CTRL_01 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H0 (TMDS_CTRL_10 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))
#define SYNC_V1_H1 (TMDS_CTRL_11 | (TMDS_CTRL_00 << 10) | (TMDS_CTRL_00 << 20))

// These constants are approximately derived from [2] and specify the geometric
// parameters for 640x480 and 720x400 video modes. Horizontal values are expressed
// in display pixels, vertical values are expressed in scanlines. The horizontal
// front porch values are the sum of the front porch and the right border.
// Likewise, the horizontal back porch values are the sum of the back porch and
// the left border.

// TODO: Rework as a table of mode parameters to support additional video modes.
#define MODE_720_H_SYNC_POLARITY 0
#define MODE_720_H_FRONT_PORCH   24
#define MODE_720_H_SYNC_WIDTH    64
#define MODE_720_H_BACK_PORCH    88
#define MODE_720_H_ACTIVE_PIXELS 720

#define MODE_720_V_SYNC_POLARITY 0
#define MODE_720_V_FRONT_PORCH   3
#define MODE_720_V_SYNC_WIDTH    4
#define MODE_720_V_BACK_PORCH    13
#define MODE_720_V_ACTIVE_LINES  400

#define MODE_640_H_SYNC_POLARITY 0
#define MODE_640_H_FRONT_PORCH   16     // right border = 8, front porch = 8
#define MODE_640_H_SYNC_WIDTH    96
#define MODE_640_H_BACK_PORCH    48     // back porch = 40, left border = 8
#define MODE_640_H_ACTIVE_PIXELS 640

#define MODE_640_V_SYNC_POLARITY 0
#define MODE_640_V_FRONT_PORCH   10
#define MODE_640_V_SYNC_WIDTH    2
#define MODE_640_V_BACK_PORCH    33
#define MODE_640_V_ACTIVE_LINES  480

#define MODE_720_V_TOTAL_LINES  ( \
    MODE_720_V_FRONT_PORCH + MODE_720_V_SYNC_WIDTH + \
    MODE_720_V_BACK_PORCH + MODE_720_V_ACTIVE_LINES \
    )
#define MODE_640_V_TOTAL_LINES  ( \
    MODE_640_V_FRONT_PORCH + MODE_640_V_SYNC_WIDTH + \
    MODE_640_V_BACK_PORCH + MODE_640_V_ACTIVE_LINES \
    )

#define HSTX_CMD_RAW         (0x0u << 12)
#define HSTX_CMD_RAW_REPEAT  (0x1u << 12)
#define HSTX_CMD_TMDS        (0x2u << 12)
#define HSTX_CMD_TMDS_REPEAT (0x3u << 12)
#define HSTX_CMD_NOP         (0xfu << 12)

// ----------------------------------------------------------------------------
// HSTX command lists

#define VSYNC_LEN 6
#define VACTIVE_LEN 9

static uint32_t vblank_line640_vsync_off[VSYNC_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_640_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_640_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_640_H_BACK_PORCH + MODE_640_H_ACTIVE_PIXELS),
    SYNC_V1_H1
};

static uint32_t vblank_line640_vsync_on[VSYNC_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_640_H_FRONT_PORCH,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT | MODE_640_H_SYNC_WIDTH,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_640_H_BACK_PORCH + MODE_640_H_ACTIVE_PIXELS),
    SYNC_V0_H1
};

static uint32_t vactive_line640[VACTIVE_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_640_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_NOP,
    HSTX_CMD_RAW_REPEAT | MODE_640_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_NOP,
    HSTX_CMD_RAW_REPEAT | MODE_640_H_BACK_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_TMDS | MODE_640_H_ACTIVE_PIXELS
};

static uint32_t vblank_line720_vsync_off[VSYNC_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_720_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_RAW_REPEAT | MODE_720_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_720_H_BACK_PORCH + MODE_720_H_ACTIVE_PIXELS),
    SYNC_V1_H1
};

static uint32_t vblank_line720_vsync_on[VSYNC_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_720_H_FRONT_PORCH,
    SYNC_V0_H1,
    HSTX_CMD_RAW_REPEAT | MODE_720_H_SYNC_WIDTH,
    SYNC_V0_H0,
    HSTX_CMD_RAW_REPEAT | (MODE_720_H_BACK_PORCH + MODE_720_H_ACTIVE_PIXELS),
    SYNC_V0_H1
};

static uint32_t vactive_line720[VACTIVE_LEN] = {
    HSTX_CMD_RAW_REPEAT | MODE_720_H_FRONT_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_NOP,
    HSTX_CMD_RAW_REPEAT | MODE_720_H_SYNC_WIDTH,
    SYNC_V1_H0,
    HSTX_CMD_NOP,
    HSTX_CMD_RAW_REPEAT | MODE_720_H_BACK_PORCH,
    SYNC_V1_H1,
    HSTX_CMD_TMDS | MODE_720_H_ACTIVE_PIXELS
};

static picodvi_framebuffer_obj_t *_active_picodvi = NULL;
static bool _edid_valid = false;
static uint8_t _edid[128];

static void read_edid(void) {
    // If we've already read the monitor's EDID block, return.
    if (_edid_valid) {
        return;
    }
    // Attempt to probe the monitor's I2C port at 0x50.
    busio_i2c_obj_t *i2c = common_hal_board_create_i2c(0);
    if (!i2c) {
        return;
    }
    if (!common_hal_busio_i2c_try_lock(i2c)) {
        return;
    }
    if (!common_hal_busio_i2c_probe(i2c, 0x50)) {
        common_hal_busio_i2c_unlock(i2c);
        return;
    }

    // Read the monitor's EDID block.
    uint8_t out[1] = {0};
    common_hal_busio_i2c_write_read(i2c, 0x50, out, 1, _edid, sizeof(_edid));
    common_hal_busio_i2c_unlock(i2c);

    // Validate the EDID block by verifying its header and checksum.
    const uint8_t edid_header[] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
    if (memcmp(_edid, edid_header, sizeof(edid_header)) != 0) {
        return;
    }
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(_edid); i++) {
        checksum += _edid[i];
    }
    if (checksum != 0) {
        return;
    }
    _edid_valid = true;
}

static void __not_in_flash_func(_dma_irq_handler)(void) {
    if (_active_picodvi == NULL || _active_picodvi->dma_paused) {
        return;
    }
    if (_active_picodvi->dma_pausing) {
        // Disable command and pixel DMA channels to ensure no asynchronous transfers,
        // triggers, or interrupts occur while paused.
        dma_channel_hw_t *pix_ch = &dma_hw->ch[_active_picodvi->dma_pixel_channel];
        dma_channel_hw_t *cmd_ch = &dma_hw->ch[_active_picodvi->dma_command_channel];
        pix_ch->al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
        cmd_ch->al1_ctrl &= ~DMA_CH0_CTRL_TRIG_EN_BITS;
        _active_picodvi->dma_pausing = false;
        _active_picodvi->dma_paused = true;
        return;
    }
    // For framebuffer in PSRAM, reset XIP streaming to start of framebuffer.
    if (_active_picodvi->framebuffer_in_psram) {
        xip_ctrl_hw->stream_addr = (uintptr_t)_active_picodvi->framebuffer;
        xip_ctrl_hw->stream_ctr = _active_picodvi->pitch * sizeof(uint32_t);
    }
    uint ch_num = _active_picodvi->dma_pixel_channel;
    dma_hw->intr = 1u << ch_num;

    // Set the read_addr back to the start and trigger the first transfer (which
    // will trigger the pixel channel).
    dma_channel_hw_t *ch = &dma_hw->ch[_active_picodvi->dma_command_channel];
    ch->al3_read_addr_trig = (uintptr_t)_active_picodvi->dma_commands;
}

static void _pause_picodvi_dma(picodvi_framebuffer_obj_t *self, bool pause) {
    if (pause) {
        // Synchronize with end of the current frame.
        // TODO: Presently we stop feeding the HSTX FIFO at the end of the current
        // frame. Because the TMDS output stops when the HSTX FIFO empties, the monitor
        // may lose synchronization leading to a blank screen for an extended period
        // of time. As an alternative, consider switching to a fake framebuffer of
        // all black pixels to maintain monitor synchronization.
        self->dma_pausing = true;
        // Wait for the DMA IRQ handler to pause the DMA channels
        while (!self->dma_paused) {
            tight_loop_contents();
        }
        assert(!self->dma_pausing);
        assert(self->dma_paused);
        assert(!dma_channel_is_busy(self->dma_pixel_channel));
        assert(!dma_channel_is_busy(self->dma_command_channel));
    } else {
        self->dma_paused = false;
        _dma_irq_handler();
    }
}

static void _turn_off_dma(int channel) {
    if (channel < 0) {
        return;
    }
    dma_channel_config c = dma_channel_get_default_config(channel);
    channel_config_set_enable(&c, false);
    dma_channel_set_config(channel, &c, false /* trigger */);

    if (dma_channel_is_busy(channel)) {
        dma_channel_abort(channel);
    }
    dma_channel_set_irq1_enabled(channel, false);
    dma_channel_unclaim(channel);
}

void common_hal_picodvi_framebuffer_flash_pre_write() {
    if (_active_picodvi == NULL || !_active_picodvi->framebuffer_in_psram) {
        return;
    }
    _pause_picodvi_dma(_active_picodvi, true);
}

void common_hal_picodvi_framebuffer_flash_post_write() {
    if (_active_picodvi == NULL || !_active_picodvi->framebuffer_in_psram) {
        return;
    }
    _pause_picodvi_dma(_active_picodvi, false);
}

bool common_hal_picodvi_framebuffer_preflight(
    mp_uint_t width, mp_uint_t height,
    mp_uint_t color_depth) {

    // These modes don't duplicate pixels so we can do sub-byte colors. They
    // take too much ram for more than 8bit color though.
    bool full_resolution = color_depth == 1 || color_depth == 2 || color_depth == 4 || color_depth == 8;
    // These modes rely on the memory transfer to duplicate values across bytes.
    bool doubled = color_depth == 8 || color_depth == 16 || color_depth == 32;

    // for each supported resolution, check the color depth is supported
    if (width == 640 && height == 480) {
        return full_resolution;
    }
    if (width == 320 && height == 240) {
        return doubled;
    }
    if (width == 160 && height == 120) {
        return doubled;
    }

    if (width == 720 && height == 400) {
        return full_resolution;
    }

    if (width == 360 && height == 200) {
        return doubled;
    }

    if (width == 180 && height == 100) {
        return doubled;
    }
    return false;
}

void common_hal_picodvi_framebuffer_construct(picodvi_framebuffer_obj_t *self,
    mp_uint_t width, mp_uint_t height,
    const mcu_pin_obj_t *clk_dp, const mcu_pin_obj_t *clk_dn,
    const mcu_pin_obj_t *red_dp, const mcu_pin_obj_t *red_dn,
    const mcu_pin_obj_t *green_dp, const mcu_pin_obj_t *green_dn,
    const mcu_pin_obj_t *blue_dp, const mcu_pin_obj_t *blue_dn,
    mp_uint_t color_depth,
    bool psram_framebuffer) {
    if (_active_picodvi != NULL) {
        mp_raise_msg_varg(&mp_type_RuntimeError, MP_ERROR_TEXT("%q in use"), MP_QSTR_picodvi);
    }

    // Read, validate, and cache the monitor's EDID block.
    read_edid();

    if (!common_hal_picodvi_framebuffer_preflight(width, height, color_depth)) {
        mp_raise_ValueError_varg(MP_ERROR_TEXT("Invalid %q and %q"), MP_QSTR_width, MP_QSTR_height);
    }

    self->dma_command_channel = -1;
    self->dma_pixel_channel = -1;

    if (width % 160 == 0) {
        self->output_width = 640;
    } else {
        self->output_width = 720;
    }
    size_t output_scaling = self->output_width / width;

    size_t all_allocated = 0;
    int8_t pins[8] = {
        clk_dp->number, clk_dn->number,
        red_dp->number, red_dn->number,
        green_dp->number, green_dn->number,
        blue_dp->number, blue_dn->number
    };
    qstr pin_names[8] = {
        MP_QSTR_clk_dp, MP_QSTR_clk_dn,
        MP_QSTR_red_dp, MP_QSTR_red_dn,
        MP_QSTR_green_dp, MP_QSTR_green_dn,
        MP_QSTR_blue_dp, MP_QSTR_blue_dn
    };
    for (size_t i = 0; i < 8; i++) {
        if (!(12 <= pins[i] && pins[i] <= 19)) {
            raise_ValueError_invalid_pin_name(pin_names[i]);
        }
        pins[i] -= 12;
        size_t mask = 1 << pins[i];
        if ((all_allocated & mask) != 0) {
            raise_ValueError_invalid_pin_name(pin_names[i]);
        }
        all_allocated |= mask;
    }

    self->width = width;
    self->height = height;
    self->color_depth = color_depth;
    // Pitch is number of 32-bit words per line. We round up pitch_bytes to the nearest word
    // so that each scanline begins on a natural 32-bit word boundary.
    size_t pitch_bytes = (self->width * color_depth) / 8;
    self->pitch = (pitch_bytes + sizeof(uint32_t) - 1) / sizeof(uint32_t);
    // TODO: If pitch requires rounding up for a PSRAM framebuffer, it will require a
    // per-scanline interrupt to adjust the XIP streaming read address. Yeech.
    assert(!self->framebuffer_in_psram || self->pitch * sizeof(uint32_t) == pitch_bytes);
    size_t framebuffer_size = self->pitch * self->height;

    // A framebuffer that exceeds the capacity of the SRAM will be allocated in PSRAM.
    // If the framebuffer is in PSRAM, HSTX output will be paused during flash writes,
    // resulting in a blank screen for the duration of the flash write.
    //
    // For testing purposes, add 1MB to the framebuffer allocation size to force it into PSRAM.
    // NOTE: the dma_capable parameter of port_malloc() does nothing.
    size_t framebuffer_alloc_size = framebuffer_size * sizeof(uint32_t) /*+ 1024 * 1024*/;
    self->framebuffer = (uint32_t *)port_malloc(framebuffer_alloc_size, true);
    self->framebuffer_in_psram = ((size_t)self->framebuffer & 0xf0000000) == 0x10000000;
    if (self->framebuffer == NULL) {
        common_hal_picodvi_framebuffer_deinit(self);
        m_malloc_fail(framebuffer_size * sizeof(uint32_t));
        return;
    }
    if (self->framebuffer_in_psram) {
        // For a framebuffer in PSRAM, clean the XIP cache and relocate the framebuffer
        // to the non-cached PSRAM alias in the XIP address space. Since framebuffer
        // accesses are sequential over a large space, if we use the cached PSRAM alias
        // we will destroy the cache's utility for other code. Timing tests show that
        // the non-cached PSRAM alias is faster for sequential accesses than the cached
        // PSRAM alias.
        //
        // TODO: Consider garbage collection ramifications of using non-cached PSRAM.
        // xip_cache_clean_all();
        // xip_cache_invalidate_range((uintptr_t)self->framebuffer, framebuffer_size * sizeof(uint32_t));
        self->framebuffer = (uint32_t *)((uint32_t)self->framebuffer | 0x04000000);
    }
    memset(self->framebuffer, 0, framebuffer_size * sizeof(uint32_t));

    // We compute all DMA transfers needed for a single frame. This ensure we don't have any super
    // quick interrupts that we need to respond to. Each transfer takes two words, trans_count and
    // read_addr. Active pixel lines need two transfers due to different read addresses. When pixel
    // doubling, then we must also set transfer size.
    size_t dma_command_size = 2;
    if (output_scaling > 1) {
        dma_command_size = 4;
    }

    if (self->output_width == 640) {
        self->dma_commands_len = (MODE_640_V_FRONT_PORCH + MODE_640_V_SYNC_WIDTH + MODE_640_V_BACK_PORCH + 2 * MODE_640_V_ACTIVE_LINES + 1) * dma_command_size;
    } else {
        self->dma_commands_len = (MODE_720_V_FRONT_PORCH + MODE_720_V_SYNC_WIDTH + MODE_720_V_BACK_PORCH + 2 * MODE_720_V_ACTIVE_LINES + 1) * dma_command_size;
    }
    self->dma_commands = (uint32_t *)port_malloc(self->dma_commands_len * sizeof(uint32_t), true);
    if (self->dma_commands == NULL || ((size_t)self->dma_commands & 0xf0000000) == 0x10000000) {
        common_hal_picodvi_framebuffer_deinit(self);
        m_malloc_fail(self->dma_commands_len * sizeof(uint32_t));
        return;
    }

    // The command channel and the pixel channel form a pipeline that feeds combined HSTX
    // commands and pixel data to the HSTX FIFO. The command channel reads a pre-computed
    // list of control/status words from the dma_commands buffer and writes them to the
    // pixel channel's control/status registers. Under control of the command channel, the
    // pixel channel smears/swizzles pixel data from the framebuffer and combines
    // it with HSTX commands, forwarding the combined stream to the HSTX FIFO.

    self->dma_pixel_channel = dma_claim_unused_channel(false);
    self->dma_command_channel = dma_claim_unused_channel(false);
    if (self->dma_pixel_channel < 0 || self->dma_command_channel < 0) {
        common_hal_picodvi_framebuffer_deinit(self);
        mp_raise_RuntimeError(MP_ERROR_TEXT("Internal resource(s) in use"));
        return;
    }

    size_t command_word = 0;
    size_t frontporch_start;
    if (self->output_width == 640) {
        frontporch_start = MODE_640_V_TOTAL_LINES - MODE_640_V_FRONT_PORCH;
    } else {
        frontporch_start = MODE_720_V_TOTAL_LINES - MODE_720_V_FRONT_PORCH;
    }
    size_t frontporch_end = frontporch_start;
    if (self->output_width == 640) {
        frontporch_end += MODE_640_V_FRONT_PORCH;
    } else {
        frontporch_end += MODE_720_V_FRONT_PORCH;
    }
    size_t vsync_start = 0;
    size_t vsync_end = vsync_start;
    if (self->output_width == 640) {
        vsync_end += MODE_640_V_SYNC_WIDTH;
    } else {
        vsync_end += MODE_720_V_SYNC_WIDTH;
    }
    size_t backporch_start = vsync_end;
    size_t backporch_end = backporch_start;
    if (self->output_width == 640) {
        backporch_end += MODE_640_V_BACK_PORCH;
    } else {
        backporch_end += MODE_720_V_BACK_PORCH;
    }
    size_t active_start = backporch_end;

    uint32_t dma_ctrl = self->dma_command_channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB |
        DREQ_HSTX << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB |
        DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS |
        DMA_CH0_CTRL_TRIG_INCR_READ_BITS |
        DMA_CH0_CTRL_TRIG_EN_BITS;
    // For framebuffer in PSRAM, do not increment the read address, and use DREQ_XIP_STREAM
    // for pacing.
    if (self->framebuffer_in_psram) {
        dma_ctrl &= ~DMA_CH0_CTRL_TRIG_INCR_READ_BITS;
        dma_ctrl = (dma_ctrl & ~DMA_CH0_CTRL_TRIG_TREQ_SEL_BITS) | (DREQ_XIP_STREAM << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB);
    }
    uint32_t dma_pixel_ctrl;
    if (output_scaling > 1) {
        // We do color_depth size transfers when pixel doubling. The memory bus will
        // duplicate the bytes read (byte lane smearing) to produce 32 bits for the HSTX.
        if (color_depth == 32) {
            dma_pixel_ctrl = dma_ctrl | DMA_SIZE_32 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
        } else if (color_depth == 16) {
            dma_pixel_ctrl = dma_ctrl | DMA_SIZE_16 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
        } else {
            dma_pixel_ctrl = dma_ctrl | DMA_SIZE_8 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
        }
    } else {
        dma_pixel_ctrl = dma_ctrl | DMA_SIZE_32 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;
    }
    if (self->color_depth == 16) {
        dma_pixel_ctrl |= DMA_CH0_CTRL_TRIG_BSWAP_BITS;
    }
    dma_ctrl |= DMA_SIZE_32 << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB;

    uint32_t dma_write_addr = (uint32_t)&hstx_fifo_hw->fifo;
    // Write ctrl and write_addr once when not pixel doubling because they don't
    // change. (write_addr doesn't change when pixel doubling either but we need
    // to rewrite it because it is after the ctrl register.)
    if (output_scaling == 1) {
        dma_channel_hw_addr(self->dma_pixel_channel)->al1_ctrl = dma_ctrl;
        dma_channel_hw_addr(self->dma_pixel_channel)->al1_write_addr = dma_write_addr;
    }

    uint32_t *vblank_line_vsync_on = self->output_width == 640 ?  vblank_line640_vsync_on : vblank_line720_vsync_on;
    uint32_t *vblank_line_vsync_off = self->output_width == 640 ?  vblank_line640_vsync_off : vblank_line720_vsync_off;
    uint32_t *vactive_line = self->output_width == 640 ?  vactive_line640 : vactive_line720;

    size_t mode_v_total_lines;
    if (self->output_width == 640) {
        mode_v_total_lines = MODE_640_V_TOTAL_LINES;
    } else {
        mode_v_total_lines = MODE_720_V_TOTAL_LINES;
    }

    for (size_t v_scanline = 0; v_scanline < mode_v_total_lines; v_scanline++) {
        if (output_scaling > 1) {
            self->dma_commands[command_word++] = dma_ctrl;
            self->dma_commands[command_word++] = dma_write_addr;
        }
        if (vsync_start <= v_scanline && v_scanline < vsync_end) {
            self->dma_commands[command_word++] = VSYNC_LEN;
            self->dma_commands[command_word++] = (uintptr_t)vblank_line_vsync_on;
        } else if (backporch_start <= v_scanline && v_scanline < backporch_end) {
            self->dma_commands[command_word++] = VSYNC_LEN;
            self->dma_commands[command_word++] = (uintptr_t)vblank_line_vsync_off;
        } else if (frontporch_start <= v_scanline && v_scanline < frontporch_end) {
            self->dma_commands[command_word++] = VSYNC_LEN;
            self->dma_commands[command_word++] = (uintptr_t)vblank_line_vsync_off;
        } else {
            self->dma_commands[command_word++] = VACTIVE_LEN;
            self->dma_commands[command_word++] = (uintptr_t)vactive_line;
            size_t row = v_scanline - active_start;
            size_t transfer_count = self->pitch;
            if (output_scaling > 1) {
                self->dma_commands[command_word++] = dma_pixel_ctrl;
                self->dma_commands[command_word++] = dma_write_addr;
                row /= output_scaling;
                // When pixel scaling, we do one transfer per pixel and it gets
                // mirrored into the rest of the word.
                transfer_count = self->width;
            }
            self->dma_commands[command_word++] = transfer_count;
            uint32_t *row_start = (self->framebuffer_in_psram)?
                (uint32_t *)XIP_AUX_BASE : &self->framebuffer[row * self->pitch];
            self->dma_commands[command_word++] = (uintptr_t)row_start;
        }
    }
    // Last command is NULL which will trigger an IRQ.
    if (output_scaling > 1) {
        self->dma_commands[command_word++] = DMA_CH0_CTRL_TRIG_IRQ_QUIET_BITS |
            DMA_CH0_CTRL_TRIG_EN_BITS;
        self->dma_commands[command_word++] = 0;
    }
    self->dma_commands[command_word++] = 0;
    self->dma_commands[command_word++] = 0;

    if (color_depth == 32) {
        // Configure HSTX's TMDS encoder for RGB888
        hstx_ctrl_hw->expand_tmds =
            7 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                16 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                7 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                8 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                7 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                0 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
    } else if (color_depth == 16) {
        // Configure HSTX's TMDS encoder for RGB565
        hstx_ctrl_hw->expand_tmds =
            4 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                5 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                27 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                4 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                21 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
    } else if (color_depth == 8) {
        // Configure HSTX's TMDS encoder for RGB332
        hstx_ctrl_hw->expand_tmds =
            2 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                0 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                2 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                29 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                1 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
    } else if (color_depth == 4) {
        // Configure HSTX's TMDS encoder for RGBD
        hstx_ctrl_hw->expand_tmds =
            0 << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                28 << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                0 << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                27 << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                0 << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                26 << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
    } else {
        // Grayscale
        uint8_t rot = 24 + color_depth;
        hstx_ctrl_hw->expand_tmds =
            (color_depth - 1) << HSTX_CTRL_EXPAND_TMDS_L2_NBITS_LSB |
                rot << HSTX_CTRL_EXPAND_TMDS_L2_ROT_LSB |
                    (color_depth - 1) << HSTX_CTRL_EXPAND_TMDS_L1_NBITS_LSB |
                rot << HSTX_CTRL_EXPAND_TMDS_L1_ROT_LSB |
                    (color_depth - 1) << HSTX_CTRL_EXPAND_TMDS_L0_NBITS_LSB |
                rot << HSTX_CTRL_EXPAND_TMDS_L0_ROT_LSB;
    }
    size_t pixels_per_word;
    if (output_scaling == 1) {
        pixels_per_word = 32 / color_depth;
    } else {
        pixels_per_word = 1;
    }

    size_t shifts_before_empty = (pixels_per_word % 32);
    if (output_scaling > 1) {
        shifts_before_empty *= output_scaling;
    }

    size_t shift_amount = color_depth % 32;

    // Pixels come in 32 bits at a time. color_depth dictates the number
    // of pixels per word. Control symbols (RAW) are an entire 32-bit word.
    hstx_ctrl_hw->expand_shift =
        shifts_before_empty << HSTX_CTRL_EXPAND_SHIFT_ENC_N_SHIFTS_LSB |
            shift_amount << HSTX_CTRL_EXPAND_SHIFT_ENC_SHIFT_LSB |
            1 << HSTX_CTRL_EXPAND_SHIFT_RAW_N_SHIFTS_LSB |
            0 << HSTX_CTRL_EXPAND_SHIFT_RAW_SHIFT_LSB;

    // Serial output config: clock period of 5 cycles, pop from command
    // expander every 5 cycles, shift the output shiftreg by 2 every cycle.
    hstx_ctrl_hw->csr = 0;
    hstx_ctrl_hw->csr =
        HSTX_CTRL_CSR_EXPAND_EN_BITS |
        5u << HSTX_CTRL_CSR_CLKDIV_LSB |
            5u << HSTX_CTRL_CSR_N_SHIFTS_LSB |
            2u << HSTX_CTRL_CSR_SHIFT_LSB |
            HSTX_CTRL_CSR_EN_BITS;

    // Note we are leaving the HSTX clock at the SDK default of 125 MHz; since
    // we shift out two bits per HSTX clock cycle, this gives us an output of
    // 250 Mbps, which is very close to the bit clock for 480p 60Hz (252 MHz).
    // If we want the exact rate then we'll have to reconfigure PLLs.

    // For proper monitor operation, the HSTX clock rate must be an integral
    // multiple of the pixel clock rate. The pixel clock rate is determined by
    // the monitor's display resolution and refresh rate.

    // The HSTX clock divider is sourced by default from the system clock
    // (CLK_SYS), which normally runs at 150 MHz for the RP2350. By default,
    // the HSTX clock divider is set to 1, yielding a 150 MHz HSTX clock. The
    // HSTX clock divider is more limited than other dividers, allowing only
    // integer divisors in the range 1..4. Thus, HSTX clock rates are limited
    // to 150 MHz, 75 MHz, 37.5 MHz, and 18.75 MHz when using the default
    // configuration.

    // The HSTX clock divider can be sourced from the PLL_SYS or the PLL_USB.
    // Because HSTX logic runs in its own clock domain, it is not necessary for
    // the HSTX clock to be phase-locked with the system clock. By adjusting
    // PLL_SYS down to 144 MHz, it can be used as the source for both CLK_SYS
    // and CLK_USB because 144 / 3 is exactly 48 MHz. Doing this frees the
    // PLL_USB generator for use with HSTX, allowing accurate pixel clock rates
    // to be generated. The costs of doing this are a 4% reduction in CPU speed
    // and a restriction on overclocking such that CLK_SYS must be an integral
    // multiple of 48 MHz.

    // A potential benefit of reducing SYS_CLK to 144 MHz is that Pico-PIO-USB
    // can set its PIO clock dividers to an integral value, allowing accurate
    // generation of USB bit rates without the jitter arising from fractional
    // divisors. This is especially important for USB 2.0 full-speed mode where
    // the specification requires a transmit rate of 12.0000 Mbps +/- 0.25%.

    // Although the CLK_SYS divider can be set with fractional divisors, we
    // avoid doing so because setting a fractional divisor will cause CLK_SYS
    // to jitter (see RP2350 datasheet, section 8.1.2.3, "Divider"). The
    // downstream effect on other clocks using CLK_SYS as their source is
    // unknown, but could be problematical.

    // The PLL sections use a Voltage Controlled Oscillator (VCO) to generate
    // a high frequency clock using an external source as a reference. All
    // boards supported by the Pico SDK must use a 12 MHz reference. The VCO
    // output can be programmed to yield a frequency in the range 750 MHz to
    // 1500 MHz. The VCO output is divided down by a cascade of two dividers,
    // each with a programmable divisor in the range 1..7, to yield the PLL
    // output frequency. This arrangement allows accurate generation of any
    // desired pixel clock rate, but it is complicated to calculate the VCO
    // and divider factors. To simplify this, we pre-calculate a table of
    // commonly used pixel clock rates and their associated PLL factors using
    // the PLL calculator tool provided by the Raspberry Pi Foundation. The
    // tool can be found here:
    // https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_clocks/scripts/vcocalc.py


    // Setup the data to pin mapping. `pins` is a pair of pins in a standard
    // order: clock, red, green and blue. We don't actually care they are next
    // to one another but they'll work better that way.
    for (size_t i = 0; i < 8; i++) {
        uint lane = i / 2;
        size_t invert = i % 2 == 1 ? HSTX_CTRL_BIT0_INV_BITS : 0;
        uint32_t lane_data_sel_bits;
        // Clock
        if (lane == 0) {
            lane_data_sel_bits = HSTX_CTRL_BIT0_CLK_BITS;
        } else {
            // Output even bits during first half of each HSTX cycle, and odd bits
            // during second half. The shifter advances by two bits each cycle.
            lane -= 1;
            lane_data_sel_bits =
                (lane * 10) << HSTX_CTRL_BIT0_SEL_P_LSB |
                        (lane * 10 + 1) << HSTX_CTRL_BIT0_SEL_N_LSB;
        }
        hstx_ctrl_hw->bit[pins[i]] = lane_data_sel_bits | invert;
    }

    for (int i = 12; i <= 19; ++i) {
        gpio_set_function(i, 0); // HSTX
        never_reset_pin_number(i);
    }

    dma_channel_config c;
    c = dma_channel_get_default_config(self->dma_command_channel);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, true);
    // This wraps the transfer back to the start of the write address.
    size_t wrap = 3; // 8 bytes because we write two DMA registers.
    volatile uint32_t *write_addr = &dma_hw->ch[self->dma_pixel_channel].al3_transfer_count;
    if (output_scaling > 1) {
        wrap = 4; // 16 bytes because we write all four DMA registers.
        write_addr = &dma_hw->ch[self->dma_pixel_channel].al3_ctrl;
    }
    channel_config_set_ring(&c, true, wrap);
    // No chain because we use an interrupt to reload this channel instead of a
    // third channel.
    dma_channel_configure(
        self->dma_command_channel,
        &c,
        write_addr,
        self->dma_commands,
        (1 << wrap) / sizeof(uint32_t),
        false
        );

    dma_hw->ints1 = (1u << self->dma_pixel_channel);
    dma_hw->inte1 = (1u << self->dma_pixel_channel);
    irq_set_exclusive_handler(DMA_IRQ_1, _dma_irq_handler);
    irq_set_enabled(DMA_IRQ_1, true);
    irq_set_priority(DMA_IRQ_1, PICO_HIGHEST_IRQ_PRIORITY);
    self->dma_irq_handler_installed = true;

    bus_ctrl_hw->priority = BUSCTRL_BUS_PRIORITY_DMA_W_BITS | BUSCTRL_BUS_PRIORITY_DMA_R_BITS;

    // For the output.
    self->framebuffer_len = framebuffer_size;

    _active_picodvi = self;

    // For a framebuffer in PSRAM, ensure that the XIP streaming is quiescent.
    if (self->framebuffer_in_psram) {
        while (!(xip_ctrl_hw->stat & XIP_STAT_FIFO_EMPTY)) {
            (void)xip_ctrl_hw->stream_fifo;
        }
    }

    common_hal_picodvi_framebuffer_refresh(self);
    _dma_irq_handler();
}

void common_hal_picodvi_framebuffer_deinit(picodvi_framebuffer_obj_t *self) {
    if (common_hal_picodvi_framebuffer_deinited(self)) {
        return;
    }

    for (int i = 12; i <= 19; ++i) {
        reset_pin_number(i);
    }

    _turn_off_dma(self->dma_pixel_channel);
    _turn_off_dma(self->dma_command_channel);
    self->dma_pixel_channel = -1;
    self->dma_command_channel = -1;

    if (self->dma_irq_handler_installed) {
        irq_set_enabled(DMA_IRQ_1, false);
        irq_remove_handler(DMA_IRQ_1, _dma_irq_handler);
        self->dma_irq_handler_installed = false;
    }

    _active_picodvi = NULL;

    if (self->framebuffer_in_psram) {
        // Relocate framebuffer back into cached PSRAM alias because that's where
        // tlsf expects it to be for the free.
        self->framebuffer = (uint32_t *)((uint32_t)self->framebuffer & ~0x04000000);
    }
    port_free(self->framebuffer);
    self->framebuffer = NULL;

    port_free(self->dma_commands);
    self->dma_commands = NULL;

    self->base.type = &mp_type_NoneType;
}

bool common_hal_picodvi_framebuffer_deinited(picodvi_framebuffer_obj_t *self) {
    return self->framebuffer == NULL;
}

void common_hal_picodvi_framebuffer_refresh(picodvi_framebuffer_obj_t *self) {
}

int common_hal_picodvi_framebuffer_get_width(picodvi_framebuffer_obj_t *self) {
    return self->width;
}

int common_hal_picodvi_framebuffer_get_height(picodvi_framebuffer_obj_t *self) {
    return self->height;
}

int common_hal_picodvi_framebuffer_get_color_depth(picodvi_framebuffer_obj_t *self) {
    return self->color_depth;
}

int common_hal_picodvi_framebuffer_get_native_frames_per_second(picodvi_framebuffer_obj_t *self) {
    return self->output_width == 640 ? 60 : 70;
}

bool common_hal_picodvi_framebuffer_get_grayscale(picodvi_framebuffer_obj_t *self) {
    return self->color_depth < 4;
}

mp_int_t common_hal_picodvi_framebuffer_get_buffer(mp_obj_t self_in, mp_buffer_info_t *bufinfo, mp_uint_t flags) {
    picodvi_framebuffer_obj_t *self = (picodvi_framebuffer_obj_t *)self_in;
    bufinfo->buf = self->framebuffer;
    if (self->color_depth == 32) {
        bufinfo->typecode = 'I';
    } else if (self->color_depth == 16) {
        bufinfo->typecode = 'H';
    } else {
        bufinfo->typecode = 'B';
    }
    bufinfo->len = self->framebuffer_len * sizeof(uint32_t);
    return 0;
}

int common_hal_picodvi_framebuffer_get_row_stride(picodvi_framebuffer_obj_t *self) {
    // Pitch is in words but row stride is expected as bytes.
    return self->pitch * sizeof(uint32_t);
}
