/*
 * Copyright (c) 2023-2025 Arm Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "device_definition.h"
#include "hdlcd_drv.h"
#include "hdlcd_helper.h"
#include "stdint.h"
#include <stddef.h>
#include <stdio.h>

#include "runtime_initialization_settings.h"

/*
 * Semihosting is a mechanism that enables code running on an ARM target
 * to communicate and use the Input/Output facilities of a host computer
 * that is running a debugger.
 * There is an issue where if you use armclang at -O0 optimisation with
 * no parameters specified in the main function, the initialisation code
 * contains a breakpoint for semihosting by default. This will stop the
 * code from running before main is reached.
 * Semihosting can be disabled by defining __ARM_use_no_argv symbol
 * (or using higher optimization level).
 */
#if defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
__asm("  .global __ARM_use_no_argv\n");
#endif

/* Display full resolution or downscaled image */
#define DISPLAY_FR_NDS 1

static void hdlcd_show(uint32_t address, uint32_t width, uint32_t height, uint32_t mode);
static void handle_fr_frame(uint32_t address, uint32_t width, uint32_t height, uint32_t mode, uint32_t frame_id);
static void handle_ds_frame(uint32_t address, uint32_t width, uint32_t height, uint32_t mode, uint32_t frame_id);

extern int isp_init();

void vStartISPDemo()
{
    enum hdlcd_error_t hdlcd_err;

    printf("Starting HDLCD config!\r\n");

    hdlcd_err = hdlcd_init(&HDLCD_DEV);
    if (hdlcd_err != HDLCD_ERR_NONE) {
        printf("HDLCD init error! \r\n");
        while (1)
            ;
    }

    hdlcd_err = hdlcd_static_config(&HDLCD_DEV);
    if (hdlcd_err != HDLCD_ERR_NONE) {
        printf("HDLCD static config error! \r\n");
        while (1)
            ;
    }

    printf("Starting ISP test!\r\n");

    isp_init();

    ds_frame_ready = handle_ds_frame;
    fr_frame_ready = handle_fr_frame;
}

static void handle_fr_frame(uint32_t address, uint32_t width, uint32_t height, uint32_t mode, uint32_t frame_id)
{
    /* If the previous full frame wasn't inferred yet, skip this frame */
#if DISPLAY_FR_NDS
    hdlcd_show(address, width, height, mode);
#endif /* DISPLAY_FR */
}

static void handle_ds_frame(uint32_t address, uint32_t width, uint32_t height, uint32_t mode, uint32_t frame_id)
{
#if !DISPLAY_FR_NDS
    hdlcd_show(address, width, height, mode);
#endif /* DISPLAY_FR_NDS */
}

static void hdlcd_show(uint32_t address, uint32_t width, uint32_t height, uint32_t mode)
{
    struct hdlcd_resolution_cfg_t custom_resolution_cfg;

    enum hdlcd_error_t hdlcd_err = 0;
    enum hdlcd_pixel_format format = HDLCD_PIXEL_FORMAT_RGB565;

    printf("Displaying image...\r\n");
    if (format == HDLCD_PIXEL_FORMAT_NOT_SUPPORTED) {
        printf("Unsupported pixel format: 0x%x\r\n", mode);
        return;
    }

    hdlcd_disable(&HDLCD_DEV);

    // TODO: Choose standard resolution
    // Note: This only works, because FVP ignores all timing values
    custom_resolution_cfg.v_data = height;
    custom_resolution_cfg.h_data = width;
    hdlcd_set_custom_resolution(&HDLCD_DEV, &custom_resolution_cfg);

    struct hdlcd_buffer_cfg_t hdlcd_buff = {.base_address = address,
                                            .line_length = width * HDLCD_MODES[format].bytes_per_pixel,
                                            .line_count = height - 1,
                                            .line_pitch = width * HDLCD_MODES[format].bytes_per_pixel,
                                            .pixel_format = HDLCD_MODES[format].pixel_format};

    hdlcd_err = hdlcd_buffer_config(&HDLCD_DEV, &hdlcd_buff);
    if (hdlcd_err != HDLCD_ERR_NONE) {
        printf("HDLCD buffer config error! \r\n");
        while (1)
            ;
    }
    hdlcd_err = hdlcd_pixel_config(&HDLCD_DEV, HDLCD_MODES[format].pixel_cfg);
    if (hdlcd_err != HDLCD_ERR_NONE) {
        printf("HDLCD pixel config error! \r\n");
        while (1)
            ;
    }

    hdlcd_enable(&HDLCD_DEV);

    printf("Image displayed\r\n");
}
