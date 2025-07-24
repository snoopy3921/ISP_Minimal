/*
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2024-2025, Arm Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or
 *    other materials provided with the distribution.
 * - Neither the name of ARM nor the names of its contributors may be used to
 *    endorse or promote products derived from this software without specific prior
 *    written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "acamera_command_api.h"
#include "acamera_firmware_api.h"
#include "acamera_firmware_config.h"
#include "acamera_interface_config.h"
#include "acamera_isp_config.h"
#include "acamera_logger.h"
#include "acamera_types.h"

#include "system_cdma.h"
#include "system_cdma_platform.h"
#include "system_interrupts.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "runtime_initialization_settings.h"
#include <stdint.h>

#define ENABLE_TPG_AT_START           0
#define ISP_VIRTUAL_CAMERA_SEND_FRAME 0x1

static acamera_settings settings[FIRMWARE_CONTEXT_NUMBER] = {
    {
        .sensor_init = fvp_sensor_init,
        .sensor_deinit = fvp_sensor_deinit,
        .get_calibrations = get_calibrations_dummy,
        .lens_init = NULL,
        .lens_deinit = NULL,
        .isp_base = 0x0,
        .hw_isp_addr = ISP_SOC_START_ADDR,
        .callback_dma_alloc_coherent = callback_dma_alloc_coherent,
        .callback_dma_free_coherent = callback_dma_free_coherent,
        .callback_stream_get_frame = callback_stream_get_frame,
        .callback_stream_put_frame = callback_stream_put_frame,
    },

};

// This example will run the infinite loop to process the firmware events.
// This variable also can be changed outside to stop the processing.
volatile int32_t acamera_main_loop_active = 1;

/* Stream semaphore is used to block stream thread until previous frame is displayed */
SemaphoreHandle_t xStreamSemaphore;
StaticSemaphore_t xStreamSemaphoreBuffer;

static void stream_control_thread(void *pvParameters);
static void acamera_thread(void *pvParameters);
extern uint32_t acamera_get_api_context(void);

// this is a main application IRQ handler to drive the firmware
// The main purpose is to redirect ISP irq event to the firmware core
// Please see the ACamera Porting Guide for details.
static void interrupt_handler(void *ptr, uint32_t mask)
{
    // the lower bits are for ISP interrupts on ACamera FPGA reference platform
    uint32_t isp_mask = mask & 0x0000FFFF;

    // tell the firmware that isp interrupts happened
    if (isp_mask) {
        // the first irq pins are connected to ISP
        acamera_interrupt_handler();
    }
}

static void create_tasks()
{
    xTaskCreate(acamera_thread,
                "acamera",
                ACAMERA_THREAD_STACK_SIZE,
                NULL,
                (configMAX_PRIORITIES - 2) | portPRIVILEGE_BIT,
                NULL);

    xStreamSemaphore = xSemaphoreCreateBinaryStatic(&xStreamSemaphoreBuffer);
    xTaskCreate(stream_control_thread,
                "stream",
                STREAM_CTRL_THREAD_STACK_SIZE,
                NULL,
                (configMAX_PRIORITIES - 3) | portPRIVILEGE_BIT,
                NULL);
}

static int isp_initial_config()
{
    int32_t result;
    uint32_t rc;
    uint32_t ctx_num;
    uint32_t prev_ctx_num = 0;

    // The firmware supports multicontext.
    // It means that the customer can use the same firmware for controlling
    // several instances of different sensors/isp. To initialise a context
    // the structure acamera_settings must be filled properly.
    // the total number of initialized context must not exceed FIRMWARE_CONTEXT_NUMBER
    // all contexts are numerated from 0 till ctx_number - 1
    result = acamera_init(settings, FIRMWARE_CONTEXT_NUMBER);
    if (result != 0) {
        LOG(LOG_ERR, "Failed to start firmware processing thread. (0x%x)", result);
        return result;
    }

    uint32_t ctx_id = acamera_get_api_context();
    acamera_command(ctx_id, TGENERAL, ACTIVE_CONTEXT, 0, COMMAND_GET, &prev_ctx_num);

    system_interrupt_set_handler(interrupt_handler, NULL);

    for (ctx_num = 0; ctx_num < FIRMWARE_CONTEXT_NUMBER; ctx_num++) {
        acamera_command(ctx_id, TGENERAL, ACTIVE_CONTEXT, ctx_num, COMMAND_SET, &rc);

        /* Disable most ISP calibrations. FVP sensor streams processed RGB images which does not need calibration  */
        acamera_command(ctx_id, TALGORITHMS, AE_MODE_ID, AE_FULL_MANUAL, COMMAND_SET, &rc);
        acamera_command(ctx_id, TALGORITHMS, AWB_MODE_ID, AWB_MANUAL, COMMAND_SET, &rc);

        acamera_command(ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_TEMPER, ON, COMMAND_SET, &rc);
        acamera_command(ctx_id, TISP_MODULES, ISP_MODULES_MANUAL_IRIDIX, ON, COMMAND_SET, &rc);

        acamera_command(ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE, ON, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_MANUAL_EXPOSURE_RATIO, ON, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_MANUAL_ISP_DIGITAL_GAIN, ON, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_MANUAL_AWB, ON, COMMAND_SET, &rc);

#define AWB_GAIN 9
        acamera_command(ctx_id, TSYSTEM, SYSTEM_AWB_BLUE_GAIN, AWB_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_AWB_GREEN_EVEN_GAIN, AWB_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_AWB_GREEN_ODD_GAIN, AWB_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_AWB_RED_GAIN, AWB_GAIN, COMMAND_SET, &rc);
#define CCM_GAIN 4095
        acamera_command(ctx_id, TSYSTEM, SYSTEM_MANUAL_CCM, ON, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_RR, CCM_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_GG, CCM_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_BB, CCM_GAIN, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_RG, 0, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_RB, 0, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_GR, 0, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_GB, 0, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_BR, 0, COMMAND_SET, &rc);
        acamera_command(ctx_id, TSYSTEM, SYSTEM_CCM_MATRIX_BG, 0, COMMAND_SET, &rc);

        acamera_command(ctx_id, TSYSTEM, TEST_PATTERN_MODE_ID, 5, COMMAND_SET, &rc);

#if ENABLE_TPG_AT_START
        acamera_command(ctx_id, TSYSTEM, TEST_PATTERN_ENABLE_ID, ON, COMMAND_SET, &rc);
#endif

        acamera_command(ctx_id, TSCENE_MODES, SHARPENING_STRENGTH_ID, 0, COMMAND_SET, &rc);

        acamera_command(ctx_id, TIMAGE, FR_FORMAT_BASE_PLANE_ID, DMA_FORMAT_RGB565, COMMAND_SET, &rc);
        acamera_command(ctx_id, TIMAGE, DS1_FORMAT_BASE_PLANE_ID, DMA_FORMAT_RGB565, COMMAND_SET, &rc);
    }
    acamera_command(ctx_id, TGENERAL, ACTIVE_CONTEXT, prev_ctx_num, COMMAND_SET, &rc);

    return 0;
}

int isp_init()
{
    system_cdma_setup();
    create_tasks();
    return 0;
}

static void stream_control_thread(void *pvParameters)
{
    uint32_t frame_cntr = 0;
    while (1) {
        if (STREAM_ENABLED) {
            LOG(LOG_CRIT, "\033[1;34m-- CAMERA STREAM TRIGGER #%d --\033[1;0m", frame_cntr);
            frame_cntr++;
            *((uint8_t *)(ISP_VIRTUAL_CAMERA_BASE_NS)) = ISP_VIRTUAL_CAMERA_SEND_FRAME;

            /* Displaying a frame gives the semaphore. If previous frame is not yet
             * displayed, or no frames have been displayed, task waits 100 ticks. */
            xSemaphoreTake(xStreamSemaphore, 100);
        }
        vTaskDelay(5);
    }
}

static void acamera_thread(void *pvParameters)
{
    static uint32_t DO_INITIAL_SETUP = 1;
    uint32_t rc = 0;

    if (isp_initial_config()) {
        acamera_main_loop_active = 0;
    }
    // acamera_process function must be called on every incoming interrupt
    // to give the firmware the possibility to apply
    // all internal algorithms and change the ISP state.
    // The external application can be run in the same loop on bare metal systems.
    while (acamera_main_loop_active) {
        // acamera_process must be called for each initialised context
        acamera_process();

        // This needs to be after first acamera_process(); because for the first time, it resets the FSMs and
        // Crop FSM resets the scaler width/height values
        if (DO_INITIAL_SETUP) {
            DO_INITIAL_SETUP = 0;
            uint32_t ctx_id = acamera_get_api_context();
            acamera_command(ctx_id, TIMAGE, IMAGE_RESIZE_TYPE_ID, SCALER_DS, COMMAND_SET, &rc);
            acamera_command(ctx_id, TIMAGE, IMAGE_RESIZE_WIDTH_ID, 192, COMMAND_SET, &rc);
            acamera_command(ctx_id, TIMAGE, IMAGE_RESIZE_HEIGHT_ID, 192, COMMAND_SET, &rc);
            acamera_command(ctx_id, TIMAGE, IMAGE_RESIZE_ENABLE_ID, RUN, COMMAND_SET, &rc);

            acamera_command(ctx_id, TSENSOR, SENSOR_STREAMING, ON, COMMAND_SET, &rc);
        }
        taskYIELD();
    }

    // this api function will free
    // all resources allocated by the firmware
    acamera_terminate();

    LOG(LOG_CRIT, "Acamera terminated");

    for (;;)
        ; // Task shouldn't exit

    return;
}
