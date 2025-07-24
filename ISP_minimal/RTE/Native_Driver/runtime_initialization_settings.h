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

#ifndef CMSIS_device_header
/* CMSIS pack default header, containing the CMSIS_device_header definition */
#include "RTE_Components.h"
#endif
#include CMSIS_device_header

#include "acamera_firmware_api.h"
#include "acamera_firmware_config.h"
#include "acamera_lens_api.h"
#include "acamera_sensor_api.h"
#include "acamera_types.h"

#define CONNECTION_THREAD_STACK_SIZE  (configMINIMAL_STACK_SIZE * 2)
#define ACAMERA_THREAD_STACK_SIZE     (configMINIMAL_STACK_SIZE * 6)
#define STREAM_CTRL_THREAD_STACK_SIZE (configMINIMAL_STACK_SIZE)

#define COHERENT_DMA_MEMORY_BASE DDR4_BLK1_BASE_S

#define MAX_FRAME_WIDTH       2048
#define MAX_FRAME_HEIGHT      1080
#define MAX_BITS_PER_PIXEL    20
#define MAX_BYTES_PER_PIXEL   4
#define MAX_INPUT_FRAME_SIZE  (MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * MAX_BITS_PER_PIXEL / 8)
#define MAX_OUTPUT_FRAME_SIZE (MAX_FRAME_WIDTH * MAX_FRAME_HEIGHT * MAX_BYTES_PER_PIXEL)

/* 5400KB */
#define COHERENT_DMA_MEMORY_SIZE MAX_INPUT_FRAME_SIZE

#define MAX_BUFFERED_FRAMES 4
#define FR_BUFFER_BASE      (COHERENT_DMA_MEMORY_BASE + COHERENT_DMA_MEMORY_SIZE)
#define DS_BUFFER_BASE      (FR_BUFFER_BASE + (MAX_OUTPUT_FRAME_SIZE * MAX_BUFFERED_FRAMES))

typedef void (*frame_ready_handler_t)(
    uint32_t address, uint32_t width, uint32_t height, uint32_t mode, uint32_t frame_id);
extern frame_ready_handler_t ds_frame_ready, fr_frame_ready;

extern void fvp_sensor_init(void **ctx, sensor_control_t *);
extern void fvp_sensor_deinit(void *ctx);
extern uint32_t get_calibrations_dummy(uint32_t ctx_num, void *sensor_arg, ACameraCalibrations *);

extern void *callback_dma_alloc_coherent(uint32_t ctx_id, uint64_t size, uint64_t *dma_addr);
extern void callback_dma_free_coherent(uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr);

// The ISP pipeline can have several outputs such as Full Resolution, DownScaler1, DownScaler2 etc
// It is possible to set up the firmware to return the metadata for each output frame from
// the specific channel. This callbacks must be set in acamera_settings structure and passed to the firmware in
// acamera_init api function
// The context id can be used to differentiate contexts
extern int
callback_stream_get_frame(uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes);
extern int
callback_stream_put_frame(uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes);
