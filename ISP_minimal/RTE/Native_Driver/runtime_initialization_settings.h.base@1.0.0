/*
*
* SPDX-License-Identifier: BSD-3-Clause
*
* Copyright (c) 2016-2022, Arm Limited. All rights reserved.
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

#include "acamera_types.h"
#include "acamera_firmware_config.h"
#include "acamera_firmware_api.h"
#include "acamera_sensor_api.h"
#include "acamera_lens_api.h"



extern void sensor_init_dummy( void **ctx, sensor_control_t *);
extern void sensor_deinit_dummy( void *ctx );
extern uint32_t get_calibrations_dummy( uint32_t ctx_num, void *sensor_arg, ACameraCalibrations *);

#if FIRMWARE_CONTEXT_NUMBER == 2


extern void sensor_init_dummy( void **ctx, sensor_control_t *);
extern void sensor_deinit_dummy( void *ctx );
extern uint32_t get_calibrations_dummy( uint32_t ctx_num, void *sensor_arg, ACameraCalibrations *);

#endif // FIRMWARE_CONTEXT_NUMBER == 2

extern int32_t lens_init( void **ctx, lens_control_t *ctrl );
extern void lens_deinit( void *ctx );



extern void *callback_dma_alloc_coherent( uint32_t ctx_id, uint64_t size, uint64_t *dma_addr );
extern void callback_dma_free_coherent( uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr );

extern int callback_stream_get_frame( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes );
extern int callback_stream_put_frame( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes );

static acamera_settings settings[ FIRMWARE_CONTEXT_NUMBER ] = {    {
        .sensor_init = sensor_init_dummy,
        .sensor_deinit = sensor_deinit_dummy,
        .get_calibrations = get_calibrations_dummy,
        .lens_init = lens_init,
        .lens_deinit = lens_deinit,
        .isp_base = 0x0,
        .hw_isp_addr = 0x0,
        .callback_dma_alloc_coherent = callback_dma_alloc_coherent,
        .callback_dma_free_coherent = callback_dma_free_coherent,
        .callback_stream_get_frame = callback_stream_get_frame,
        .callback_stream_put_frame = callback_stream_put_frame,
    },

#if FIRMWARE_CONTEXT_NUMBER == 2
    {
        .sensor_init = sensor_init_dummy,
        .sensor_deinit = sensor_deinit_dummy,
        .get_calibrations = get_calibrations_dummy,
        .lens_init = lens_init,
        .lens_deinit = lens_deinit,
        .isp_base = 0x0,
        .hw_isp_addr = 0x0,
        .callback_dma_alloc_coherent = callback_dma_alloc_coherent,
        .callback_dma_free_coherent = callback_dma_free_coherent,
        .callback_stream_get_frame = callback_stream_get_frame,
        .callback_stream_put_frame = callback_stream_put_frame,
    },

#endif // FIRMWARE_CONTEXT_NUMBER == 2
};
