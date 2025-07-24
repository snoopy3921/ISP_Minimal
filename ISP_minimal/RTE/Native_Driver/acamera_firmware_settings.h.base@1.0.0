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

#ifndef __ACAMERA_FIRMWARE_SETTNGS_H__
#define __ACAMERA_FIRMWARE_SETTNGS_H__


#include "acamera_sensor_api.h"
#include "acamera_lens_api.h"

// Formats which are supported on output of ISP.
// They can be used to set a desired output format
// in acamera_settings structure.
#define DMA_FORMAT_DISABLE      0
#define DMA_FORMAT_RGB32        1
#define DMA_FORMAT_A2R10G10B10  2
#define DMA_FORMAT_RGB565       3
#define DMA_FORMAT_RGB24        4
#define DMA_FORMAT_GEN32        5
#define DMA_FORMAT_RAW16        6
#define DMA_FORMAT_RAW12        7
#define DMA_FORMAT_AYUV         8
#define DMA_FORMAT_Y410         9
#define DMA_FORMAT_YUY2         10
#define DMA_FORMAT_UYVY         11
#define DMA_FORMAT_Y210         12
#define DMA_FORMAT_NV12_Y       13
#define DMA_FORMAT_NV12_UV      ( 1 << 6 | 13 )
#define DMA_FORMAT_NV12_VU      ( 2 << 6 | 13 )
#define DMA_FORMAT_YV12_Y       14
#define DMA_FORMAT_YV12_U       ( 1 << 6 | 14 )
#define DMA_FORMAT_YV12_V       ( 2 << 6 | 14 )

// Return pixel width in bits for format.
static inline uint32_t _get_pixel_width( uint32_t format )
{
  uint32_t result = 32;
  switch ( format ) {
  case DMA_FORMAT_RGB24:
    result = 24;
    break;
  case DMA_FORMAT_RGB565:
  case DMA_FORMAT_RAW16:
  case DMA_FORMAT_YUY2:
  case DMA_FORMAT_UYVY:
    result = 16;
    break;
  case DMA_FORMAT_RAW12:
    result = 12;
    break;
  case DMA_FORMAT_NV12_Y:
  case DMA_FORMAT_NV12_UV:
  case DMA_FORMAT_NV12_VU:
  case DMA_FORMAT_YV12_Y:
  case DMA_FORMAT_YV12_U:
  case DMA_FORMAT_YV12_V:
    result = 8;
    break;
  }
  return result;
}

// Calibration table
typedef struct _ACameraCalibrations {
    LookupTable *calibrations[CALIBRATION_TOTAL_SIZE];
} ACameraCalibrations;

typedef struct _aframe_t {
    uint32_t type;        // Frame type
    uint32_t width;       // Frame width
    uint32_t height;      // Frame height
    uint32_t address;     // Start of memory block
    uint32_t line_offset; // Line offset for the frame
    uint32_t size;        // Total size of the memory in bytes
    uint32_t status;
    void *virt_addr;      // Virt address accessed by CPU
    uint32_t frame_id;
} aframe_t;

typedef struct _tframe_t {
    aframe_t primary;     // Primary frames
    aframe_t secondary;   // Secondary frames
} tframe_t;

// This structure is used by firmware to return the information
// about an output frame.
typedef struct _metadata_t {
    uint8_t format;
    uint16_t width;
    uint16_t height;
    uint32_t frame_number;
    uint16_t line_size;
    uint32_t exposure;
    uint32_t int_time;
    uint32_t int_time_medium;
    uint32_t int_time_long;
    uint32_t again;
    uint32_t dgain;
    uint32_t addr;
    uint32_t isp_dgain;
    int8_t dis_offset_x;
    int8_t dis_offset_y;
    uint32_t frame_id;
} metadata_t;


typedef struct _acamera_settings {
    void (*sensor_init)( void **ctx, sensor_control_t *ctrl); // Must be initialized to provide sensor initialization entry.
    void (*sensor_deinit)( void *ctx );                       // Must be initialized to provide sensor initialization entry.
    int32_t (*lens_init)( void **ctx, lens_control_t *ctrl);  // Initialize lens driver for AF. May be NULL. Must return 0 on success and -1 on fail.
    void (*lens_deinit)( void *ctx );                         // Initialize lens driver for AF. May be NULL. Must return 0 on success and -1 on fail.
    uint32_t (*get_calibrations)( uint32_t ctx_num, void *sensor_arg, ACameraCalibrations *); // Must be initialized to provide calibrations.
    uintptr_t isp_base;                                       // ISP base offset (not absolute memory address ). Should be started from start of ISP memory. All ISP r/w accesses inside the firmware will use this value as the start_offset.
    uint32_t hw_isp_addr;                                     // Hardware ISP register configuration address.
    void *(*callback_dma_alloc_coherent)( uint32_t ctx_id, uint64_t size, uint64_t *dma_addr );
    void (*callback_dma_free_coherent)( uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr );
    int (*callback_stream_get_frame)( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes );
    int (*callback_stream_put_frame)( uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes );
} acamera_settings;

#endif
