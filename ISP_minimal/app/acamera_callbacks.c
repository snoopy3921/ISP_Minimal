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

#include "acamera_logger.h"
#include "acamera_types.h"

#include "runtime_initialization_settings.h"

frame_ready_handler_t ds_frame_ready = NULL;
frame_ready_handler_t fr_frame_ready = NULL;

static uint32_t coherent_dma_allocated_size = 0;
typedef uint8_t frame_buffer_t[MAX_OUTPUT_FRAME_SIZE];
static frame_buffer_t *fr_buffer = (frame_buffer_t *)FR_BUFFER_BASE;
static frame_buffer_t *ds_buffer = (frame_buffer_t *)DS_BUFFER_BASE;

/* TODO: This is for FVP stream optimisation  */
#include "FreeRTOS.h"
#include "semphr.h"
extern SemaphoreHandle_t xStreamSemaphore;

/* Only 1 allocation is supported, it is for the Temper Frame */
void *callback_dma_alloc_coherent(uint32_t ctx_id, uint64_t size, uint64_t *dma_addr)
{
    uint32_t addr;

    if (coherent_dma_allocated_size + size > COHERENT_DMA_MEMORY_SIZE) {
        LOG(LOG_ERR, "Not enough memory");
        *dma_addr = 0;
        return NULL;
    }

    addr = COHERENT_DMA_MEMORY_BASE + coherent_dma_allocated_size;
    coherent_dma_allocated_size += size;

    *dma_addr = addr;

    /* compute bus address */
    *dma_addr -= ISP_SOC_DMA_BUS_OFFSET;

    LOG(LOG_DEBUG,
        "DMA alloc: 0x%x, Memory left: 0x%x",
        (uint32_t)size,
        COHERENT_DMA_MEMORY_SIZE - coherent_dma_allocated_size);
    return (void *)addr;
}

/* Only 1 allocation is supported, it is for the Temper Frame */
void callback_dma_free_coherent(uint32_t ctx_id, uint64_t size, void *virt_addr, uint64_t dma_addr)
{
    if (((uint32_t)virt_addr) != COHERENT_DMA_MEMORY_BASE) {
        LOG(LOG_ERR, "DMA free: Trying to free unknown memory: 0x%x", (uint32_t)virt_addr);
    }
    coherent_dma_allocated_size = 0;
}

int callback_stream_get_frame(uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes)
{
    uint32_t address = 0;

    while (num_planes > 1) {
        num_planes--;
        aframes[num_planes].address = 0;
        aframes[num_planes].status = dma_buf_purge;
    }
    if (type == ACAMERA_STREAM_FR) {
        static uint32_t fr_frame_cntr = 0;
        address = (uint32_t)fr_buffer[fr_frame_cntr % MAX_BUFFERED_FRAMES];
        fr_frame_cntr++;
    } else if (type == ACAMERA_STREAM_DS1) {
        static uint32_t ds_frame_cntr = 0;
        address = (uint32_t)ds_buffer[ds_frame_cntr % MAX_BUFFERED_FRAMES];
        ds_frame_cntr++;
    }
    aframes[0].address = address;
    if (address == 0) {
        LOG(LOG_ERR, "No buffer available!");
        aframes[0].status = dma_buf_purge;
        return -1;
    }
    aframes[0].status = dma_buf_empty;
    return 0;
}

// Return pixel width in bits for format.
static inline uint32_t get_pixel_width(uint32_t format)
{
    uint32_t result = 32;
    switch (format) {
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

int callback_stream_put_frame(uint32_t ctx_id, acamera_stream_type_t type, aframe_t *aframes, uint64_t num_planes)
{
    uint32_t addr, w, h, bpp, format, format_bpp;
    /* Note: We only care about the first plane */
    LOG(LOG_CRIT, "\033[1;33m-- aframes->frame_id = %d --\033[1;0m", aframes->frame_id);
    while (num_planes > 0) {
        num_planes--;
        aframes[num_planes].status = dma_buf_purge;
    }

    if (aframes->address) {
        addr = aframes->address;
        w = aframes->width;
        h = aframes->height;
        bpp = aframes->size / w / h;

        LOG(LOG_CRIT, "\033[1;33m-- %u X %u @ %u bytes per pixel --\033[1;0m", w, h, bpp);
#if 0
        for(uint32_t y = 10; y < 11; y++) {
            for(uint32_t x = 100; x < 110; x++) {
                printf("%08x", *(uint32_t*)(addr + (y * w + x) * 4));
            }
            printf("\r\n");
        }
#endif
    }
    if (type == ACAMERA_STREAM_DS1) {
        acamera_command(ctx_id, TIMAGE, DS1_FORMAT_BASE_PLANE_ID, 0, COMMAND_GET, &format);
        format_bpp = _get_pixel_width(format) / 8;
        if (bpp != format_bpp) {
            /* Current frame and current format mismatch */
            /* Default to 2 bytes = RGB565, otherwise RGB32 */
            format = (bpp == 2) ? DMA_FORMAT_RGB565 : DMA_FORMAT_RGB32;
            LOG(LOG_NOTICE, "Bits per pixel mismatch! Expected: %u. Using format: 0x%x", format_bpp * 8, format);
        }
        if (ds_frame_ready != NULL) {
            ds_frame_ready(addr, w, h, format, aframes->frame_id);
        }

        /* Let stream thread start the next frame */
        xSemaphoreGive(xStreamSemaphore);
    } else if (type == ACAMERA_STREAM_FR) {
        acamera_command(ctx_id, TIMAGE, FR_FORMAT_BASE_PLANE_ID, 0, COMMAND_GET, &format);
        format_bpp = _get_pixel_width(format) / 8;
        if (bpp != format_bpp) {
            format = (bpp == 2) ? DMA_FORMAT_RGB565 : DMA_FORMAT_RGB32;
            LOG(LOG_NOTICE, "Bits per pixel mismatch! Expected: %u. Using format: 0x%x", format_bpp * 8, format);
        }

        if (fr_frame_ready != NULL) {
            fr_frame_ready(addr, w, h, format, aframes->frame_id);
        }
    }
    return 0;
}
