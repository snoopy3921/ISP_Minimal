/*
 * Copyright (c) 2022-2025 Arm Limited
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

#ifndef CMSIS_device_header
/* CMSIS pack default header, containing the CMSIS_device_header definition */
#include "RTE_Components.h"
#endif
#include CMSIS_device_header

#include "FreeRTOS.h"
#include "task.h"

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

extern void vStartISPDemo();
extern int stdout_init();

int main(void)
{

    __enable_irq();

    stdout_init();

    NVIC_SetPriority(ISP_IRQn, 2);
    NVIC_SetPriority(HDLCD_IRQn, 2);
    NVIC_SetPriority(UARTTX0_IRQn, 0);
    NVIC_SetPriority(UARTRX1_IRQn, 1);
    NVIC_SetPriority(UARTTX1_IRQn, 2);

    vStartISPDemo();

    /* Start the scheduler itself. */
    vTaskStartScheduler();

    while (1) {
    }
}
