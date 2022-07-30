/*
 * This file is part of the ct-Bot motor controller firmware.
 * Copyright (c) 2021-2022 Timo Sandmann
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    support.c
 * @brief   FreeRTOS support implementations for RP2040 SMP kernel
 * @author  Timo Sandmann
 * @date    30.07.2022
 */

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/uart.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>


#if (configSUPPORT_PICO_SYNC_INTEROP != 1)
#error "configSUPPORT_PICO_SYNC_INTEROP must be 1"
#endif

#if (LIB_PICO_SYNC_MUTEX != 1)
#error "LIB_PICO_SYNC_MUTEX must be 1"
#endif


void* pvPortMalloc(size_t size) {
    void* p_mem = malloc(size);

    configASSERT(p_mem);

    return p_mem;
}

void vPortFree(void* p_mem) {
    free(p_mem);
}

#if (configCHECK_FOR_STACK_OVERFLOW > 0)
void vApplicationStackOverflowHook(TaskHandle_t task, char* task_name) {
    static char taskname[configMAX_TASK_NAME_LEN + 1];

    (void) task;

    memcpy(taskname, task_name, configMAX_TASK_NAME_LEN);
    panic("STACK OVERFLOW: %s\n", taskname);
}
#endif // configCHECK_FOR_STACK_OVERFLOW
