/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifndef configNUMBER_OF_CORES
    #define configNUMBER_OF_CORES 2
#endif
#ifndef configUSE_CORE_AFFINITY
    #define configUSE_CORE_AFFINITY 1
#endif
#ifndef configRUN_MULTIPLE_PRIORITIES
    #define configRUN_MULTIPLE_PRIORITIES 1
#endif

/* Scheduler Related */
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 1
#define configUSE_PASSIVE_IDLE_HOOK 1
#define configUSE_TICK_HOOK 1
#define configCPU_CLOCK_HZ ((unsigned long)F_CPU)
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES (8)
#define configMINIMAL_STACK_SIZE ((configSTACK_DEPTH_TYPE)256)
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1

/* Synchronization Related */
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 1
#define configUSE_APPLICATION_TASK_TAG 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configQUEUE_REGISTRY_SIZE 8
#define configUSE_QUEUE_SETS 1
#define configUSE_NEWLIB_REENTRANT 1

/* System */
#define configSTACK_DEPTH_TYPE uint32_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE ((size_t)(164 * 1024))

/* Hook function related definitions. */
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define configUSE_MALLOC_FAILED_HOOK 1

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS 1
#define configUSE_TRACE_FACILITY 1
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES 0
#define configMAX_CO_ROUTINE_PRIORITIES (2)

/* Software timer related definitions. */
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (2)
#define configTIMER_QUEUE_LENGTH 5
#define configTIMER_TASK_STACK_DEPTH (1024)

/* Interrupt nesting behaviour configuration. */
/*  Interrupt priorities used by the kernel port layer itself.  These are generic
    to all Cortex-M ports, and do not rely on any particular library functions. */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
    See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#ifdef PICO_RP2350
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY 16
#else
    #define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#endif

#ifndef configMAX_TASK_NAME_LEN
    #define configMAX_TASK_NAME_LEN (32)
#endif

/* Run time stats related definitions. */

#ifdef configGENERATE_RUN_TIME_STATS
extern void vMainConfigureTimerForRunTimeStats(void);
extern unsigned long ulMainGetRunTimeCounterValue(void);
    #define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS()  // vMainConfigureTimerForRunTimeStats()
    #define portGET_RUN_TIME_COUNTER_VALUE() ulMainGetRunTimeCounterValue()
#endif

/* Cortex-M specific definitions. */
#ifndef configPRIO_BITS
    #undef __NVIC_PRIO_BITS
    #ifdef __NVIC_PRIO_BITS
        /* __BVIC_PRIO_BITS will be specified when CMSIS is being used. */
        #define configPRIO_BITS __NVIC_PRIO_BITS
    #else
        #define configPRIO_BITS 3 /* 8 priority levels */
    #endif
#endif

#define configENABLE_MPU 0
#define configENABLE_TRUSTZONE 0
#define configRUN_FREERTOS_SECURE_ONLY 1
#define configENABLE_FPU 1
/*  The lowest interrupt priority that can be used in a call to a "set priority"
    function. */
#ifndef configLIBRARY_LOWEST_INTERRUPT_PRIORITY
    #define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0x7
#endif

/*  The highest interrupt priority that can be used by any interrupt service
    routine that makes calls to interrupt safe FreeRTOS API functions.  DO NOT CALL
    INTERRUPT SAFE FREERTOS API FUNCTIONS FROM ANY INTERRUPT THAT HAS A HIGHER
    PRIORITY THAN THIS! (higher priorities are lower numeric values. */
#ifndef configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
    #define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#endif

#ifndef configASSERT
    #ifdef __cplusplus
extern "C" {
    #endif
void rtosFatalError(void);
    #ifdef __cplusplus
};
    #endif

    #define configASSERT(x)           \
        if((x) == 0) {                \
            portDISABLE_INTERRUPTS(); \
            rtosFatalError();         \
        }
#endif

#ifndef configUSE_DYNAMIC_EXCEPTION_HANDLERS
    #define configUSE_DYNAMIC_EXCEPTION_HANDLERS 0
#endif
#ifndef configSUPPORT_PICO_SYNC_INTEROP
    #define configSUPPORT_PICO_SYNC_INTEROP 1
#endif
#ifndef configSUPPORT_PICO_TIME_INTEROP
    #define configSUPPORT_PICO_TIME_INTEROP 1
#endif

#ifndef LIB_PICO_MULTICORE
    #define LIB_PICO_MULTICORE 1
#endif

#include "rp2040_config.h"
#endif  // FREERTOS_CONFIG_H