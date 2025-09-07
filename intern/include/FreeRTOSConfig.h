#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions for POSIX/Linux port.
 *----------------------------------------------------------*/

/* Preemption model */
#define configUSE_PREEMPTION            1
#define configUSE_TIME_SLICING          1

/* Tick rate: POSIX sim can use higher ticks without issue */
#define configUSE_16_BIT_TICKS          0   // POSIX => 32-bit tick
#define configCPU_CLOCK_HZ              (1)
#define configTICK_RATE_HZ              (1000)

/* Task priorities and stack sizes */
#define configMAX_PRIORITIES            (7)
#define configMINIMAL_STACK_SIZE        (512)   /* bytes in POSIX sim */
#define configTOTAL_HEAP_SIZE           (128*1024)

/* Memory allocation */
#define configSUPPORT_STATIC_ALLOCATION 0
#define configSUPPORT_DYNAMIC_ALLOCATION 1

/* Hook functions (turn on malloc fail hook for debug) */
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0
#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_MALLOC_FAILED_HOOK    1

/* Run time and stats */
#define configGENERATE_RUN_TIME_STATS   0
#define configUSE_TRACE_FACILITY        0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* Timers */
#define configUSE_TIMERS                1
#define configTIMER_TASK_PRIORITY       (configMAX_PRIORITIES-1)
#define configTIMER_QUEUE_LENGTH        10
#define configTIMER_TASK_STACK_DEPTH    2048

/* Assertions */
#include <assert.h>
#define configASSERT(x)                 assert(x)

/* Optional API functions */
#define INCLUDE_vTaskPrioritySet        1
#define INCLUDE_uxTaskPriorityGet       1
#define INCLUDE_vTaskDelete             1
#define INCLUDE_vTaskSuspend            1
#define INCLUDE_vTaskDelayUntil         1
#define INCLUDE_vTaskDelay              1
#define INCLUDE_xTaskGetSchedulerState  1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskGetStackHighWaterMark 1

#endif /* FREERTOS_CONFIG_H */
