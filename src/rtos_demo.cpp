extern "C" 
{
#include "FreeRTOS.h"
#include "task.h"
}
#include "rtos_demo.hpp"
#include <iostream>


void task1(void* pvParameters) {
    while (true) {
        std::cout << "Task 1 is running" << std::endl << std::flush;
        vTaskDelay(pdMS_TO_TICKS(1000)); // 1 second
    }
}

void task2(void* pvParameters) {
    while (true) {
        std::cout << "Task 2 is running" << std::endl << std::flush;
        vTaskDelay(pdMS_TO_TICKS(1500)); // 1.5 second
    }
}

extern "C"{
    /* Called if a task overflows its stack */
    void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
        std::cerr << "Stack overflow in task: " << pcTaskName << std::endl;
        std::abort(); // stop program so debugger can catch it
    }

    /* Called if malloc fails inside FreeRTOS */
    void vApplicationMallocFailedHook(void) {
        std::cerr << "FreeRTOS malloc failed!" << std::endl;
        std::abort();
    }

    /* Called on each idle tick (can be empty) */
    void vApplicationIdleHook(void) {

    }
}




