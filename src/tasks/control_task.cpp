#include "tasks/control_task.hpp"
#include <iostream>
#include "state/state.hpp"
#include "tasks/dashboard_task.hpp"

extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

void ControlTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20)); // 50Hz control

        std::lock_guard<std::mutex> lock(stateMutex);
        // control logic
        vehicleState.vx += 0.1f;
        vehicleState.vy += 0.1f;
        vehicleState.vz += 0.1f;

        //std::cout << "Control Update" << std::endl;
        dashControlCounter++;
    }
}
