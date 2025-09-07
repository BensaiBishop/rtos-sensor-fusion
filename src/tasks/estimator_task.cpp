#include "tasks/estimator_task.hpp"
#include "state/state.hpp"
#include "tasks/dashboard_task.hpp"
#include <iostream>
extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

void EstimatorTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10)); // 100Hz update

        std::lock_guard<std::mutex> lock(stateMutex);
        // Simple integration 
        vehicleState.x += vehicleState.vx * 0.01f;
        vehicleState.y += vehicleState.vy * 0.01f;
        vehicleState.z += vehicleState.vz * 0.01f;

        //std::cout << "Estimator updated state" << std::endl;
        dashEstimateCounter++;
    }
}