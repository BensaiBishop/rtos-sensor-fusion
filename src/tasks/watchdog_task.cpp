#include "tasks/watchdog_task.hpp"
#include <iostream>
#include "state/state.hpp"
extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

void WatchdogTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(100)); // 10Hz watchdog

        TickType_t now = xTaskGetTickCount();

        std::lock_guard<std::mutex> lock(sensorMutex);

        if(now - sensorTimestamps.imu > pdMS_TO_TICKS(20))
            std::cerr << "[Watchdog] IMU missed update!" << std::endl;
        if(now - sensorTimestamps.gps > pdMS_TO_TICKS(2000))
            std::cerr << "[Watchdog] GPS missed update!" << std::endl;
        if(now - sensorTimestamps.baro > pdMS_TO_TICKS(200))
            std::cerr << "[Watchdog] Barometer missed update!" << std::endl;
    }
}