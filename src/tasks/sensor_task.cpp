#include "tasks/sensor_task.hpp"
#include "state/state.hpp"
#include <iostream>
#include "tasks/dashboard_task.hpp"

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    
}

void SensorTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    TickType_t imuCounter = 0, gpsCounter = 0, baroCounter = 0;

    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5)); // base tick 200Hz

        imuCounter += 5;
        gpsCounter += 5;
        baroCounter += 5;

        // 200hz or 5ms
        if(imuCounter >= 5) {
            imuCounter = 0;
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                sensorTimestamps.imu = xTaskGetTickCount();
            }
            //std::cout << "IMU Update" << std::endl;
            dashImuCounter++;
        }
        // 1hz or 1000ms
        if(gpsCounter >= 1000) {
            gpsCounter = 0;
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                sensorTimestamps.gps = xTaskGetTickCount();
            }
            //std::cout << "GPS Update" << std::endl;
            dashGpsCounter++;
        }
        // 10hz or 100ms
        if(baroCounter >= 100) {
            baroCounter = 0;
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                sensorTimestamps.baro = xTaskGetTickCount();
            }
            //std::cout << "Barometer Update" << std::endl;
            dashBaroCounter++;
        }
    }
}