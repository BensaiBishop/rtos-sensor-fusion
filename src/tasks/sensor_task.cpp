#include "tasks/sensor_task.hpp"
#include "state/state.hpp"
#include "sim/trajectory.hpp"
#include "sim/sensor_models.hpp"
#include "tasks/dashboard_task.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

void SensorTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    float t = 0.0f;
    const float dt = 0.005f; // 200Hz

    int baroCounter = 0;
    int gpsCounter = 0;

    while (true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(5)); // 200 Hz
        t += dt;

        // Generate trajectory
        TrueState newTrue = generateTrajectory(t);

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            trueState = newTrue;
        }

        // IMU 200 Hz
        {
            std::lock_guard<std::mutex> lock(sensorMutex);
            imuMeas.vx = noisy(newTrue.vx, 0.05f);
            imuMeas.vy = noisy(newTrue.vy, 0.05f);
            imuMeas.vz = noisy(newTrue.vz, 0.05f);
        }
        dashImuCounter++;

        // Baro 10 Hz
        if (++baroCounter >= 20) {
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                baroMeas.z = noisy(newTrue.z, 0.05f);
            }
            dashBaroCounter++;
            baroCounter = 0;
        }

        // GPS 1 Hz
        if (++gpsCounter >= 200) {
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                gpsMeas.x = noisy(newTrue.x, 0.05f);
                gpsMeas.y = noisy(newTrue.y, 0.05f);
            }
            dashGpsCounter++;
            gpsCounter = 0;
        }
    }
}
