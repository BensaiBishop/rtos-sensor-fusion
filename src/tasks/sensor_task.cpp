#include "tasks/sensor_task.hpp"
#include "state/state.hpp"
#include "tasks/dashboard_task.hpp"
#include <cmath>
#include <cstdlib>
#define M_PI 3.14159265358979323846

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
    
}

// Parameters for sine wave motion
struct TrajectoryParams {
    float amplitudeX;
    float amplitudeY;
    float amplitudeZ;
    float freqX;
    float freqY;
    float freqZ;
};
TrajectoryParams traj{10.0f, 5.0f, 2.0f, 0.05f, 0.1f, 0.2f};

// Simple noise generator
inline float noisy(float val, float noiseStd) {
    float r = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // uniform [-1,1]
    return val + r * noiseStd;
}

void SensorTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();
    float t = 0.0f; //time
    const float dt = 0.005f; // 200hz

    int baroCounter = 0;
    int gpsCounter = 0;

    while (true) {
        xTaskDelayUntil (&lastWake, pdMS_TO_TICKS(5)); //200 Hz
        t += dt;

        //compute the sine motion for the uuv
        TrueState newTrue;
        newTrue.x = traj.amplitudeX * sinf(2.0f * M_PI * traj.freqX * t);
        newTrue.y = traj.amplitudeY * sinf(2.0f * M_PI * traj.freqY * t);
        newTrue.z = traj.amplitudeZ * sinf(2.0f * M_PI * traj.freqZ * t);

        newTrue.vx = 2.0f * M_PI * traj.freqX * traj.amplitudeX * cosf(2.0 * M_PI * traj.freqX * t);
        newTrue.vy = 2.0f * M_PI * traj.freqY * traj.amplitudeY * cosf(2.0f * M_PI * traj.freqY * t);
        newTrue.vz = 2.0f * M_PI * traj.freqZ * traj.amplitudeZ * cosf(2.0f * M_PI * traj.freqZ * t);

        {
            std::lock_guard<std::mutex> lock(stateMutex);
            trueState = newTrue; 
        }

        //What the sensor is receiving
        //imu 200Hz
        {
            std::lock_guard<std::mutex> lock(sensorMutex);
            imuMeas.vx = noisy(newTrue.vx, 0.05f);
            imuMeas.vy = noisy(newTrue.vy, 0.05f);
            imuMeas.vz = noisy(newTrue.vz, 0.05f);
        }
        dashImuCounter++;

        //baro 10Hz
        if (++baroCounter >= 20) {
            {
                std::lock_guard<std::mutex> lock(sensorMutex);
                baroMeas.z = noisy(newTrue.x, 0.05f);
            }
            dashBaroCounter++;
            baroCounter = 0;
        }

        //gps 1Hz
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
