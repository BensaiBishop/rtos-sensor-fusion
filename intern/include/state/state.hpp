#ifndef STATE_HPP
#define STATE_HPP

#include <mutex>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

// Vehicle state
struct VehicleState
{
    float x, y, z;
    float vx, vy, vz;
};

// Shared sensor timestamps
struct SensorTimestamps {
    TickType_t imu;
    TickType_t gps;
    TickType_t baro;
};

// Global shared objects
extern VehicleState vehicleState;
extern SensorTimestamps sensorTimestamps;
extern std::mutex stateMutex;
extern std::mutex sensorMutex;

#endif /*STATE_HPP*/

