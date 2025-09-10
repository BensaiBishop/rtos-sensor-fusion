#ifndef STATE_HPP
#define STATE_HPP

#include <mutex>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}


// True/simulated vehicle state (written by SensorTask)
struct TrueState {
    float x, y, z;   // position
    float vx, vy, vz; // velocity
};

// Estimated state (written by EstimatorTask)
struct EstState {
    float x, y, z;
    float vx, vy, vz;
};

// Sensor measurements
struct ImuMeasure {
    float vx, vy, vz;  // linear velocity
};

struct GpsMeasure {
    float x, y;  // position 
};

struct BaroMeasure {
    float z;     // depth
};

// Shared sensor timestamps (for watchdog etc.)
struct SensorTimestamps {
    TickType_t imu;
    TickType_t gps;
    TickType_t baro;
};

struct SensorOutput {
    ImuMeasure imuMeas;
    BaroMeasure baroMeas;
    GpsMeasure gpsMeas;
    bool baroUpdated;
    bool gpsUpdated;
};


// Globals
extern TrueState trueState;           // simulation
extern EstState estState;             // filter estimate
extern SensorTimestamps sensorTimestamps;
extern ImuMeasure imuMeas;
extern GpsMeasure gpsMeas;
extern BaroMeasure baroMeas;

// Mutexes
extern std::mutex stateMutex;   // protects trueState and estState
extern std::mutex sensorMutex;  // protects sensorTimestamps

#endif /*STATE_HPP*/

