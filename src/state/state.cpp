#include "state/state.hpp"

TrueState trueState{0,0,0, 0,0,0};
EstState  estState{0,0,0,  0,0,0};
SensorTimestamps sensorTimestamps{0,0,0};
ImuMeasure imuMeas{0,0,0};
GpsMeasure gpsMeas{0,0};
BaroMeasure baroMeas{0};

std::mutex stateMutex{};
std::mutex sensorMutex{};

