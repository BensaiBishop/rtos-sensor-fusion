#include "state/state.hpp"


VehicleState vehicleState{};
SensorTimestamps sensorTimestamps{};
std::mutex stateMutex{};
std::mutex sensorMutex{};
