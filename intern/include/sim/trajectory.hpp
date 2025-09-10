#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP
#include <cmath>
#include "state/state.hpp"

// Parameters for sine wave motion
struct TrajectoryParams {
    float amplitudeX;
    float amplitudeY;
    float amplitudeZ;
    float freqX;
    float freqY;
    float freqZ;
};

// Default parameters
extern TrajectoryParams traj;

// Generate the "true" vehicle state at time t
TrueState generateTrajectory(float t);

#endif /* TRAJECTORY_HPP */
