#include "sim/trajectory.hpp"
#include <cmath>

#define M_PI 3.14159265358979323846

TrajectoryParams traj{10.0f, 5.0f, 2.0f, 0.05f, 0.1f, 0.2f};

TrueState generateTrajectory(float t) {
    TrueState s;

    s.x = traj.amplitudeX * sinf(2.0f * M_PI * traj.freqX * t);
    s.y = traj.amplitudeY * sinf(2.0f * M_PI * traj.freqY * t);
    s.z = traj.amplitudeZ * sinf(2.0f * M_PI * traj.freqZ * t);

    s.vx = 2.0f * M_PI * traj.freqX * traj.amplitudeX * cosf(2.0f * M_PI * traj.freqX * t);
    s.vy = 2.0f * M_PI * traj.freqY * traj.amplitudeY * cosf(2.0f * M_PI * traj.freqY * t);
    s.vz = 2.0f * M_PI * traj.freqZ * traj.amplitudeZ * cosf(2.0f * M_PI * traj.freqZ * t);

    return s;
}
