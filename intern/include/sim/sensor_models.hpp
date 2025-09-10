#ifndef SENSOR_MODELS_HPP
#define SENSOR_MODELS_HPP

#include <cstdlib>

// Simple noise generator
inline float noisy(float val, float noiseStd) {
    float r = ((float)rand() / RAND_MAX) * 2.0f - 1.0f; // uniform [-1,1]
    return val + r * noiseStd;
}

#endif /* SENSOR_MODELS_HPP */
