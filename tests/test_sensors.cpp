#include <gtest/gtest.h>
#include <cmath>
#include "sim/trajectory.hpp"
#include "sim/sensor_models.hpp"

// Test that the trajectory generator returns expected positions and velocities
TEST(TrajectoryTest, SineTrajectory) {
    TrajectoryParams testParams{1.0f, 2.0f, 3.0f, 0.5f, 1.0f, 2.0f};
    traj = testParams; // override global trajectory

    float t = 0.0f;
    TrueState s = generateTrajectory(t);

    EXPECT_NEAR(s.x, 0.0f, 1e-6);
    EXPECT_NEAR(s.y, 0.0f, 1e-6);
    EXPECT_NEAR(s.z, 0.0f, 1e-6);

    EXPECT_NEAR(s.vx, 2.0f * M_PI * testParams.freqX * testParams.amplitudeX,1e-6);
}

// Test that noisy() returns a value within a reasonable bound
TEST(SensorModelsTest, NoisyFunction) {
    float val = 5.0f;
    float stddev = 0.1f;

    for (int i = 0; i < 1000; ++i) {
        float noisyVal = noisy(val, stddev);
        // Uniform noise is [-stddev, stddev]
        EXPECT_GE(noisyVal, val - stddev);
        EXPECT_LE(noisyVal, val + stddev);
    }
}

// Test barometer measurement simulation
TEST(SensorModelsTest, BaroMeasurement) {
    float trueZ = -10.0f;
    float baro = noisy(trueZ, 0.05f);
    EXPECT_NEAR(baro, trueZ, 0.05f); // within one standard deviation
}

// Test GPS measurement simulation
TEST(SensorModelsTest, GpsMeasurement) {
    float trueX = 3.5f, trueY = -2.1f;
    float gpsX = noisy(trueX, 0.1f);
    float gpsY = noisy(trueY, 0.1f);
    EXPECT_NEAR(gpsX, trueX, 0.1f);
    EXPECT_NEAR(gpsY, trueY, 0.1f);
}

