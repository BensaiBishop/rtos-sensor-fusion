#include <gtest/gtest.h>

float computePID (float error, float dt, float Kp, float Ki, float Kd) {
    static float integral = 0.0f;
    static float prevError = 0.0f;

    integral += error * dt;
    float derivative = (error - prevError) / dt;
    prevError = error;

    return Kp * error + Ki * integral + Kd * derivative;
}

// only dKd
TEST(ControlTaskTest, PID_BasicProportional) {
    float output = computePID (1.0f, 1.0f, 2.0f, 0.0f, 0.0f);
    EXPECT_FLOAT_EQ(output, 2.0f);
}

// with I, intregral should accumulate
TEST(ControlTaskTest, PID_WithIntegral) {
    float output1 = computePID (1.0f, 1.0f, 0.0f, 1.0f, 0.0f);
    float output2 = computePID (1.0f, 1.0f, 0.0f, 1.0f, 0.0f);
    EXPECT_GT(output2,output1);
}

// derivative of step error
TEST(ControlTaskTest, PID_WithDerivative) {
    float output = computePID (1.0f, 1.0f, 0.0f, 0.0f, 1.0f);
    EXPECT_FLOAT_EQ(output, 1.0f);
}