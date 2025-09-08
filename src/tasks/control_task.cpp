#include "tasks/control_task.hpp"
#include <iostream>
#include "state/state.hpp"
#include "tasks/dashboard_task.hpp"
#include <cmath>

extern "C"{
    #include "FreeRTOS.h"
    #include "task.h"
}

// doesn't actually do anything because the plany motion is scripted in sensor to test EKF

//desired depth (the setpoint)
constexpr float desiredDepth = -5.0f;

// PID gains 
constexpr float Kp = 2.0f;
constexpr float Ki = 0.4f;
constexpr float Kd = 0.2f;

// actuator limits
constexpr float max_control = 10.0f;    // clamp control command
constexpr float max_accel   = 5.0f;     // clamp acceleration applied to plant

// settled check
int stableCounter = 0;             // how many cycles in tolerance
const int requiredStableCycles = 50; // 50Hz * 1s = 50 cycles


void ControlTask(void* pvParameters) {
    TickType_t lastWake = xTaskGetTickCount();

    float integral = 0.0f;
    float prevError = 0.0f;
    const float dt = 0.02f; //20ms sample time

    //vars for converges calc
    static bool settled = false;
    static TickType_t settleTick = 0;

    while(true) {
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(20)); // 50Hz control

        float estZ = 0.0f;
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            estZ = estState.z;  
        }

        // PID compute
        float error = desiredDepth - estZ;    
        integral += error * dt; 

        //clamp integral
        constexpr float integral_max = 50.0f;
        if (integral > integral_max) {integral = integral_max;} 
        if (integral < -integral_max) {integral = -integral_max;} 

        float derivative = (error - prevError) / dt;
        prevError = error;

        float control = Kp * error + Ki * integral * Kd * derivative;
        prevError = error;

        //clamp control
        if (control > max_control) control = max_control;
        if (control < -max_control) control = -max_control;

        //convert control to accel
        float plant_gain = 0.5f; // tweak to change responsiveness
        float accel = control * plant_gain;

        //clamp accel
        if (accel > max_accel) {accel = max_accel;} 
        if (accel < -max_accel) {accel = -max_accel;} 

        //apply control to truth model to simulate thrust
        {
            std::lock_guard<std::mutex> lock(stateMutex);
            trueState.vz += accel * dt;

            //clamp velocity
            constexpr float max_vz = 10.0f;
            if (trueState.vz > max_vz) trueState.vz = max_vz;
            if (trueState.vz < -max_vz) trueState.vz = -max_vz;
        }
        dashControlCounter++;

        if (!settled) {
            if (fabs(error) < 0.1f) {
                stableCounter++;
                if (stableCounter >= requiredStableCycles) {
                    settled = true;
                    settleTick = xTaskGetTickCount();
                    std::cout << "Settled at t = "
                              << (settleTick / 1000.0f) << " s" << std::endl;
                }
            } else {
                stableCounter = 0; // reset if error leaves tolerance
            }
        }
    }
}
