// dashboard_task.cpp
#include "tasks/dashboard_task.hpp"
#include <iostream>
#include "state/state.hpp"

extern "C" {
    #include "FreeRTOS.h"
    #include "task.h"
}

std::atomic<int> dashImuCounter{0};
std::atomic<int> dashGpsCounter{0};
std::atomic<int> dashBaroCounter{0};
std::atomic<int> dashControlCounter{0};
std::atomic<int> dashEstimateCounter{0};

void DashboardTask(void* pvParameters) {
    (void) pvParameters;

    std::cout << "\033[2J"; //clr screan
    while (true) {
        {
            std::cout << "\033[H"; // Move cursor to top-left
            std::cout << "| IMU: " << dashImuCounter.load() << "| GPS: " << dashGpsCounter.load() << "| Barometer:" << dashBaroCounter.load() << "| Control: " << dashControlCounter.load() << "| Estimator: " << dashEstimateCounter.load() << std::endl;
            std::cout << "| True -> X:" << trueState.x << "| Y:" << trueState.y << "| Z:" << trueState.z <<std::endl;
            std::cout << "| Est -> X: " << estState.x << "| Y: " << estState.y << "| Z: " << estState.z <<std::endl;
            std::cout << "| VelT -> vx: " << trueState.vx << "| vy: " << trueState.vy << "| vz: " << trueState.vz <<std::endl;
            std::cout << std::flush;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // update every 100 ms
    }
}
