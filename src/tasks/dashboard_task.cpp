// dashboard_task.cpp
#include "tasks/dashboard_task.hpp"
#include <iostream>

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
    // Clear the screen once at start
    std::cout << "\033[2J"; 

    while (true) {
        std::cout << "\033[H"; // Move cursor to top-left

        std::cout << "IMU:      " << dashImuCounter.load() << std::endl;
        std::cout << "GPS:      " << dashGpsCounter.load() << std::endl;
        std::cout << "Barometer:" << dashBaroCounter.load() << std::endl;
        std::cout << "Control:  " << dashControlCounter.load() << std::endl;
        std::cout << "Estimator:  " << dashEstimateCounter.load() << std::endl;
        std::cout << std::flush;

        vTaskDelay(pdMS_TO_TICKS(100)); // update every 100 ms
    }
}
