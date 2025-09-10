#include "tasks/sensor_task.hpp"
#include "tasks/control_task.hpp"
#include "tasks/estimator_task.hpp"
#include "tasks/watchdog_task.hpp"
#include "tasks/dashboard_task.hpp"

#include <iostream>

extern "C" 
{
#include "FreeRTOS.h"
#include "task.h"
}

int main() {
    std::cout << "Starting POSIX FreeRTOS demo..." << std::endl;

    xTaskCreate(WatchdogTask, "Watchdog", 1024, nullptr, 1, nullptr);
    xTaskCreate(ControlTask, "Control", 2048, nullptr, 2, nullptr);
    xTaskCreate(EstimatorTask, "Estimator", 2048, nullptr, 3, nullptr);
    xTaskCreate(SensorTask, "Sensor", 2048, nullptr, 4, nullptr);
    xTaskCreate(DashboardTask, "Dashboard", 1024, nullptr, 5, nullptr);
    std::cout << "Estimator stack high-water mark: "
          << uxTaskGetStackHighWaterMark(NULL) << std::endl;


    vTaskStartScheduler();

    std::cout << "Scheduler ended!" << std::endl;
}