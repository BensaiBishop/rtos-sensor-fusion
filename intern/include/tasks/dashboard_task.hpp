// dashboard_task.hpp
#ifndef DASHBOARD_TASK_HPP
#define DASHBOARD_TASK_HPP

#include <atomic>

extern std::atomic<int> dashImuCounter;
extern std::atomic<int> dashGpsCounter;
extern std::atomic<int> dashBaroCounter;
extern std::atomic<int> dashControlCounter;
extern std::atomic<int> dashEstimateCounter;

void DashboardTask(void* pvParameters);

#endif /* DASHBOARD_TASK_HPP */
