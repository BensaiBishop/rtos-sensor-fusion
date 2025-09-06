# rtos-sensor-fusion

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

A **C++ FreeRTOS-based prototype** simulating a small unmanned underwater vehicle (UUV) control stack.  
This project demonstrates professional embedded engineering practices:
- Multi-task scheduling with **FreeRTOS** (sensors, estimator, control, watchdog).
- **Sensor fusion** via a simple estimator (IMU + GPS + barometer).
- **Fault injection** and graceful degradation (GPS dropout, IMU jitter).
- **Unit & integration tests** with GoogleTest.
- **CI/CD pipeline** (Dockerized build, static analysis, tests on GitHub Actions).
- **MISRA C++ compliance checks** with cppcheck/clang-tidy.
