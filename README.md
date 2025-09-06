# rtos-sensor-fusion

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

- A C++ FreeRTOS-based prototype simulating a small unmanned underwater vehicle (UUV) control stack.  
- Multi-task scheduling with **FreeRTOS (sensors, estimator, control, watchdog).
- Sensor fusion via a simple estimator (IMU + GPS + barometer).
- Fault injection and graceful degradation (GPS dropout, IMU jitter).
- Unit & integration tests with GoogleTest.
- CI/CD pipeline (Dockerized build, static analysis, tests on GitHub Actions).
- MISRA C++ compliance checks with cppcheck/clang-tidy.

## ⚙️ Getting Started ##

1. Clone Repo & Init Submodules
git https://github.com/BensaiBishop/rtos-sensor-fusion.git
cd rtos-sensor-fusion
git submodule update --init --recursive

2. Build Dev Environment with Docker
docker build -f Dockerfile.dev -t rtos-dev .
docker run -it --rm -v $(pwd):/workspace -w /workspace rtos-dev /bin/bash
# (If using cmd) docker run -it --rm -v %cd%:/workspace -w /workspace rtos-dev /bin/bash 


3. Build & Run
mkdir -p build && cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j$(nproc)
./rtos-sensor-fusion   # run the demo
ctest                  # run the test