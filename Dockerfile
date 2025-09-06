FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools & dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    clang \
    clang-tidy \
    cppcheck \
    python3 \
    python3-pip \
    gdb \
    wget \
    unzip \
    curl \
    vim \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Build GoogleTest
RUN cd /usr/src/gtest && cmake . && make -j$(nproc) && cp *.a /usr/lib

# Static analysis tools
RUN pip3 install --no-cache-dir cpplint

WORKDIR /workspace

CMD ["/bin/bash"]
