# Supports Ubuntu 18.04, 20.04, 22.04, 24.04.
# Default: 22.04. Override: --build-arg UBUNTU_VERSION=18.04
ARG UBUNTU_VERSION=22.04

# ── Stage 1: Build DRFL daemon ────────────────────────────────────────────
FROM ubuntu:${UBUNTU_VERSION} AS daemon-builder

ARG UBUNTU_VERSION=22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    g++ \
    cmake \
    git \
    pkg-config \
    libcurl4-openssl-dev \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*
# Note: no libpoco-dev — daemon uses Doosan-bundled Poco from API-DRFL submodule

WORKDIR /daemon

COPY lux_drfl_daemon/ ./

# third_party/API-DRFL must be checked out on host before docker build.
# Provides: include/DRFLEx.h + library/Linux/64bits/<arch>/<ver>/libDRFL.a
#           + bundled libPocoFoundation.so + libPocoNet.so
RUN cmake -B build \
        -DDRCF_VERSION=3 \
        -DUBUNTU_VERSION=${UBUNTU_VERSION} \
        . \
    && cmake --build build -j$(nproc) \
    && strip build/drfl_daemon

# ── Stage 2: Compile server.py with Nuitka ───────────────────────────────
FROM python:3.11-slim-bullseye AS compiler

RUN apt-get update && apt-get install -y \
    gcc \
    patchelf \
    && rm -rf /var/lib/apt/lists/*

RUN pip install nuitka zstandard

WORKDIR /build

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY server.py .

RUN python -m nuitka \
    --onefile \
    --output-filename=robot_ui \
    --include-package=fastapi \
    --include-package=uvicorn \
    --include-package=pydantic \
    --include-package=serial \
    server.py

# ── Stage 3: Runtime image ────────────────────────────────────────────────
FROM ubuntu:${UBUNTU_VERSION}

ARG UBUNTU_VERSION=22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    iproute2 \
    iputils-ping \
    sudo \
    libcurl4 \
    && rm -rf /var/lib/apt/lists/*
# No libpoco-dev or libpocofoundationXX — we bundle Doosan's Poco .so files below

COPY --from=daemon-builder /daemon/build/drfl_daemon      /app/bin/drfl_daemon
COPY --from=daemon-builder /daemon/build/poco_libs/       /app/lib/
COPY --from=compiler       /build/robot_ui                /app/robot_ui

WORKDIR /app

COPY ui_dist/ ./ui/

RUN mkdir -p /app/plans /app/stats \
    && ldconfig /app/lib

ENV DRFL_DAEMON_BIN=/app/bin/drfl_daemon
ENV LD_LIBRARY_PATH=/app/lib

EXPOSE 8000

CMD ["/app/robot_ui"]
