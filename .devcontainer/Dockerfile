# Dev container for argos-smd-at-kineis-firmware (STM32WL55, Cortex-M4 CM4).
#
# Provides a reproducible Linux build / test / IDE environment:
#   - arm-none-eabi GCC 13.3.rel1  (matches the validated host toolchain,
#     GCC 13.3.x -> identical -Werror behaviour; do NOT bump blindly)
#   - native gcc + make for the Unity unit tests (Tests/scripts/run_tests.sh).
#     Bonus: /tmp is writable here, so the test runner works out of the box
#     (it does not on Windows git-bash).
#   - clangd + bear for IDE intellisense (compile_commands.json)
#
# Flashing: the Makefile `make flash*` targets drive a SEGGER J-Link on the
# HOST (proprietary, not bundled). st-link tools are included for users with an
# ST-LINK probe, but USB passthrough to a container is unreliable on
# Windows/macOS, so flashing from the host is the supported path.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/gcc-arm-none-eabi/bin:${PATH}"

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        make \
        cmake \
        git \
        wget \
        ca-certificates \
        xz-utils \
        unzip \
        libusb-1.0-0-dev \
        stlink-tools \
        gdb-multiarch \
        clangd \
        bear \
        python3 \
        python3-venv \
        python3-pip \
        minicom \
    && rm -rf /var/lib/apt/lists/*

# arm-none-eabi GCC 13.3.rel1 — pinned to match the host build (GCC 13.3.x).
# Bumping GCC can surface new -Werror diagnostics and break the build.
ARG ARM_TC=arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi
RUN wget -q "https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/${ARM_TC}.tar.xz" \
    && tar -xf "${ARM_TC}.tar.xz" -C /opt \
    && mv "/opt/${ARM_TC}" /opt/gcc-arm-none-eabi \
    && rm "${ARM_TC}.tar.xz"

WORKDIR /workspace
CMD ["/bin/bash"]