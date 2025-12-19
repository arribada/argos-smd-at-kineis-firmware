FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
ENV PATH="/opt/gcc-arm-none-eabi/bin:${PATH}"

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    libusb-1.0-0-dev \
    clangd \
    bear


RUN wget -q https://developer.arm.com/-/media/Files/downloads/gnu/13.2.rel1/binrel/arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz \
    && tar -xf arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz -C /opt \
    && mv /opt/arm-gnu-toolchain-* /opt/gcc-arm-none-eabi \
    && rm arm-gnu-toolchain-13.2.rel1-x86_64-arm-none-eabi.tar.xz

RUN git clone https://github.com/stlink-org/stlink.git /tmp/stlink \
    && cd /tmp/stlink \
    && make release \
    && make install DESTDIR=/usr/local \
    && rm -rf /tmp/stlink

RUN bash -c "$(wget https://raw.githubusercontent.com/ohmybash/oh-my-bash/master/tools/install.sh -O -)"

WORKDIR /workspace

CMD ["/bin/bash"]
