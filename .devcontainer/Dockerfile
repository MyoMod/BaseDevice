FROM mcr.microsoft.com/devcontainers/base:noble

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Find new gcc versions here: 
# https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads
ENV GCC_VERSION=13.3.rel1


#COPY version_vcpkg.env install_vcpkg.sh /opt/

# Install packages via apt
RUN apt-get update \
    && export DEBIAN_FRONTEND=noninteractive \
    #
    # Install packages for C++ development
    && apt-get -y install --no-install-recommends curl xz-utils x11-apps xauth git python3 \
    cmake ninja-build \
    #
    # Install JLink
    && wget -q --post-data 'accept_license_agreement=accepted&non_emb_ctr=confirmed' https://www.segger.com/downloads/jlink/JLink_Linux_x86_64.deb -O /opt/JLink_Linux_x86_64.deb \
    && printf '#!/bin/bash \n echo not running udevadm "$@"' > /usr/bin/udevadm && chmod +x /usr/bin/udevadm \
    && apt update && apt upgrade -y && apt install -y /opt/JLink_Linux_x86_64.deb \
    && rm -f /opt/JLink_Linux_x86_64.deb \
    #
    # Install ozone
    && wget -q https://www.segger.com/downloads/jlink/Ozone_Linux_V334_x86_64.deb \
    && apt install -y libxrandr-dev ./Ozone_Linux_V334_x86_64.deb \
    && rm -f Ozone_Linux_V334_x86_64.deb


# Install GCC
WORKDIR /opt/
RUN apt update && apt install -y gcc-arm-none-eabi \
        libnewlib-arm-none-eabi \
        build-essential \
        libstdc++-arm-none-eabi-newlib


# Enable pretty printing for gdb
RUN     apt install -y software-properties-common \
        && add-apt-repository ppa:deadsnakes/ppa -y \
        && apt update \
        && apt install -y python3.8 python3.8-distutils \
        #Install libstdc++6 for gdb pretty printing python scripts
        && apt install libstdc++6

# Tell gdb where to find the python scripts
COPY gdbinit /home/$USERNAME/.gdbinit

# Install dependencies for pico-sdk
RUN apt-get install -y  git \
                        cmake \
                        gcc-arm-none-eabi \
                        gcc \
                        g++ \
                        gdb-multiarch \
                        automake \
                        autoconf \
                        build-essential \
                        texinfo \
                        libtool \
                        libftdi-dev \
                        libusb-1.0-0-dev \
                        pkg-config

WORKDIR /home/sdk/pico
RUN git clone -b master https://github.com/raspberrypi/pico-sdk.git && \
    cd pico-sdk && \
    git submodule update --init

ENV PICO_SDK_PATH=/home/sdk/pico/pico-sdk

# install picoprobe
RUN git clone https://github.com/raspberrypi/picoprobe.git && \
    cd picoprobe && \
    git submodule update --init && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 

# Install openocd
Run git clone "https://github.com/raspberrypi/openocd.git" -b rp2040-v0.12.0 --depth=1 && \
    cd openocd && \
    ./bootstrap && \
    ./configure --enable-picoprobe --enable-ftdi && \
    make -j4 && \
    make install


# Install pimoroni pico libraries
RUN git clone https://github.com/pimoroni/pimoroni-pico.git /home/sdk/pico/pimoroni-pico && \
        cd /home/sdk/pico/pimoroni-pico && \
        git submodule update --init && \
        mkdir build

RUN echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> \
        /etc/sudoers

WORKDIR /opt/
RUN wget -qO- https://developer.arm.com/-/media/Files/downloads/gnu/${GCC_VERSION}/binrel/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi.tar.xz | tar -xJ
#rename the folder
RUN mv /opt/arm-gnu-toolchain-${GCC_VERSION}-x86_64-arm-none-eabi /opt/arm-none-eabi-gcc-${GCC_VERSION} && \
    ln -s /opt/arm-none-eabi-gcc-${GCC_VERSION} /opt/arm-none-eabi-gcc
ENV PATH "$PATH:/opt/arm-none-eabi-gcc/bin"
# install missing dependencies
RUN apt-get update && apt-get install -y libncurses6 \
        libncurses5-dev \
        libncursesw6 \
        libncursesw5-dev && \
        #
        # Clean up
        apt-get autoremove -y && \
        apt-get clean -y && \
        rm -rf /var/lib/apt/lists/* && \
        #
        # symlink libncurses5 to libncurses6
        #  see https://stackoverflow.com/questions/17005654/error-while-loading-shared-libraries-libncurses-so-5
        ln -s /usr/lib/x86_64-linux-gnu/libncursesw.so.6.4 /usr/lib/libncurses.so.5 && \
        ln -s /usr/lib/x86_64-linux-gnu/libncursesw.so.6.4 /usr/lib/libncursesw.so.5 && \
        ln -s /usr/lib/x86_64-linux-gnu/libncursesw.so.6.4 /usr/lib/libtinfo.so.5