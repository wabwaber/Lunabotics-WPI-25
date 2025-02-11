FROM ros:humble-perception-jammy

#ENTRYPOINT [ "/bin/bash", "-c"]
#SHELL ["/bin/bash", "c"]

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \ 
    git-all \
    software-properties-common

RUN apt-get update && apt-get install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2-* \
    ros-humble-rviz2*

RUN apt-get install -y libeigen3-dev libboost-all-dev libceres-dev

RUN apt-get update && apt-get upgrade -y

RUN apt-get install -y \
    libssl-dev \
    libusb-1.0.0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev

RUN apt-get update && apt-get install -y \
    usbutils \
    nano \
    curl \
    apt-transport-https

RUN apt-get install -y \
    git \
    wget \
    cmake \
    build-essential

# RUN apt-get install -y \
#     libglfw3-dev \
#     libgl1-mesa-dev \
#     libglu1-mesa \
#     at \
#     v4l-utils

# RUN git clone https://github.com/IntelRealSense/librealsense.git

# RUN cd librealsense

# RUN /librealsense/scripts/setup_udev_rules.sh

# RUN mkdir build && cd build

# RUN cmake ../ -DBUILD_EXAMPLES=true

# RUN make uninstall && make clean && make && make install


RUN apt update && apt upgrade -y