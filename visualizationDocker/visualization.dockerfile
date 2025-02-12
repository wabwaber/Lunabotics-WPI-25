FROM ros:humble-perception-jammy

#ENTRYPOINT [ "/bin/bash", "-c"]
#SHELL ["/bin/bash", "c"]

#A lot of these commands could be combined in one massive install call however for the sake of figuring out where it went wrong and also ensuring any dependencies are updated

#Get the universal dependancies for realsense and openVINS
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \ 
    git-all \
    software-properties-common

#install the realsense SDK and ROS wrapper
RUN apt-get update && apt-get install -y \
    ros-humble-librealsense2* \
    ros-humble-realsense2-*

#Install rvis2 (seperate because its a large install)
RUN apt-get install -y \
    ros-humble-rviz2*

#Install openVINS specific dependencies
RUN apt-get install -y libeigen3-dev libboost-all-dev libceres-dev

#Update any packages that are out of date (NOTE: this step may take a while)
RUN apt-get update && apt-get upgrade -y

#I have no idea -Matt (maybe realsense building from source dependencies)
RUN apt-get install -y \
    libssl-dev \
    libusb-1.0.0-dev \
    libudev-dev \
    pkg-config \
    libgtk-3-dev

#basic tools that I use 
RUN apt-get update && apt-get install -y \
    usbutils \
    nano \
    curl \
    apt-transport-https

#get the build tools for openVINS and realsense wrapper (if building from source)
RUN apt-get install -y \
    git \
    wget \
    cmake \
    build-essential

#below here are the commands for hte 
RUN mkdir -p /catkin_ws/openVins/src/

WORKDIR /catkin_ws/openVins/src

RUN git clone https://github.com/rpng/open_vins/

WORKDIR /catkin_ws/openVins/ 

RUN colcon build

RUN apt update && apt upgrade -y