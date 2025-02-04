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

RUN apt-get update && apt-get install -y \
    usbutils

RUN apt update && apt upgrade -y

RUN apt install nano -y && apt update