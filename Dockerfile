FROM arm64v8/ros:noetic
LABEL maintainer="Siddharth Saha <sisaha@ucsd.edu>"

USER root

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update --fix-missing \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    python3-pip \
    git \
    vim \
    software-properties-common \
    cmake \
    nano \
    rsync \
    zip \
    python3-catkin-pkg \
    ros-noetic-rviz \
    usbutils \
    ros-noetic-image-view

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install git+https://github.com/LiamBindle/PyVESC

WORKDIR /root/ros_ws/
SHELL ["/bin/bash", "-c"]
RUN mkdir src/

RUN cd src && \
    git clone https://github.com/Triton-AI/rplidar_ros.git && \
    git clone https://github.com/Triton-AI/vesc.git && \
    git clone -b develop https://github.com/Triton-AI/usb_cam.git

RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
RUN echo 'source /root/ros_ws/devel/setup.bash' >> ~/.bashrc


RUN cd src && \
    git clone https://github.com/Triton-AI/catkin.git && \
    cd catkin && \
    mkdir build && \
    cd build && \
    source /opt/ros/noetic/setup.bash && \
    cmake ../ && \
    make && \
    make install

RUN cd src && \
    git clone https://github.com/Triton-AI/serial.git && \
    cd serial && \
    source /opt/ros/noetic/setup.bash && \
    make && \
    make install

RUN rosdep install --from-paths src --ignore-src -r -y
RUN cd /root/ros_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make
