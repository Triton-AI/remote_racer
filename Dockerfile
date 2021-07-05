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
    ros-noetic-usb-cam \
    python3-catkin-pkg

RUN python3 -m pip install --upgrade pip

RUN python3 -m pip install pyvesc

WORKDIR /root/ros_ws/
SHELL ["/bin/bash", "-c"]
RUN mkdir src/

RUN cd src && \
    git clone https://github.com/Slamtec/rplidar_ros.git

RUN cd src && \
    git clone https://gitlab.com/djnighti/ucsd_robo_car_ros.git

RUN cd src && \
    git clone https://github.com/f1tenth/vesc.git

RUN echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
RUN echo 'source /root/ros_ws/noetic/setup.bash' >> ~/.bashrc

RUN cd src && \
    git clone https://github.com/ros/catkin && \
    cd catkin && \
    mkdir build && \
    cd build && \
    cmake ../ && \
    make && \
    make install

RUN cd src && \
    git clone https://github.com/wjwwood/serial.git && \
    cd serial && \
    make && \
    make install

RUN rosdep install --from-paths src --ignore-src -r -y
RUN cd /root/ros_ws && \
    source /opt/ros/noetic/setup.bash && \
    catkin_make