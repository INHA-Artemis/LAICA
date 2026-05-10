FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    iputils-ping \
    libboost-all-dev \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    usbutils \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
      rosdep init; \
    fi || true

WORKDIR /home/laica/LAICA_ws
COPY . /home/laica/LAICA_ws/src

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc && \
    echo "if [ -f /home/laica/LAICA_ws/install/setup.bash ]; then source /home/laica/LAICA_ws/install/setup.bash; fi" >> /root/.bashrc

CMD ["/bin/bash"]
