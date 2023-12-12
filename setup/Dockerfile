FROM osrf/ros:humble-desktop-jammy

# USE BASH
SHELL ["/bin/bash", "-c"]

# RUN LINE BELOW TO REMOVE debconf ERRORS (MUST RUN BEFORE ANY apt-get CALLS)
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

##### Setup Gazebo #####
# essential install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    lsb-release \
    binutils \
    mesa-utils \
    kmod \
    x-window-system \
    curl \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# install gazebo packages
RUN curl -sSL http://get.gazebosim.org | sh

# some necessary packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-gazebo-ros-pkgs \
    libsdl2-dev \
    libsdl2-2.0-0 \
    libopencv-dev \
    libcanberra-gtk-module \
    qtbase5-dev \
    qt5-qmake \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/dev_ws

# setup bashrc settings
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /usr/share/gazebo/setup.bash" >> ~/.bashrc && \
    echo "alias b='source /home/dev_ws/install/setup.bash'" >> ~/.bashrc

# setup entrypoint
COPY ./gazebo_entrypoint.sh /
RUN chmod +x /gazebo_entrypoint.sh

ENTRYPOINT ["/gazebo_entrypoint.sh"]

# personal packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
   vim \
   gdb \
   && rm -rf /var/lib/apt/lists/*