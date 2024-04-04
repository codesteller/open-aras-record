# generated from docker_images/create_ros_image.Dockerfile.em
FROM osrf/ros:noetic-desktop-focal

# Install dependencies
## Add TZDATA to avoid interactive prompt
ENV DEBIAN_FRONTEND=noninteractive

ENV UDEV=on

RUN apt-get update && apt-get install -y tzdata
## TZ configure to Asia/Kolkata
RUN ln -fs /usr/share/zoneinfo/Asia/Kolkata /etc/localtime
RUN dpkg-reconfigure --frontend noninteractive tzdata

RUN apt-get update && apt-get upgrade -y 

# install ros packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full 

RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   software-properties-common git libusb-1.0-0-dev \
   wget zsh python3-catkin-tools v4l-utils vlc libv4l-dev

RUN apt-get install -y ros-noetic-pcl-ros  ros-noetic-imu-tools ros-noetic-rviz-imu-plugin \
    ros-noetic-rtabmap-ros ros-noetic-rqt ros-noetic-rqt-image-view

RUN apt-get install -y \
    python3-dev \
    python3-pip \
    python3-catkin-tools \
    python3-osrf-pycommon \
    python3-rosdep 

RUN pip3 install rospkg pyyaml numpy opencv-python==4.5.4.60 \
    opencv-contrib-python==4.5.4.60 transforms3d pyquaternion \
    scipy matplotlib ipython jupyterlab

RUN apt-get install -y ros-noetic-foxglove-bridge

# Setup Environment
# RUN echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | tee /etc/udev/rules.d/80-movidius.rules

ENV ARDUINO_IDE_VERSION 1.8.19
RUN wget -q -O- https://downloads.arduino.cc/arduino-${ARDUINO_IDE_VERSION}-linux64.tar.xz \
	| tar -xJC /opt/ \
    && sh /opt/arduino-${ARDUINO_IDE_VERSION}/install.sh

# Arduino Dependencies
RUN apt-get install -y ros-noetic-rosserial ros-noetic-rosserial-arduino libxtst6

SHELL ["/bin/bash", "-c"]
RUN cd /opt/arduino-${ARDUINO_IDE_VERSION}/libraries \
    && rm -rf ros_lib \
    && source /opt/ros/noetic/setup.bash \
    && rosrun rosserial_arduino make_libraries.py .

RUN apt-get install -y neovim tmux ros-noetic-rviz-imu-plugin

RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
# RUN echo "source /workspace/calibration/devel/setup.bash" >> ~/.bashrc
# RUN echo 'export PS1="[NOETIC][DAI][\u@docker] \W # "' >> ~/.bashrc

CMD ["bash"]


#catkin create pkg vedyne_serial --catkin-deps roscpp geometry_msgs sensor_msgs serial std_msgs std_srvs tf 
