FROM ros:melodic-ros-base

RUN sudo apt-get update
RUN sudo apt-get install python-catkin-tools python-rosinstall-generator -y

# Install Python3 packages
RUN sudo apt-get install python3-pip -y
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install requests

RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws
RUN catkin init
RUN wstool init src

# Install MAVLink
RUN rosinstall_generator --rosdistro melodic mavlink | tee /tmp/mavros.rosinstall

# Install MAVROS from source using released version
RUN rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall

# Create workspace & deps
RUN wstool merge -t src /tmp/mavros.rosinstall
RUN wstool update -t src -j4
RUN rosdep install --from-paths src --ignore-src -y

# Install GeographicLib datasets
RUN ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Build source
RUN catkin config --extend /opt/ros/melodic
RUN catkin build

SHELL [ "/bin/bash", "-c" ]
RUN source devel/setup.bash

CMD [ "/bin/bash" ]
