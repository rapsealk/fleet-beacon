FROM ros:melodic-ros-base

SHELL [ "/bin/bash", "-c" ]
RUN source /opt/ros/$ROS_DISTRO/setup.bash

RUN export ROS_HOSTNAME=127.0.0.1
RUN export ROS_MASTER_URI=http://localhost:11311

CMD [ "roscore" ]
