#!/bin/bash
source /opt/ros/$ROS_DISTRO/setup.bash

export ROS_HOSTNAME=127.0.0.1
export ROS_MASTER_URI=http://localhost:11311

roscore
