#!/bin/bash
export FB_MAVROS_PACKAGE=catkin_ws/src/beginner_tutorials

docker run -it --rm --network host -v $PWD/$FB_MAVROS_PACKAGE:/root/$FB_MAVROS_PACKAGE -v $PWD/$1:/root/$FB_MAVROS_PACKAGE/config.json -v $PWD/../../media/$2:/root/$FB_MAVROS_PACKAGE/000.mp4 rapsealk/mavros:melodic-ros-base
