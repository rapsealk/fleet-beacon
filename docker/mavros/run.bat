SET package=catkin_ws/src/beginner_tutorials

docker run -it --rm --network host -v %cd%/%package%:/root/%package% -v %cd%/%1:/root/%package%/config.json mavros:melodic-ros-base
