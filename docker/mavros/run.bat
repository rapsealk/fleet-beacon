SET package=catkin_ws/src/beginner_tutorials

docker run -it --rm -v %cd%/%package%:/root/%package% -v %cd%/%1:/root/%package%/config.json -v %2:/root/%package%/000.mp4 rapsealk/mavros:melodic-ros-base
