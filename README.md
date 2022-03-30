# Fleet Beacon
![ci-lint](https://github.com/rapsealk/fleet-beacon/workflows/Lint/badge.svg)
![ci-unittest](https://github.com/rapsealk/fleet-beacon/workflows/UnitTest/badge.svg)
![ci-build-docker](https://github.com/rapsealk/fleet-beacon/workflows/Build%20Docker%20Images/badge.svg)
![Python](https://img.shields.io/badge/Python-3.8-blue?logo=python&style=flat-square)
![ROS](https://img.shields.io/badge/ROS-Melodic-black?logo=ros&style=flat-square)
![Docker](https://img.shields.io/badge/Runtime-Docker-blue?logo=docker&style=flat-square)

## 데모 영상
* Warehouse 목록
![Fleet-Beacon-01](https://user-images.githubusercontent.com/14137676/160910963-f2b28b58-7603-40b1-9fe7-643d5522d17e.gif)
* Mission 생성
![Fleet-Beacon-02](https://user-images.githubusercontent.com/14137676/160911024-0d04e1dd-1245-4b05-8811-05aed2beb5fd.gif)
* Mission 할당 및 진행
![Fleet-Beacon-03](https://user-images.githubusercontent.com/14137676/160911087-24a2521e-7e62-49f7-b345-978c5c8ce65a.gif)

## Docker Image Repository
* mavros ([mavros:melodic-ros-base](https://hub.docker.com/repository/docker/rapsealk/mavros))

## 사용 방법
* `fleet-beacon` 서버
```bash
# Python으로 실행하기
$ python -m pip install -e .
$ python main.py [--workers <WORKERS>] [--host <HOST>] [--port <PORT>]
# 혹은 Docker 이미지로 실행하기
$ docker build . --tag fleet-beacon:latest
$ docker run -it --rm fleet-beacon
```
* `mavros`
```bash
$ cd docker/mavros
$ python gen_config.py --warehouse <WAREHOUSE_ID>   # e.g. `1bd3ddf8-db54-459e-a703-081ecfb9ad84.json` 파일 생성
$ export ROS_PACKAGE=beginner_tutorials
$ export CATKIN_WS=catkin_ws/src/$ROS_PACKAGE
$ docker run -it --rm -v $PWD/$CATKIN_WS:/root/$CATKIN_WS -v $PWD/$CATKIN_WS/1bd3ddf8-db54-459e-a703-081ecfb9ad84.json:/root/$CATKIN_WS/config.json rapsealk/mavros:melodic-ros-base
(docker) $ catkin build
(docker) $ source devel/setup.bash
(docker) $ roscore &
(docker) $ rosrun beginner_tutorials px4_node.py &
(docker) $ rosrun beginner_tutorials offb_node.py
```
