#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import math
import os
import sys
import time
from collections import namedtuple
from threading import Thread
from typing import Iterable

import redis
# import requests
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix

LatLng = namedtuple("LatLng",
                    ("latitude", "longitude"))


def _haversine(theta: float) -> float:
    return math.sin(theta / 2.0)


def get_haversine_distance(src: LatLng, dst: LatLng, radius: float = 6371) -> float:
    # phi(φ): latitude, lambda(λ): longitude
    (rad_lat_src, rad_lng_src) = (math.radians(src.latitude), math.radians(src.longitude))
    (rad_lat_dst, rad_lng_dst) = (math.radians(dst.latitude), math.radians(dst.longitude))

    delta_lat = rad_lat_dst - rad_lat_src
    delta_lng = rad_lng_dst - rad_lng_src

    sin_delta_lat = math.pow(math.sin(delta_lat / 2.0), 2.0)
    sin_delta_lng = math.pow(math.sin(delta_lng / 2.0), 2.0)

    return 2 * radius * math.asin(math.sqrt(sin_delta_lat + math.cos(rad_lat_src) * math.cos(rad_lat_dst) * sin_delta_lng))


"""
def get_mac_address():
    return ":".join(("%012x" % uuid.getnode())[i:i+2] for i in range(0, 12, 2)).upper()
"""

REDIS_HOST = "host.docker.internal"
REDIS_PORT = 6379


class OffboardNode:
    def __init__(self, name: str = "offb_node", anonymous: bool = False):
        rospy.init_node(name, anonymous=anonymous)

        self._current_state = State()
        self._nav_sat_fix = NavSatFix()

        self._uuid = ""
        self._warehouse = 0

        self._goal_state = "MANUAL"
        self._missions = []
        self._current_mission_index = -1

    def initialize(self):
        # Publishers
        self.global_position_publisher = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)

        # Subscribers
        self._subscribers = [
            rospy.Subscriber("mavros/state", State, self._state_callback, queue_size=10),
            rospy.Subscriber("mavros/global_position/global", NavSatFix, self._global_position_callback, queue_size=10)
        ]

        # Service clients
        self.service_clients = {}
        services = (("mavros/cmd/arming", CommandBool), ("mavros/set_mode", SetMode))
        for service_name, message_type in services:
            rospy.loginfo(f"Waiting for service: \"{service_name}\"")
            rospy.wait_for_service(service_name)
            try:
                self.service_clients[service_name] = rospy.ServiceProxy(service_name, message_type)
                rospy.loginfo(f"Service found: {service_name}")
            except rospy.ServiceException as e:
                rospy.logerr(e)
                sys.exit(1)

        # Initialize
        with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.json"), "r") as f:
            config = json.load(f)
            base_url = config["base_url"]
            self._uuid = config["uuid"]
            self._warehouse = config["warehouse"]

        # HTTP Request
        """
        response = requests.post(f"{base_url}/api/unit", json={
            "uuid": config["uuid"],
            "warehouse_id": config["warehouse"]
        })
        if response.status_code == requests.codes.created:
            pass    # print(f"[POST /robot] {response.content()}")
        else:
            pass
        """

        # Heartbeat Thread
        heartbeat_thread = Thread(target=self._send_heartbeat, args=(REDIS_HOST, REDIS_PORT), daemon=True)
        heartbeat_thread.start()
        rospy.loginfo(f"Hearbeat thread: {heartbeat_thread}")

        # Redis Thread
        redis_thread = Thread(target=self._subscribe_redis_queue, args=([self.uuid],), daemon=True)
        redis_thread.start()
        rospy.loginfo(f"Redis thread: {redis_thread}")

        rate = rospy.Rate(1.0)     # 20.0

        while not rospy.is_shutdown() and not self.current_state.connected:
            rate.sleep()

        while not rospy.is_shutdown() and self.current_state.connected:
            # Mission
            rospy.loginfo(f"""
                [Main Loop]
                - current_state.mode = {self.current_state.mode}
                - current_state.armed = {self.current_state.armed}
            """)
            if self._missions:
                rospy.loginfo(f"[Main Loop] Current Mission Index: {self._current_mission_index}")
                if self.current_state.mode != "OFFBOARD":
                    response = self.service_clients["mavros/set_mode"](custom_mode="OFFBOARD")
                    rospy.loginfo(f"[Main Loop] Response(mavros/set_mode): {response}")
                    if response.mode_sent:
                        pass
                elif not self.current_state.armed:
                    response = self.service_clients["mavros/cmd/arming"](True)
                    rospy.loginfo(f"[Main Loop] Response(mavros/cmd/arming): {response}")
                else:
                    # TODO: Takeoff
                    goal_position = self._missions[self._current_mission_index]
                    rospy.loginfo(f"[Main Loop] Goal position: {goal_position}")
                    if self.reached_at(goal_position):
                        self._current_mission_index += 1
                    else:
                        goal_position = self.compute_next_target_position(goal_position)
                        rospy.loginfo(f"[Main Loop] Target position: {goal_position}")
                        message = GeoPoseStamped()
                        message.pose.position.latitude = goal_position.latitude
                        message.pose.position.longitude = goal_position.longitude
                        self.global_position_publisher.publish(message)
                        # rospy.loginfo(f"[Main Loop] Publish(mavros/setpoint_position/global) = {message}")

                if self._current_mission_index >= len(self._missions):
                    # TODO: Land
                    self.clear_mission()

            rate.sleep()

        # Join child processes/threads
        redis_thread.join()
        heartbeat_thread.join()

    def assign_mission(self, missions: Iterable[LatLng]):
        self._missions = tuple(missions)
        self._current_mission_index = 0

        rospy.loginfo("Assign Mission")
        for i, mission in enumerate(missions):
            rospy.loginfo(f"Mission[{i}] Lat: {mission.latitude}, Lng: {mission.longitude}")

    def clear_mission(self):
        self._missions = []
        self._current_mission_index = -1

    def compute_next_target_position(self, goal: LatLng) -> LatLng:
        current = LatLng(self.nav_sat_fix.latitude, self.nav_sat_fix.longitude)
        return LatLng(
            current.latitude + (goal.latitude - current.latitude) / 100,
            current.longitude + (goal.longitude - current.longitude) / 100)

    def reached_at(self, position: LatLng) -> bool:
        current_position = LatLng(self.nav_sat_fix.latitude, self.nav_sat_fix.longitude)
        return get_haversine_distance(current_position, position) < 0.01     # Km

    """
    ROS Callback functions
    """
    def _state_callback(self, message: State):
        self._current_state = message

    def _global_position_callback(self, message: NavSatFix):
        self._nav_sat_fix = message
        rospy.loginfo(f"""
            [mavros/global_position/global] NavSatFix: ({self.nav_sat_fix.latitude}, {self.nav_sat_fix.longitude})
        """)

    """
    Sub Threads
    """
    def _send_heartbeat(self, host: str = "127.0.0.1", port: int = 6379, channel: str = "heartbeat"):
        r = redis.Redis(host=host, port=port)
        while True:
            message = json.dumps({
                "timestamp": int(time.time() * 1000),
                "uuid": self.uuid,
                "warehouse_id": self._warehouse,
                "global_position": {
                    "latitude": self.nav_sat_fix.latitude,
                    "longitude": self.nav_sat_fix.longitude,
                    "altitude": self.nav_sat_fix.altitude
                }
            })
            rospy.loginfo(f"[Redis] Publish: {message}")
            r.publish(channel=channel, message=message)
            r.publish(channel="global_position", message=message)
            time.sleep(1)

    def _subscribe_redis_queue(self, topics: Iterable[str]):
        pubsub = redis.Redis(host=REDIS_HOST, port=REDIS_PORT, db=0).pubsub()
        for topic in topics:
            pubsub.subscribe(topic)

        while True:
            try:
                message = pubsub.get_message(timeout=5)
                if not message:
                    continue
                message_type = message.get("type")
                if message_type == "subscribe":
                    pass
                elif message_type == "message":
                    message_data = message.get("data", b"{}")
                    message_data = json.loads(message_data)
                    message_data_type = message_data.get("type")
                    if message_data_type == "mission":
                        waypoints = [LatLng(waypoint["latitude"], waypoint["longitude"])
                                     for waypoint in message_data.get("waypoints", [])]
                        self.assign_mission(waypoints)
            except Exception as e:
                rospy.logerr(e)

    @property
    def current_state(self) -> State:
        return self._current_state

    @property
    def nav_sat_fix(self) -> NavSatFix:
        return self._nav_sat_fix

    @property
    def uuid(self) -> str:
        return self._uuid


if __name__ == "__main__":
    node = OffboardNode("offb_node", anonymous=True)
    node.initialize()
