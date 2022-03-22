#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import multiprocessing
import os
import sys
import time
from datetime import datetime

import redis
import requests
import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix

"""
def get_mac_address():
    return ":".join(("%012x" % uuid.getnode())[i:i+2] for i in range(0, 12, 2)).upper()
"""

UUID = ""

current_state = State()
nav_sat_fix = NavSatFix()


def _state_callback(message: State):
    global current_state
    current_state = message


def _global_position_callback(message: NavSatFix):
    global nav_sat_fix
    nav_sat_fix = message


def send_heartbeat(host: str = "127.0.0.1", port: int = 6379, channel: str = "heartbeat"):
    r = redis.Redis(host=host, port=port)
    while True:
        message = json.dumps({
            "uuid": UUID,
            "timestamp": datetime.now().isoformat(),
            "global_position": {
                "latitude": nav_sat_fix.latitude,
                "longitude": nav_sat_fix.longitude,
                "altitude": nav_sat_fix.altitude
            }
        })
        r.publish(channel=channel, message=message)
        time.sleep(1)


def main():
    rospy.init_node("offb_node", anonymous=True)

    # Publishers
    global_position_publisher = rospy.Publisher("mavros/setpoint_position/global", GeoPoseStamped, queue_size=10)

    # Subscribers
    _ = [rospy.Subscriber("mavros/state", State, _state_callback, queue_size=10),
         rospy.Subscriber("mavros/global_position/global", NavSatFix, _global_position_callback, queue_size=10)]

    # Service clients
    arming_client = None
    rospy.loginfo("Waiting for service: \"mavros/cmd/arming\"")
    rospy.wait_for_service("mavros/cmd/arming")
    try:
        arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    except rospy.ServiceException as e:
        rospy.logerr(e)
        sys.exit(1)

    set_mode_client = None
    rospy.loginfo("Waiting for service: \"mavros/set_mode\"")
    rospy.wait_for_service("mavros/set_mode")
    try:
        set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    except rospy.ServiceException as e:
        rospy.logerr(e)
        sys.exit(1)

    # Initialize
    with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.json"), "r") as f:
        global UUID
        config = json.load(f)
        base_url = config["base_url"]
        UUID = config["uuid"]

    """
    global_position = GeoPoseStamped()
    global_position.pose.position.latitude = config["global_position"].get("latitude", 0)
    global_position.pose.position.longitude = config["global_position"].get("longitude", 0)
    global_position.pose.position.altitude = config["global_position"].get("altitude", 0)

    global_position_publisher.publish(global_position)
    """

    nav_sat_fix.latitude = config["global_position"].get("latitude", 0)
    nav_sat_fix.longitude = config["global_position"].get("longitude", 0)
    nav_sat_fix.altitude = config["global_position"].get("altitude", 0)

    # HTTP Request
    response = requests.post(f"{base_url}/robot", json={key: config[key] for key in ["uuid", "warehouse"]})
    if response.status_code == requests.codes.created:
        pass    # print(f"[POST /robot] {response.content()}")
    else:
        pass

    # Heartbeat
    process = multiprocessing.Process(target=send_heartbeat, args=("host.docker.internal", 6379), daemon=True)
    process.start()

    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and not current_state.connected:
        rospy.spin()
        rate.sleep()

    process.join()


if __name__ == "__main__":
    main()
