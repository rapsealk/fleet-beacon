#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import json

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus

current_state = State()
global_position = GeoPoseStamped()
nav_sat_fix = NavSatFix()


def _setpoint_position_global_callback(message: GeoPoseStamped):
    global global_position
    global_position = message

    nav_sat_fix.latitude = global_position.pose.position.latitude
    nav_sat_fix.longitude = global_position.pose.position.longitude
    nav_sat_fix.altitude = global_position.pose.position.altitude


def _handle_mavros_cmd_arming(request: CommandBool) -> CommandBoolResponse:
    current_state.armed = request.value
    return CommandBoolResponse(success=True, result=0)


def _handle_mavros_set_mode(request: SetMode) -> SetModeResponse:
    current_state.mode = request.custom_mode
    return SetModeResponse(mode_sent=True)


def main():
    rospy.init_node("PX4_node", anonymous=True)

    # Initialize state
    current_state.connected = True
    current_state.armed = False
    current_state.guided = True
    current_state.manual_input = False
    current_state.mode = "MANUAL"
    current_state.system_status = 0

    global_position.pose.position.latitude = 0
    global_position.pose.position.longitude = 0
    global_position.pose.position.altitude = 0

    with open(os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.json"), "r") as f:
        config = json.load(f)
        nav_sat_fix.latitude = config["global_position"].get("latitude", -1)
        nav_sat_fix.longitude = config["global_position"].get("longitude", -1)
        nav_sat_fix.altitude = config["global_position"].get("altitude", -1)

    nav_sat_fix.status.status = NavSatStatus.STATUS_FIX
    nav_sat_fix.status.service = NavSatStatus.SERVICE_GPS

    # Publishers
    state_publisher = rospy.Publisher("mavros/state", State, queue_size=10)
    global_position_publisher = rospy.Publisher("mavros/global_position/global", NavSatFix, queue_size=10)

    # Subscribers
    _ = rospy.Subscriber("mavros/setpoint_position/global", GeoPoseStamped, _setpoint_position_global_callback, queue_size=10)

    # Services
    rospy.Service("mavros/cmd/arming", CommandBool, _handle_mavros_cmd_arming)
    rospy.Service("mavros/set_mode", SetMode, _handle_mavros_set_mode)

    rate = rospy.Rate(1.0)     # (OFFBOARD) publishing rate MUST be faster than 2Hz (20.0)

    while not rospy.is_shutdown() and current_state.connected:
        # rospy.loginfo('[{}] current_state: {}'.format(datetime.now().isoformat(), current_state))
        state_publisher.publish(current_state)
        global_position_publisher.publish(nav_sat_fix)
        rate.sleep()


if __name__ == "__main__":
    main()
