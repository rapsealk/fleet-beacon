#!/usr/bin/python3
# -*- coding: utf-8 -*-
# from datetime import datetime

import rospy
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolResponse, SetMode, SetModeResponse
from geographic_msgs.msg import GeoPoseStamped

current_state = State()
global_position = GeoPoseStamped()


def _setpoint_position_global_callback(message: GeoPoseStamped):
    global global_position
    global_position = message


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

    # Publishers
    state_publisher = rospy.Publisher("mavros/state", State, queue_size=10)

    # Subscribers
    _ = rospy.Subscriber("mavros/setpoint_position/global", GeoPoseStamped, _setpoint_position_global_callback, queue_size=10)

    # Services
    rospy.Service("mavros/cmd/arming", CommandBool, _handle_mavros_cmd_arming)
    rospy.Service("mavros/set_mode", SetMode, _handle_mavros_set_mode)

    rate = rospy.Rate(20.0)     # (OFFBOARD) publishing rate MUST be faster than 2Hz

    while not rospy.is_shutdown() and not current_state.connected:
        # print('[{}] current_state: {}'.format(datetime.now().isoformat(), current_state))
        state_publisher.publish(current_state)
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    main()
