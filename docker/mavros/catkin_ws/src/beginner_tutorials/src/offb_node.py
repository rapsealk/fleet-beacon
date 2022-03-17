#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()


def state_cb(msg):
    global current_state
    current_state = msg


def main():
    rospy.init_node("offb_node")

    _ = rospy.Subscriber("mavros/state", State, state_cb, queue_size=10)
    local_pose_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20.0)

    while not rospy.is_shutdown() and not current_state.connected:
        rospy.spin()
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 0

    for i in range(100):
        # assert not rospy.is_shutdown()
        local_pose_pub.publish(pose)
        rospy.spin()
        rate.sleep()

    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            try:
                response = set_mode_client(base_mode=0, custom_mode="OFFBOARD")
                if response.mode_sent:
                    rospy.loginfo("Offboard enabled")
            except rospy.ServiceException as e:
                rospy.logerr(e)
            last_request = rospy.Time.now()
        elif not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            try:
                response = arming_client(value=True)
                if response.success:
                    rospy.loginfo("Vehicle armed")
            except rospy.ServiceException as e:
                rospy.logerr(e)
            last_request = rospy.Time.now()

        local_pose_pub.publish(pose)

        rospy.spin()
        rate.sleep()


if __name__ == "__main__":
    main()
