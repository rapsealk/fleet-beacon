/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */
#include <iostream>
#include <istream>
#include <ostream>
#include <string>

#include <ros/ros.h>
#include <geographic_msgs/GeoPoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

#include <boost/asio.hpp>

mavros_msgs::State current_state;
sensor_msgs::NavSatFix nav_sat_fix;

void _setpoint_position_global_callback(const geographic_msgs::GeoPoseStamped::ConstPtr& message)
{
    nav_sat_fix.latitude = message->pose.position.latitude;
    nav_sat_fix.longitude = message->pose.position.longitude;
    nav_sat_fix.altitude = message->pose.position.altitude;
}

bool _handle_mavros_cmd_arming(mavros_msgs::CommandBool::Request &request,
                               mavros_msgs::CommandBool::Response &response)
{
    current_state.armed = request.value;
    response.success = true;
    // response.result = 0;
    return true;
}

bool _handle_mavros_set_mode(mavros_msgs::SetMode::Request &request,
                             mavros_msgs::SetMode::Response &response)
{
    current_state.mode = request.custom_mode;
    response.mode_sent = true;
    return true;
}

int main(int argc, char* argv[])
{
    ///
    boost::asio::io_service io_service;
    ///

    ros::init(argc, argv, "px4_node");
    ros::NodeHandle nh;

    /** Initialize */
    current_state.connected = true;

    nav_sat_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    nav_sat_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    /** Publishers */
    ros::Publisher state_publisher = nh.advertise<mavros_msgs::State>("mavros/state", 10);
    ros::Publisher global_position_publisher = nh.advertise<sensor_msgs::NavSatFix>("mavros/global_position/global", 10);

    /** Subscribers */
    ros::Subscriber setpoint_position_global_subscriber =
        nh.subscribe<geographic_msgs::GeoPoseStamped>("mavros/setpoint_position/global", 10, _setpoint_position_global_callback);

    /** Services */
    ros::ServiceServer arming_service = nh.advertiseService("mavros/cmd/arming", _handle_mavros_cmd_arming);
    ros::ServiceServer set_mode_service = nh.advertiseService("mavros/set_mode", _handle_mavros_set_mode);

    /** (OFFBOARD) publishing rate MUST be faster than 2Hz. */
    ros::Rate rate(20.0);

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok() && current_state.connected)
    {
        state_publisher.publish(current_state);
        global_position_publisher.publish(nav_sat_fix);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
