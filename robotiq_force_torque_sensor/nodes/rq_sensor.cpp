/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
* Modifications Copyright (c) 2017 READY Robotics
*/

/**
 * \file rq_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Nicolas Lauzier <nicolas@robotiq.com>
 */

#include <string.h>
#include <stdio.h>

#include "boost/bind.hpp"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include <launch_manager_interface/optional_comm_channel.hpp>
#include "geometry_msgs/WrenchStamped.h"
#include "robotiq_force_torque_sensor/rq_sensor_state.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"


static void decode_message_and_do(INT_8 const  * const buff, INT_8 * const ret);
bool wait_for_other_connection(void);
bool try_connection(void);
static int max_retries_(5);
void shutdown(void);
void simulateSensor(ros::NodeHandle& nh);
void launchRealSensor(ros::NodeHandle& nh);
ros::Publisher sensor_pub;
ros::Publisher wrench_pub;
ros::Publisher watchdog_pub;
ros::ServiceServer service;

/**
 * \brief Decode the message received and do the associated action
 * \param buff message to decode
 * \param ret buffer containing the return value from a GET command
 */
static void decode_message_and_do(INT_8 const  * const buff, INT_8 * const ret)
{
    INT_8 get_or_set[3];
    INT_8 nom_var[4];

    if(buff == NULL || strlen(buff) != 7)
    {
        return;
    }

    strncpy(get_or_set, &buff[0], 3);
    strncpy(nom_var, &buff[4], strlen(buff) -3);

    if(strstr(get_or_set, "GET"))
    {
        rq_state_get_command(nom_var, ret);
    }
    else if(strstr(get_or_set, "SET"))
    {
        if(strstr(nom_var, "ZRO"))
        {
            rq_state_do_zero_force_flag();
            strcpy(ret,"Done");
        }
    }
}

bool receiverCallback(robotiq_force_torque_sensor::sensor_accessor::Request& req,
    robotiq_force_torque_sensor::sensor_accessor::Response& res)
{
    ROS_INFO("I heard: [%s]",req.command.c_str());
    INT_8 buffer[512];
    decode_message_and_do((char*)req.command.c_str(), buffer);
    res.res = buffer;
    ROS_INFO("I send: [%s]", res.res.c_str());
    return true;
}

/**
 * \fn bool wait_for_other_connection()
 * \brief Each second, checks for a sensor that has been connected
 */
bool wait_for_other_connection(void)
{
    INT_8 ret;
    INT_8 count = 0;

    while(ros::ok() && count < 6)
    {
        if (count % 2 == 0)
        {
            ROS_INFO("Waiting for sensor connection...");
        }
        usleep(10000); //Attend 1 seconde.

        ret = rq_sensor_state(max_retries_);
        if(ret == 0)
        {
            ROS_INFO("Sensor connected!");
            return true;
        }
        count++;
        ros::spinOnce();
    }
    return false;
}

/**
 * \fn bool try_connection()
 * \brief A helper function to avoid repeated code
 */
bool try_connection(void)
{
    INT_8 ret;
    ret = rq_sensor_state(max_retries_);
    if(ret == -1)
    {
        if (!wait_for_other_connection())
        {
            return false;
        }
    }
    return true;
}

/**
 * \fn void get_data(void)
 * \brief Builds the message with the force/torque data
 * \return ft_sensor updated with the latest data
 */
static robotiq_force_torque_sensor::ft_sensor get_data(void)
{
    robotiq_force_torque_sensor::ft_sensor msgStream;

    msgStream.Fx = rq_state_get_received_data(0);
    msgStream.Fy = rq_state_get_received_data(1);
    msgStream.Fz = rq_state_get_received_data(2);
    msgStream.Mx = rq_state_get_received_data(3);
    msgStream.My = rq_state_get_received_data(4);
    msgStream.Mz = rq_state_get_received_data(5);

    return msgStream;
}

void simulateSensor(ros::NodeHandle& nh) 
{
    OptionalCommChannel launch_control_link(true);
    launch_control_link.send_pending();
    ros::Publisher connection_pub = nh.advertise<std_msgs::Bool>("robotiq_force_torque_sensor_connected", 1);
    sensor_pub = nh.advertise<robotiq_force_torque_sensor::ft_sensor>("robotiq_force_torque_sensor", 512);
    wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("robotiq_force_torque_wrench", 512);
    watchdog_pub = nh.advertise<std_msgs::Empty>("/robotiq_ft300_force_torque_sensor/watchdog", 1);
    service = nh.advertiseService("robotiq_force_torque_sensor_acc", receiverCallback);

    std_msgs::Bool connection_msg;
    robotiq_force_torque_sensor::ft_sensor msgStream;
    geometry_msgs::WrenchStamped wrenchMsg;
    std_msgs::Empty watchdog_msg;
    ros::param::param<std::string>("~frame_id", wrenchMsg.header.frame_id, "robotiq_force_torque_frame_id");
    INT_32 publish_count = 0;


    // Publish the connection state
    connection_msg.data = true;
    connection_pub.publish(connection_msg);
    ros::spinOnce();

    ROS_INFO("Starting Sensor");

    msgStream.Fx = 0.0;
    msgStream.Fy = 0.0;
    msgStream.Fz = 0.0;
    msgStream.Mx = 0.0;
    msgStream.My = 0.0;
    msgStream.Mz = 0.0;
    wrenchMsg.wrench.force.x = msgStream.Fx;
    wrenchMsg.wrench.force.y = msgStream.Fy;
    wrenchMsg.wrench.force.z = msgStream.Fz;
    wrenchMsg.wrench.torque.x = msgStream.Mx;
    wrenchMsg.wrench.torque.y = msgStream.My;
    wrenchMsg.wrench.torque.z = msgStream.Mz;


    launch_control_link.send_normal();
    while(ros::ok()) {
        sensor_pub.publish(msgStream);
        wrenchMsg.header.stamp = ros::Time::now();
        wrench_pub.publish(wrenchMsg);

        // Publish the connection state every 1000 iterations
        if (publish_count % 10 == 0) {
            watchdog_pub.publish(watchdog_msg);
        }

        if (publish_count == 1000) {
            connection_msg.data = true;
            connection_pub.publish(connection_msg);
            publish_count = 0;
        }
        publish_count++;
        // Keep publishing rate to 100Hz
        ros::Duration(0.01).sleep();
        ros::spinOnce();
    }

    ROS_INFO("Sensor Stopped");

    // Publish the connection state
    connection_msg.data = false;
    connection_pub.publish(connection_msg);
    ros::spinOnce();

    // unregister publishers
    sensor_pub.shutdown();
    wrench_pub.shutdown();
    watchdog_pub.shutdown();
    service.shutdown();
    connection_pub.shutdown();
}


void launchRealSensor(ros::NodeHandle& nh)
{
    OptionalCommChannel launch_control_link(true);
    launch_control_link.send_pending();
    std_msgs::Bool connection_msg;
    ros::param::param<int>("~max_retries", max_retries_, 100);

    INT_8 bufStream[512];
    INT_32 publish_count = 0;
    robotiq_force_torque_sensor::ft_sensor msgStream;
    bool connected = false;
    uint8_t created_publishers = 0;

    //If we can't initialize, we return an error
    connected = try_connection();

    //Reads basic info on the sensor
    if (connected) {
        connected = try_connection();
    }

    //Starts the stream
    if (connected) {
        connected = try_connection();
    }

    ros::Publisher connection_pub = nh.advertise<std_msgs::Bool>("robotiq_force_torque_sensor_connected", 1);
    if (connected) {
        sensor_pub = nh.advertise<robotiq_force_torque_sensor::ft_sensor>("robotiq_force_torque_sensor", 512);
        wrench_pub = nh.advertise<geometry_msgs::WrenchStamped>("robotiq_force_torque_wrench", 512);
        watchdog_pub = nh.advertise<std_msgs::Empty>("/robotiq_ft300_force_torque_sensor/watchdog", 1);
        service = nh.advertiseService("robotiq_force_torque_sensor_acc", receiverCallback);
        created_publishers = 1;
    }

    //std_msgs::String msg;
    geometry_msgs::WrenchStamped wrenchMsg;
    std_msgs::Empty watchdog_msg;
    ros::param::param<std::string>("~frame_id", wrenchMsg.header.frame_id, "robotiq_force_torque_frame_id");

    // Publish the connection state
    connection_msg.data = connected;
    connection_pub.publish(connection_msg);
    ros::spinOnce();

    if (connected)
    {
        ROS_INFO("Starting Sensor");
        launch_control_link.send_normal();
    }
    else {
        launch_control_link.send_failure();
    }

    while(ros::ok() && connected)
    {
        connected = try_connection();

        if(rq_sensor_get_current_state() == RQ_STATE_RUN)
        {
            strcpy(bufStream,"");
            msgStream = get_data();

            if(rq_state_got_new_message())
            {
                sensor_pub.publish(msgStream);

                // compose WrenchStamped Msg
                wrenchMsg.header.stamp = ros::Time::now();
                wrenchMsg.wrench.force.x = msgStream.Fx;
                wrenchMsg.wrench.force.y = msgStream.Fy;
                wrenchMsg.wrench.force.z = msgStream.Fz;
                wrenchMsg.wrench.torque.x = msgStream.Mx;
                wrenchMsg.wrench.torque.y = msgStream.My;
                wrenchMsg.wrench.torque.z = msgStream.Mz;
                wrench_pub.publish(wrenchMsg);
            }
        }

        // Publish the connection state every 1000 iterations
        if (publish_count % 10 == 0)
        {
            watchdog_pub.publish(watchdog_msg);
        }

        if (publish_count == 1000)
        {
            connection_msg.data = connected;
            connection_pub.publish(connection_msg);
            publish_count = 0;
        }
        publish_count++;
        ros::spinOnce();
    }

    if (!connected)
    {
        connection_msg.data = connected;
        connection_pub.publish(connection_msg);
        ros::spinOnce();
    }

    ROS_INFO("Sensor Stopped");
    if (created_publishers)
    {
        // unregister publishers
        sensor_pub.shutdown();
        wrench_pub.shutdown();
        watchdog_pub.shutdown();
        service.shutdown();
    }

    // unregister
    connection_pub.shutdown();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotiq_force_torque_sensor");
    ROS_INFO("Trying to Start Sensor");
    ros::NodeHandle n;

    bool simulated;
    ros::param::param<bool>("/simulate_robot", simulated, false);
    if (simulated) {
        simulateSensor(n);
    }
    else {
        launchRealSensor(n);
    }
    return 0;
}
