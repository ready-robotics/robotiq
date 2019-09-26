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

#include <stdio.h>
#include <string.h>

#include "boost/bind.hpp"
#include "geometry_msgs/WrenchStamped.h"
#include "robotiq_force_torque_sensor/ft_sensor.h"
#include "robotiq_force_torque_sensor/rq_sensor_state.h"
#include "robotiq_force_torque_sensor/sensor_accessor.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"

class SensorBase
{
public:
    SensorBase() : _connection_pub(_nh.advertise<std_msgs::Bool>("robotiq_force_torque_sensor_connected", 1))
    {
        ros::param::param<std::string>("~frame_id", _wrench_msg.header.frame_id, "robotiq_force_torque_frame_id");
    }

    virtual ~SensorBase() = 0;
    virtual bool makeConnection() = 0;

    void Connect();
    bool isConnected() const
    {
        return _connection_msg.data;
    }

    void publishSensor();
    void publishWatchdog()
    {
        _watchdog_pub.publish(std_msgs::Empty());
    }

    void publishConnection()
    {
        _connection_pub.publish(_connection_msg);
        ros::spinOnce();
    }

protected:
    void setConnection(bool newState)
    {
        _connection_msg.data = newState;
        publishConnection();
    }

private:
    virtual bool _getSensorData(robotiq_force_torque_sensor::ft_sensor &msgStream) = 0;
    bool _receiverCallback(robotiq_force_torque_sensor::sensor_accessor::Request &req, robotiq_force_torque_sensor::sensor_accessor::Response &res);
    virtual void _decodeMessageAndDo(INT_8 const *const buff, INT_8 *const ret) = 0;

    ros::NodeHandle _nh;
    std_msgs::Bool _connection_msg;
    ros::Publisher _connection_pub;

    robotiq_force_torque_sensor::ft_sensor msgStream;
    ros::Publisher _sensor_pub;

    geometry_msgs::WrenchStamped _wrench_msg;
    ros::Publisher _wrench_pub;
    ros::Publisher _watchdog_pub;
    ros::ServiceServer _service;
};

class SimulatedSensor : public SensorBase
{
public:
    bool makeConnection() override
    {
        setConnection(true);
        return true;
    }

private:
    bool _getSensorData(robotiq_force_torque_sensor::ft_sensor &msgStream) override
    {
        msgStream.Fx = 0.0;
        msgStream.Fy = 0.0;
        msgStream.Fz = 0.0;
        msgStream.Mx = 0.0;
        msgStream.My = 0.0;
        msgStream.Mz = 0.0;
        // Keep publishing rate to 100Hz
        ros::Duration(0.01).sleep();
        return true;
    }
    void _decodeMessageAndDo(INT_8 const *const /*buff*/, INT_8 *const /*ret*/) override
    {
    }
};

class RealSensor : public SensorBase
{
public:
    RealSensor() : _max_retries(5)
    {
        ros::param::param("~max_retries", _max_retries, 100);
    }

    bool makeConnection() override
    {
        uint8_t tries_left = 3;
        while (!isConnected() && tries_left > 0) {
            tries_left--;
            if (_tryConnection()) {
                setConnection(true);
                return true;
            }
        }
        return false;
    }

private:
    void _decodeMessageAndDo(INT_8 const *const buff, INT_8 *const ret) override;
    bool _waitForOtherConnection(void);
    bool _tryConnection();
    bool _getSensorData(robotiq_force_torque_sensor::ft_sensor &msgStream) override;

    // Would prefer a uint32_t but ROS doesn't provide unsigned values
    int32_t _max_retries;
};

SensorBase::~SensorBase()
{
    if (isConnected()) {
        setConnection(false);
    }
    ROS_INFO("Sensor Stopped");
    // publishers send shutdown upon destruction if advertized
}

void SensorBase::Connect()
{
    if (makeConnection()) {
        _sensor_pub = _nh.advertise<robotiq_force_torque_sensor::ft_sensor>("robotiq_force_torque_sensor", 512);
        _wrench_pub = _nh.advertise<geometry_msgs::WrenchStamped>("robotiq_force_torque_wrench", 512);
        _watchdog_pub = _nh.advertise<std_msgs::Empty>("/robotiq_ft300_force_torque_sensor/watchdog", 1);
        _service = _nh.advertiseService("robotiq_force_torque_sensor_acc", &SensorBase::_receiverCallback, this);
    }
}

void SensorBase::publishSensor()
{
    robotiq_force_torque_sensor::ft_sensor msgStream;

    if (_getSensorData(msgStream)) {
        _sensor_pub.publish(msgStream);

        _wrench_msg.header.stamp = ros::Time::now();
        _wrench_msg.wrench.force.x = msgStream.Fx;
        _wrench_msg.wrench.force.y = msgStream.Fy;
        _wrench_msg.wrench.force.z = msgStream.Fz;
        _wrench_msg.wrench.torque.x = msgStream.Mx;
        _wrench_msg.wrench.torque.y = msgStream.My;
        _wrench_msg.wrench.torque.z = msgStream.Mz;
        _wrench_pub.publish(_wrench_msg);
    }
}

bool SensorBase::_receiverCallback(robotiq_force_torque_sensor::sensor_accessor::Request &req,
                                   robotiq_force_torque_sensor::sensor_accessor::Response &res)
{
    ROS_INFO("I heard: [%s]", req.command.c_str());
    INT_8 buffer[512];
    _decodeMessageAndDo((char *)req.command.c_str(), buffer);
    res.res = buffer;
    ROS_INFO("I send: [%s]", res.res.c_str());
    return true;
}

/**
 * \brief Decode the message received and do the associated action
 * \param buff message to decode
 * \param ret buffer containing the return value from a GET command
 */
void RealSensor::_decodeMessageAndDo(INT_8 const *const buff, INT_8 *const ret)
{
    INT_8 get_or_set[3];
    INT_8 nom_var[4];

    if (buff == NULL || strlen(buff) != 7) {
        return;
    }

    strncpy(get_or_set, &buff[0], 3);
    strncpy(nom_var, &buff[4], strlen(buff) - 3);

    if (strstr(get_or_set, "GET")) {
        rq_state_get_command(nom_var, ret);
    }
    else if (strstr(get_or_set, "SET")) {
        if (strstr(nom_var, "ZRO")) {
            rq_state_do_zero_force_flag();
            strcpy(ret, "Done");
        }
    }
}

/**
 * \fn bool waitForOtherConnection()
 * \brief Each second, checks for a sensor that has been connected
 */
bool RealSensor::_waitForOtherConnection(void)
{
    uint8_t tries_left = 6;

    while (ros::ok() && tries_left > 0) {
        if (tries_left-- % 2 == 0) {
            ROS_INFO("Waiting for sensor connection...");
        }
        if (rq_sensor_state(_max_retries) == 0) {
            ROS_INFO("Sensor connected!");
            return true;
        }
        ros::Duration(1.0).sleep();
    }
    return false;
}

/**
 * \fn bool try_connection()
 * \brief A helper function to avoid repeated code
 */
bool RealSensor::_tryConnection()
{
    if (rq_sensor_state(_max_retries) != -1) {
        return true;
    }
    return _waitForOtherConnection();
}

bool RealSensor::_getSensorData(robotiq_force_torque_sensor::ft_sensor &msgStream)
{
    if (!_tryConnection() || rq_sensor_get_current_state() != RQ_STATE_RUN) {
        return false;
    }

    msgStream.Fx = rq_state_get_received_data(0);
    msgStream.Fy = rq_state_get_received_data(1);
    msgStream.Fz = rq_state_get_received_data(2);
    msgStream.Mx = rq_state_get_received_data(3);
    msgStream.My = rq_state_get_received_data(4);
    msgStream.Mz = rq_state_get_received_data(5);

    return rq_state_got_new_message();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robotiq_force_torque_sensor");

    std::unique_ptr<SensorBase> sensor_ptr;

    bool simulated;
    ros::param::param<bool>("/simulate_robot", simulated, false);
    if (simulated) {
        ROS_INFO("Trying to Start Simulated Robotiq Sensor");
        sensor_ptr.reset(new SimulatedSensor);
    }
    else {
        ROS_INFO("Trying to Start Robotiq FT300 Sensor");
        sensor_ptr.reset(new RealSensor);
    }

    uint32_t publish_count = 0;

    sensor_ptr->Connect();
    while (ros::ok() && sensor_ptr->isConnected()) {
        sensor_ptr->publishSensor();

        if (publish_count % 10 == 0) {
            sensor_ptr->publishWatchdog();
        }
        if (publish_count == 1000) {
            sensor_ptr->publishConnection();
            publish_count = 0;
        }
        publish_count++;
    }

    return 0;
}
