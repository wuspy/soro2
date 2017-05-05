/*
 * Copyright 2017 The University of Oklahoma.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "connectionstatuscontroller.h"
#include "maincontroller.h"
#include "soro_core/logger.h"

#define LogTag "ConnectionStatusController"

namespace Soro {

ConnectionStatusController::ConnectionStatusController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for latency topic...");
    _latencySubscriber = _nh.subscribe
            <std_msgs::UInt32, Soro::ConnectionStatusController>
            ("latency", 10, &ConnectionStatusController::onNewLatencyMessage, this);
    if (!_latencySubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for latency topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for data_rate topic...");
    _dataRateSubscriber = _nh.subscribe
            <ros_generated::data_rate, Soro::ConnectionStatusController>
            ("data_rate", 10, &ConnectionStatusController::onNewDataRateMessage, this);
    if (!_dataRateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for data_rate topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for data_up_log topic...");
    _dataUpPublisher = _nh.advertise<std_msgs::UInt32>("data_up_log", 1);
    if (!_dataUpPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for data_up_log topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for data_down_log topic...");
    _dataDownPublisher = _nh.advertise<std_msgs::UInt32>("data_down_log", 1);
    if (!_dataDownPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for data_down_log topic");

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");

    _disconnectWatchdogTimerId = -1;
    setDisconnectTimeThreshold(2000);
    _connected = false;
}

void ConnectionStatusController::setDisconnectTimeThreshold(int threshold)
{
    _disconnectTimeThreshold = threshold;
}

int ConnectionStatusController::getDisconnectTimeThreshold() const
{
    return _disconnectTimeThreshold;
}

void ConnectionStatusController::onNewDataRateMessage(ros_generated::data_rate msg)
{
    Logger::logInfo(LogTag, "New bitrate: " + QString::number(msg.dataRateUp) + ", " + QString::number(msg.dataRateDown));
    Q_EMIT dataRateUpdate((quint64)msg.dataRateUp, (quint64)msg.dataRateDown);
}

void ConnectionStatusController::onNewLatencyMessage(std_msgs::UInt32 msg)
{
    Logger::logInfo(LogTag, "New latency: " + QString::number(msg.data));
    if (_disconnectWatchdogTimerId != -1) {
        killTimer(_disconnectWatchdogTimerId);
    }
    _disconnectWatchdogTimerId = startTimer(_disconnectTimeThreshold);
    setConnected(true);

    Q_EMIT latencyUpdate((quint32)msg.data);
}

void ConnectionStatusController::logDataDown(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _dataDownPublisher.publish(msg);
}

void ConnectionStatusController::logDataUp(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _dataUpPublisher.publish(msg);
}

bool ConnectionStatusController::isConnected() const
{
    return _connected;
}

void ConnectionStatusController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _disconnectWatchdogTimerId)
    {
        setConnected(false);
        killTimer(_disconnectWatchdogTimerId);
        _disconnectWatchdogTimerId = -1;
    }
}

void ConnectionStatusController::setConnected(bool connected)
{
    if (connected != _connected) {
        _connected = connected;
        Q_EMIT connectedChanged(_connected);
    }
}

} // namespace Soro
