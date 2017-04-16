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
#include "libsoromc/logger.h"

#define LogTag "ConnectionStatusController"

namespace Soro {

ConnectionStatusController::ConnectionStatusController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for latency topic...");
    _latencySubscriber = MainController::getNodeHandle()->subscribe
            <std_msgs::UInt32, Soro::ConnectionStatusController>
            ("latency", 1, &ConnectionStatusController::onNewLatencyMessage, this);

    Logger::logInfo(LogTag, "Creating ROS subscriber for bitrate topic...");
    _bitrateSubscriber = MainController::getNodeHandle()->subscribe
            <ros_generated::bitrate, Soro::ConnectionStatusController>
            ("bitrate", 1, &ConnectionStatusController::onNewBitrateMessage, this);

    Logger::logInfo(LogTag, "Creating ROS publisher for bits_up_log topic...");
    _bitsUpPublisher = MainController::getNodeHandle()->advertise<std_msgs::UInt32>("bits_up_log", 1);

    Logger::logInfo(LogTag, "Creating ROS publisher for bits_down_log topic...");
    _bitsDownPublisher = MainController::getNodeHandle()->advertise<std_msgs::UInt32>("bits_down_log", 1);

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

void ConnectionStatusController::onNewBitrateMessage(ros_generated::bitrate msg)
{
    Q_EMIT bitrateUpdate((quint64)msg.bitrateUp, (quint64)msg.bitrateDown);
}

void ConnectionStatusController::onNewLatencyMessage(std_msgs::UInt32 msg)
{
    if (_disconnectWatchdogTimerId != -1) {
        killTimer(_disconnectWatchdogTimerId);
    }
    _disconnectWatchdogTimerId = startTimer(_disconnectTimeThreshold);
    setConnected(true);

    Q_EMIT latencyUpdate((quint32)msg.data);
}

void ConnectionStatusController::logBitsDown(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _bitsDownPublisher.publish(msg);
}

void ConnectionStatusController::logBitsUp(quint32 bits)
{
    std_msgs::UInt32 msg;
    msg.data = bits;
    _bitsUpPublisher.publish(msg);
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
