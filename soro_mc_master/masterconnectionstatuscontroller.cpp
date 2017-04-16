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

#include "masterconnectionstatuscontroller.h"
#include "maincontroller.h"
#include "libsoromc/logger.h"
#include <QtConcurrent/QtConcurrent>

#define LogTag "MasterConnectionStatusController"

namespace Soro {

MasterConnectionStatusController::MasterConnectionStatusController
            (uint pingInterval, uint bitrateInterval, QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for bits_up_log topic...");
    _bitsUpSubscriber = _nh.subscribe
            <std_msgs::UInt32, Soro::MasterConnectionStatusController>
            ("bits_up_log", 10, &MasterConnectionStatusController::onNewBitsUpMessage, this);
    if (!_bitsUpSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for bits_up_log topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for bits_down_log topic...");
    _bitsDownSubscriber = _nh.subscribe
            <std_msgs::UInt32, Soro::MasterConnectionStatusController>
            ("bits_down_log", 10, &MasterConnectionStatusController::onNewBitsDownMessage, this);
    if (!_bitsDownSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for bits_down_log topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for bitrate topic...");
    _bitratePublisher = _nh.advertise<ros_generated::bitrate>("bitrate", 1);
    if (!_bitratePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for bitrate topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for bits_up_log topic...");
    _latencyPublisher = _nh.advertise<std_msgs::UInt32>("latency", 1);
    if (!_latencyPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for latency topic");

    _pingWorker = new PingWorker(pingInterval);
    _pingWorker->moveToThread(&_workerThread);
    connect(_pingWorker, &PingWorker::ack, this, &MasterConnectionStatusController::onNewLatency, Qt::QueuedConnection);

    Logger::logInfo(LogTag, "All ROS connections created");

    _bitrateInterval = bitrateInterval;
    _pingInterval = pingInterval;
    _disconnectWatchdogTimerId = -1;
    _connected = false;
    _bitsDown = _bitsUp = 0;
    _bitrateCalcTimerId = startTimer(bitrateInterval);
}

MasterConnectionStatusController::~MasterConnectionStatusController()
{
    if (_pingWorker) delete _pingWorker;
}

void MasterConnectionStatusController::onNewBitsUpMessage(std_msgs::UInt32 msg)
{
    logBitsUp(msg.data);
}

void MasterConnectionStatusController::onNewBitsDownMessage(std_msgs::UInt32 msg)
{
    logBitsDown(msg.data);
}

void MasterConnectionStatusController::onNewLatency(quint32 latency)
{
    if (_disconnectWatchdogTimerId != -1) {
        killTimer(_disconnectWatchdogTimerId);
    }
    _disconnectWatchdogTimerId = startTimer(_pingInterval * 5);
    setConnected(true);

    std_msgs::UInt32 msg;
    msg.data = latency;
    _latencyPublisher.publish(msg);

    Q_EMIT latencyUpdate(latency);
}

void MasterConnectionStatusController::logBitsDown(quint32 bits)
{
    _bitsDown += bits;
}

void MasterConnectionStatusController::logBitsUp(quint32 bits)
{
    _bitsUp += bits;
}

bool MasterConnectionStatusController::isConnected() const
{
    return _connected;
}

void MasterConnectionStatusController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _disconnectWatchdogTimerId)
    {
        setConnected(false);
        killTimer(_disconnectWatchdogTimerId);
        _disconnectWatchdogTimerId = -1;
    }
    else if (e->timerId() == _bitrateCalcTimerId)
    {
        quint64 rateUp = (float)_bitsUp / (float)_bitrateInterval;
        quint64 rateDown = (float)_bitsDown / (float)_bitrateInterval;
        _bitsUp = _bitsDown = 0;

        ros_generated::bitrate msg;
        msg.bitrateDown = rateDown;
        msg.bitrateUp = rateUp;

        // Send bitrate message on bitrate topic
        _bitratePublisher.publish(msg);

        Q_EMIT bitrateUpdate(rateUp, rateDown);
    }
}

void MasterConnectionStatusController::setConnected(bool connected)
{
    if (connected != _connected) {
        _connected = connected;
        Q_EMIT connectedChanged(_connected);
    }
}

} // namespace Soro
