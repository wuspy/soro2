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
#include "soro_core/logger.h"
#include <QtConcurrent/QtConcurrent>

#define LogTag "MasterConnectionStatusController"

namespace Soro {

MasterConnectionStatusController::MasterConnectionStatusController
            (const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;

    Logger::logInfo(LogTag, "Creating ROS subscriber for data_up_log topic...");
    _dataUpSubscriber = _nh.subscribe
            <std_msgs::UInt32, Soro::MasterConnectionStatusController>
            ("data_up_log", 10, &MasterConnectionStatusController::onNewDataUpMessage, this);
    if (!_dataUpSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for data_up_log topic");

    Logger::logInfo(LogTag, "Creating ROS subscriber for data_down_log topic...");
    _dataDownSubscriber = _nh.subscribe
            <std_msgs::UInt32, Soro::MasterConnectionStatusController>
            ("data_down_log", 10, &MasterConnectionStatusController::onNewDataDownMessage, this);
    if (!_dataDownSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for data_down_log topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for data_rate topic...");
    _dataRatePublisher = _nh.advertise<ros_generated::data_rate>("data_rate", 1);
    if (!_dataRatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for data_rate topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for latency topic...");
    _latencyPublisher = _nh.advertise<std_msgs::UInt32>("latency", 1);
    if (!_latencyPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for latency topic");

    _pingWorker = new PingWorker(_settings->getPingInterval());
    _pingWorker->moveToThread(&_workerThread);
    connect(_pingWorker, &PingWorker::ack, this, &MasterConnectionStatusController::onNewLatency, Qt::QueuedConnection);
    _workerThread.start();

    Logger::logInfo(LogTag, "All ROS connections created");

    _disconnectWatchdogTimerId = -1;
    _connected = false;
    _bytesDown = _bytesUp = 0;
    _dataRateCalcTimerId = startTimer(_settings->getDataRateCalcInterval());
}

MasterConnectionStatusController::~MasterConnectionStatusController()
{
    if (_pingWorker) delete _pingWorker;
}

void MasterConnectionStatusController::onNewDataUpMessage(std_msgs::UInt32 msg)
{
    logDataUp(msg.data);
}

void MasterConnectionStatusController::onNewDataDownMessage(std_msgs::UInt32 msg)
{
    logDataDown(msg.data);
}

void MasterConnectionStatusController::onNewLatency(quint32 latency)
{
    if (_disconnectWatchdogTimerId != -1) {
        killTimer(_disconnectWatchdogTimerId);
    }
    _disconnectWatchdogTimerId = startTimer(_settings->getPingInterval() * 5);
    setConnected(true);

    std_msgs::UInt32 msg;
    msg.data = latency;
    _latencyPublisher.publish(msg);

    Q_EMIT latencyUpdate(latency);
}

void MasterConnectionStatusController::logDataDown(quint32 bytes)
{
    _bytesDown += bytes;
}

void MasterConnectionStatusController::logDataUp(quint32 bytes)
{
    _bytesUp += bytes;
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
    else if (e->timerId() == _dataRateCalcTimerId)
    {
        quint64 rateUp = (double)_bytesUp / ((double)_settings->getDataRateCalcInterval() / 1000.0);
        quint64 rateDown = (double)_bytesDown / ((double)_settings->getDataRateCalcInterval() / 1000.0);
        _bytesDown = _bytesUp = 0;

        ros_generated::data_rate msg;
        msg.dataRateDown = rateDown;
        msg.dataRateUp = rateUp;

        // Send data_rate message on data_rate topic
        _dataRatePublisher.publish(msg);

        Q_EMIT dataRateUpdate(rateUp, rateDown);
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
