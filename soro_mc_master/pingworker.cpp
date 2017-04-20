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

#include "pingworker.h"
#include "libsoromc/logger.h"
#include "maincontroller.h"

#include <QDateTime>

#define LogTag "PingWorker"

namespace Soro {

PingWorker::PingWorker(uint interval, QObject *parent): QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS client for ping service...");
    _pingClient = _nh.serviceClient<ros_generated::ping>("ping");
    if (!_pingClient) MainController::panic(LogTag, "Failed to create ROS client for ping service");
    _interval = interval;
    _nextPing = 1;
    QTimer::singleShot(_interval, this, &PingWorker::work);
}

// REMINDER: This code is designed to execuce in a background thread!!!
void PingWorker::work()
{
    qint64 start = QDateTime::currentDateTime().toMSecsSinceEpoch();
    ros_generated::ping srv;
    srv.request.ping = _nextPing++;

    uint time = QDateTime::currentDateTime().toMSecsSinceEpoch() - start;
    if (_pingClient.call(srv) && (srv.response.ack == srv.request.ping))
    {
        Logger::logInfo(LogTag, "New latency: " + QString::number(time));
        Q_EMIT ack(time);
    }
    else
    {
        Logger::logWarn(LogTag, "Could not call ping service");
    }
    QTimer::singleShot(qMax<int>(_interval - time, 10), this, &PingWorker::work);
}

} // namespace Soro
