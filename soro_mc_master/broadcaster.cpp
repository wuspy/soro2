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

#include "broadcaster.h"
#include "maincontroller.h"
#include "soro_core/constants.h"

#define LogTag "Broadcaster"

namespace Soro {

Broadcaster::Broadcaster(QObject *parent) : QObject(parent)
{
    _socket = new QUdpSocket(this);
    if (!_socket->bind(SORO_NET_MASTER_BROADCAST_PORT, QUdpSocket::ShareAddress))
    {
        MainController::panic(LogTag, QString("Could not bind to UDP port %1. Ensure no other master processes are running.").arg(SORO_NET_MASTER_BROADCAST_PORT));
    }
    _broadcastTimerId = startTimer(500);
}

void Broadcaster::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _broadcastTimerId)
    {
        _socket->writeDatagram(QByteArray("master"), QHostAddress::Broadcast, SORO_NET_MASTER_BROADCAST_PORT);
    }
}

} // namespace Soro
