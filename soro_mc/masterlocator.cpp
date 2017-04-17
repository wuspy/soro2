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

#include "masterlocator.h"
#include "libsoromc/logger.h"
#include "maincontroller.h"

#define LogTag "MasterLocator"

namespace Soro {

MasterLocator::MasterLocator(QObject *parent) : QObject(parent)
{
    _socket = new QUdpSocket(this);
    if (!_socket->bind(SORO_MC_MASTER_BROADCAST_PORT, QUdpSocket::ShareAddress))
    {
        MainController::panic(LogTag, QString("Cannot bind to UDP port %1 to receive master broadcast").arg(SORO_MC_MASTER_BROADCAST_PORT));
    }
    connect(_socket, &QUdpSocket::readyRead, this, &MasterLocator::udpReadyRead);
}

void MasterLocator::udpReadyRead()
{
    while (_socket->hasPendingDatagrams())
    {
        char data[100];
        QHostAddress address;
        quint16 port;
        qint64 len = _socket->readDatagram(data, 100, &address, &port);

        if (strncmp(data, "master", qMax(strlen("master"), (size_t)len)) == 0)
        {
            // Found master
            disconnect(_socket, &QUdpSocket::readyRead, this, &MasterLocator::udpReadyRead);
            delete _socket;
            _socket = nullptr;
            if (address.protocol() == QAbstractSocket::IPv6Protocol)
            {
                // We got an IPv6 address, which ROS won't like. Hopefully we can convert it to IPv4
                Logger::logWarn(LogTag, "The master appears to be at an IPv6 address, trying to convert it to IPv4...");
                bool ok;
                QHostAddress addressv4 = QHostAddress(address.toIPv4Address(&ok));
                if (!ok) {
                    MainController::panic(LogTag, QString("The master appears to be at IPv6 address %1, which cannot be converted to an IPv4 address. An IPv6 address cannot be used as a ROS master.").arg(address.toString()));
                }
                Logger::logInfo(LogTag, QString("Found master at %1").arg(addressv4.toString()));
                Q_EMIT masterFound(addressv4);
            }
            else
            {
                Logger::logInfo(LogTag, QString("Found master at %1").arg(address.toString()));
                Q_EMIT masterFound(address);
            }
        }
        else
        {
            Logger::logError(LogTag, "Got invalid message on mission control broadcast port");
        }
    }
}

} // namespace Soro
