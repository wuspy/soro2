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

#include "armcontrolsystem.h"
#include "maincontroller.h"
#include "soro_core/armmessage.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#define LogTag "ArmControlSystem"

namespace Soro
{

ArmControlSystem::ArmControlSystem(QHostAddress brokerAddress, quint16 brokerPort, int masterArmConnectionTimeout, QObject *parent) : QObject(parent)
{
    _enabled = false;

    LOG_I(LogTag, "Creating UDP socket...");
    if (!_armUdpSocket.bind(SORO_NET_MASTER_ARM_PORT))
    {
        MainController::panic(LogTag, "Unable to bind master arm UDP socket");
    }
    if (!_armUdpSocket.open(QIODevice::ReadWrite))
    {
        MainController::panic(LogTag, "Unable to open master arm UDP socket");
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(brokerAddress, brokerPort, this);
    _mqtt->setClientId("arm_control_system");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _watchdogTimer.setInterval(masterArmConnectionTimeout);

    connect(&_armUdpSocket, &QUdpSocket::readyRead, this,[this]()
    {
        while (_armUdpSocket.hasPendingDatagrams())
        {
            qint64 len = _armUdpSocket.readDatagram(_buffer, USHRT_MAX);
            if (_buffer[0] != 'm')
            {
                Logger::logWarn(LogTag, "Received invalid master arm message, discarding");
                return;
            }
            if (!_masterConnected)
            {
                Logger::logInfo(LogTag, "Master arm is connected");
                _masterConnected = true;
                Q_EMIT masterArmConnectedChanged(true);
            }
            _watchdogTimer.stop();
            _watchdogTimer.start();

            if (_enabled && _mqtt->isConnectedToHost())
            {
                if (len > 1) // If this is only a heartbeat, length will be 1
                {
                    ArmMessage msg(QByteArray(_buffer, len));
                    _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "arm", msg, 0));
                }
            }
        }
    });

    connect(&_watchdogTimer, &QTimer::timeout, this, [this]()
    {
       if (_masterConnected)
       {
           Logger::logWarn(LogTag, "Master arm is no longer connected");
           _masterConnected = false;
           Q_EMIT masterArmConnectedChanged(false);
       }
    });
}

void ArmControlSystem::enable()
{
    _enabled = true;
}

void ArmControlSystem::disable()
{
    _enabled = false;
}

}
