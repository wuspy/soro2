/*
 * Copyright 2017 Stephen Nickle <stephen.nickle@ou.edu>
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

#include "armcontroller.h"
#include "maincontroller.h"
#include "soro_core/serialize.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"

#define LogTag "ArmController"

namespace Soro
{

ArmController::ArmController(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _armConnected = false;

    LOG_I(LogTag, "Creating UDP socket...");
    if (!_armUdpSocket.bind(SORO_NET_SLAVE_ARM_PORT))
    {
        MainController::panic(LogTag, "Unable to bind arm UDP socket");
    }
    if (!_armUdpSocket.open(QIODevice::ReadWrite))
    {
        MainController::panic(LogTag, "Unable to open arm UDP socket");
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    _mqtt->setClientId("arm_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _watchdogTimer.setInterval(1000); // Arm connection timeout

    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        LOG_I(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("arm", 0);
    });

    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
       LOG_W(LogTag, "Disconnected from MQTT broker");
    });

    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message& message)
    {
        if (message.topic() == "arm")
        {
            // Retransmit this message over UDP to the science package microcontroller
            if (message.payload().at(0) != SORO_HEADER_MASTER_ARM_MSG)
            {
                LOG_W(LogTag, "Received invalid MQTT master arm message, discarding");
                return;
            }
            _armUdpSocket.writeDatagram(message.payload(), QHostAddress::Broadcast, SORO_NET_SLAVE_ARM_PORT);
        }
    });

    connect(&_armUdpSocket, &QUdpSocket::readyRead, this,[this]()
    {
        while (_armUdpSocket.hasPendingDatagrams())
        {
            qint64 len = _armUdpSocket.readDatagram(_buffer, USHRT_MAX);

            if (_buffer[0] != SORO_HEADER_SLAVE_ARM_MSG) return;

            if (!_armConnected)
            {
                LOG_I(LogTag, "Arm is connected");
                _armConnected = true;
                Q_EMIT armConnectedChanged(true);
            }
            _watchdogTimer.stop();
            _watchdogTimer.start();
        }
    });

    connect(&_watchdogTimer, &QTimer::timeout, this, [this]()
    {
       if (_armConnected)
       {
           LOG_W(LogTag, "Arm is no longer connected");
           _armConnected = false;
           Q_EMIT armConnectedChanged(false);

           // Publish this disconnection event on the system_down topic since the arm microcontroller
           // does not have MQTT capability
           _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "system_down", QByteArray("arm"), 2, false));
       }
    });
}

} // namespace Soro
