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

#include "sciencepackagecontroller.h"
#include "maincontroller.h"
#include "soro_core/serialize.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/switchmessage.h"
#include "soro_core/geigermessage.h"
#include "soro_core/spectrometermessage.h"
#include "soro_core/compassmessage.h"
#include "soro_core/atmospheresensormessage.h"

#define LogTag "SciencePackageController"

namespace Soro
{

SciencePackageController::SciencePackageController(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _packageConnected = false;

    LOG_I(LogTag, "Creating UDP socket...");
    if (!_packageUdpSocket.bind(SORO_NET_SCIENCE_SYSTEM_PORT))
    {
        MainController::panic(LogTag, "Unable to bind science package UDP socket");
    }
    if (!_packageUdpSocket.open(QIODevice::ReadWrite))
    {
        MainController::panic(LogTag, "Unable to open science package UDP socket");
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    _mqtt->setClientId("science_package_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        LOG_I(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("compass", 0);
        _mqtt->subscribe("science_arm", 0);
        _mqtt->subscribe("atmosphere_switch", 2);
        _mqtt->subscribe("spectrometer_switch", 2);
        _mqtt->subscribe("geiger_switch", 2);
        _mqtt->subscribe("probe_switch", 2);
        _mqtt->subscribe("drill_switch", 2);
    });

    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
       LOG_W(LogTag, "Disconnected from MQTT broker");
    });

    _watchdogTimer.setInterval(1000); // Science package connection timeout

    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message& message)
    {
        if (message.topic() == "compass")
        {
            CompassMessage msg(message.payload());
            _lastCompassHeading = msg.heading;
        }
        else if (message.topic() == "science_arm")
        {
            // Retransmit this message over UDP to the science package microcontroller
            if (message.payload().at(0) != SORO_HEADER_SCIENCE_MASTER_ARM_MSG)
            {
                LOG_W(LogTag, "Received invalid MQTT master science arm message, discarding");
                return;
            }
            _packageUdpSocket.writeDatagram(message.payload(), QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
        else if (message.topic() == "atmosphere_switch")
        {
            SwitchMessage msg(message.payload());
            _buffer[0] = SORO_HEADER_SCIENCE_CONTROLLER_MSG;
            if (msg.on)
            {
                LOG_I(LogTag, "Setting atmosphere sensors ON");
                _buffer[1] = SORO_HEADER_SCIENCE_ATMOSPHERE_ON;
            }
            else
            {
                LOG_I(LogTag, "Setting atmosphere sensors OFF");
                _buffer[1] = SORO_HEADER_SCIENCE_ATMOSPHERE_OFF;
            }
            _packageUdpSocket.writeDatagram(_buffer, 3, QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
        else if (message.topic() == "geiger_switch")
        {
            SwitchMessage msg(message.payload());
            _buffer[0] = SORO_HEADER_SCIENCE_CONTROLLER_MSG;
            if (msg.on)
            {
                LOG_I(LogTag, "Setting geiger counter ON");
                _buffer[1] = SORO_HEADER_SCIENCE_GEIGER_ON;
            }
            else
            {
                LOG_I(LogTag, "Setting geiger counter OFF");
                _buffer[1] = SORO_HEADER_SCIENCE_GEIGER_OFF;
            }
            _packageUdpSocket.writeDatagram(_buffer, 3, QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
        else if (message.topic() == "spectrometer_switch")
        {
            SwitchMessage msg(message.payload());
            _buffer[0] = SORO_HEADER_SCIENCE_CONTROLLER_MSG;
            if (msg.on)
            {
                LOG_I(LogTag, "Setting spectrometer ON");
                _buffer[1] = SORO_HEADER_SCIENCE_SPEC_ON;
            }
            else
            {
                LOG_I(LogTag, "Setting spectrometer OFF");
                _buffer[1] = SORO_HEADER_SCIENCE_SPEC_OFF;
            }
            _packageUdpSocket.writeDatagram(_buffer, 3, QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
        else if (message.topic() == "probe_switch")
        {
            SwitchMessage msg(message.payload());
            _buffer[0] = SORO_HEADER_SCIENCE_CONTROLLER_MSG;
            if (msg.on)
            {
                LOG_I(LogTag, "Setting ground probe ON");
                _buffer[1] = SORO_HEADER_SCIENCE_PROBE_ON;
            }
            else
            {
                LOG_I(LogTag, "Setting ground probe OFF");
                _buffer[1] = SORO_HEADER_SCIENCE_PROBE_OFF;
            }
            _packageUdpSocket.writeDatagram(_buffer, 3, QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
        else if (message.topic() == "drill_switch")
        {
            SwitchMessage msg(message.payload());
            _buffer[0] = SORO_HEADER_SCIENCE_CONTROLLER_MSG;
            if (msg.on)
            {
                LOG_I(LogTag, "Setting core drill ON");
                _buffer[1] = SORO_HEADER_SCIENCE_DRILL_ON;
            }
            else
            {
                LOG_I(LogTag, "Setting core drill OFF");
                _buffer[1] = SORO_HEADER_SCIENCE_DRILL_OFF;
            }
            _packageUdpSocket.writeDatagram(_buffer, 3, QHostAddress::Broadcast, SORO_NET_SCIENCE_SYSTEM_PORT);
        }
    });

    connect(&_packageUdpSocket, &QUdpSocket::readyRead, this,[this]()
    {
        while (_packageUdpSocket.hasPendingDatagrams())
        {
            qint64 len = _packageUdpSocket.readDatagram(_buffer, USHRT_MAX);

            if (_buffer[0] != SORO_HEADER_SCIENCE_PACKAGE_MSG) return;

            if (!_packageConnected)
            {
                LOG_I(LogTag, "Science package is connected");
                _packageConnected = true;
                Q_EMIT sciencePackageConnectedChanged(true);
            }
            _watchdogTimer.stop();
            _watchdogTimer.start();

            if (_mqtt->isConnectedToHost())
            {
                if (len > 1) // If this is only a heartbeat, length will be 1
                {
                    if (_buffer[1] == SORO_HEADER_SCIENCE_ATMOSPHERE)
                    {
                        if (len >= 20)
                        {
                            AtmosphereSensorMessage msg;
                            msg.mq2Reading = deserialize<quint16>(_buffer + 2);
                            msg.mq4Reading = deserialize<quint16>(_buffer + 4);
                            msg.mq5Reading = deserialize<quint16>(_buffer + 6);
                            msg.mq6Reading = deserialize<quint16>(_buffer + 8);
                            msg.mq7Reading = deserialize<quint16>(_buffer + 10);
                            msg.mq9Reading = deserialize<quint16>(_buffer + 12);
                            msg.mq135Reading = deserialize<quint16>(_buffer + 14);
                            msg.co2Ppm = deserialize<quint32>(_buffer + 16);
                            msg.oxygenPercent = deserializeF(_buffer + 20);
                            msg.dustConcentration = deserializeF(_buffer + 28);
                            msg.temperature = deserializeF(_buffer + 36);
                            msg.humidity = deserializeF(_buffer + 44);
                            msg.windSpeed = deserializeF(_buffer + 52);
                            msg.windDirection = deserializeF(_buffer + 60);

                            // Wind direction is relative to the direction the rover is pointing
                            // We must factor in the compass direction of the rover to determine the
                            // true wind direction
                            msg.windDirection += _lastCompassHeading;
                            if (msg.windDirection > 360) msg.windDirection -= 360;

                            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "atmosphere", msg, 0));
                        }
                        else
                        {
                            LOG_W(LogTag, "Received UDP atmosphere sensor message of invalid size");
                        }
                    }
                    else if (_buffer[1] == SORO_HEADER_SCIENCE_GEIGER)
                    {
                        if (len >= 6)
                        {
                            GeigerMessage msg;
                            msg.countsPerMinute = deserialize<quint16>(_buffer + 2);
                            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "geiger", msg, 0));
                        }
                        else
                        {
                            LOG_W(LogTag, "Received UDP geiger message of invalid size");
                        }
                    }
                    else if (_buffer[1] == SORO_HEADER_SCIENCE_SPEC)
                    {
                        if (len >= 2 + SpectrometerMessage::SPECTRUM_POINTS * 4)
                        {
                            SpectrometerMessage msg;
                            for (int i = 0; i < SpectrometerMessage::SPECTRUM_POINTS * 2; i += 2)
                            {
                                msg.spectrumWhite.append(deserialize<quint16>(_buffer + 2 + i));
                            }
                            for (int i = 0; i < SpectrometerMessage::SPECTRUM_POINTS * 2; i += 2)
                            {
                                msg.spectrum404.append(deserialize<quint16>(_buffer + 2 + (SpectrometerMessage::SPECTRUM_POINTS * 2) + i));
                            }
                            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "spectrometer", msg, 0));
                        }
                        else
                        {
                            LOG_W(LogTag, "Received UDP spectrometer message of invalid size");
                        }
                    }
                    else
                    {
                        LOG_W(LogTag, "Received invalid UDP science package message, discarding");
                    }
                }
            }
        }
    });

    connect(&_watchdogTimer, &QTimer::timeout, this, [this]()
    {
       if (_packageConnected)
       {
           LOG_W(LogTag, "Science package is no longer connected");
           _packageConnected = false;
           Q_EMIT sciencePackageConnectedChanged(false);

           // Publish this disconnection event on the system_down topic since the science package microcontroller
           // does not have MQTT capability
           _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "system_down", QByteArray("science_package"), 2, false));
       }
    });
}

} // namespace Soro
