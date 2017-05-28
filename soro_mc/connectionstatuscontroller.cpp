/*
 * Copyright 2017 Jacob Jordan <doublejinitials@ou.edu>
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
#include "soro_core/constants.h"
#include "soro_core/dataratemessage.h"
#include "soro_core/latencymessage.h"

#define LogTag "ConnectionStatusController"

namespace Soro {

ConnectionStatusController::ConnectionStatusController(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _connectionWatchdog.setInterval(2000);
    _connected = false;

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    _mqtt->setClientId(MainController::getId() + "_connectionstatuscontroller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->connectToHost();

    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        Logger::logInfo(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("latency", 0);
        _mqtt->subscribe("data_rate", 0);
    });

    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
        Logger::logInfo(LogTag, "Disconnected from to MQTT broker");
    });

    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message &msg)
    {
        if (msg.topic() == "latency")
        {
            if (!_connected)
            {
                _connected = true;
                Q_EMIT connectedChanged(true);
            }
            _connectionWatchdog.stop();
            _connectionWatchdog.start();

            LatencyMessage latencyMsg(msg.payload());
            Q_EMIT latencyUpdate(latencyMsg.latency);
        }
        else if (msg.topic() == "data_rate")
        {
            DataRateMessage dataRateMsg(msg.payload());
            Q_EMIT dataRateUpdate(dataRateMsg.dataRateFromRover);
        }
    });

    connect(&_connectionWatchdog, &QTimer::timeout, this, [this]()
    {
       if (_connected)
       {
           _connected = false;
           Q_EMIT connectedChanged(false);
       }
    });
}

bool ConnectionStatusController::isConnected() const
{
    return _mqtt->isConnectedToHost();
}

} // namespace Soro
