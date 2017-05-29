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

#include "masterconnectionstatuscontroller.h"
#include "maincontroller.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/latencymessage.h"
#include "soro_core/dataratemessage.h"
#include "soro_core/pingmessage.h"

#include <QDateTime>

#define LogTag "MasterConnectionStatusController"

namespace Soro {

MasterConnectionStatusController::MasterConnectionStatusController
            (const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;
    _bytesFromRover = 0;
    _nextPing = 1;
    _dataRateCalcTimer.start(settings->getDataRateCalcInterval());
    _pingTimer.start(settings->getPingInterval());

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    _mqtt->setClientId("master_connection_status_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        LOG_I(LogTag, "Connected to MQTT broker");
        _mqtt->subscribe("ping", 0);
        Q_EMIT connectedChanged(true);
    });

    connect(_mqtt, &QMQTT::Client::disconnected, this, [this]()
    {
       LOG_W(LogTag, "Disconnected from MQTT broker");
       Q_EMIT connectedChanged(false);
    });

    connect(_mqtt, &QMQTT::Client::received, this, [this](const QMQTT::Message &msg)
    {
        if (msg.topic() == "ping")
        {
            PingMessage pingMsg(msg.payload());

            // Remove all ping records older than this one
            while ((_pingIdQueue.first() != pingMsg.pingId) && !_pingIdQueue.isEmpty())
            {
                _pingIdQueue.dequeue();
                _pingTimeQueue.dequeue();
            }

            if (!_pingIdQueue.isEmpty())
            {
                // Calculate time difference
                _pingIdQueue.dequeue();
                qint64 diff = QDateTime::currentDateTime().toMSecsSinceEpoch() - _pingTimeQueue.dequeue();

                // Publish this new latency to latency topic
                LatencyMessage latencyMsg;
                latencyMsg.latency = diff;
                _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "latency", latencyMsg, 0));

                Q_EMIT latencyUpdate((quint32)diff);
            }
        }
    });

    connect(&_pingTimer, &QTimer::timeout, this, [this]()
    {
        _pingIdQueue.enqueue(_nextPing);
        _pingTimeQueue.enqueue(QDateTime::currentDateTime().toMSecsSinceEpoch());

        PingMessage msg;
        msg.pingId = _nextPing;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "ping", msg, 0));
        _nextPing++;
    });

    connect(&_dataRateCalcTimer, &QTimer::timeout, this, [this]()
    {
        quint64 rate = (float)_bytesFromRover / ((float)_settings->getDataRateCalcInterval() / 1000.0f);
        _bytesFromRover = 0;

        // Send data_rate message on data_rate topic
        DataRateMessage msg;
        msg.dataRateFromRover = rate;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "data_rate", msg, 0));

        Q_EMIT dataRateUpdate(rate);
    });
}

void MasterConnectionStatusController::logDataFromRover(quint32 bytes)
{
    _bytesFromRover += bytes;
}

bool MasterConnectionStatusController::isConnected() const
{
    return _mqtt->isConnectedToHost();
}

} // namespace Soro
