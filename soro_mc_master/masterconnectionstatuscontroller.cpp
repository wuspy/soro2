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
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/latencymessage.h"
#include "soro_core/dataratemessage.h"
#include "soro_core/pingmessage.h"

#include <QDateTime>

#define LogTag "MasterConnectionStatusController"

namespace Soro {

MasterConnectionStatusController::MasterConnectionStatusController
            (QHostAddress brokerAddress, quint16 brokerPort, int pingInterval, int dataRateCalcInterval, QObject *parent) : QObject(parent)
{
    _pingInterval = pingInterval;
    _dataRateCalcInterval = dataRateCalcInterval;
    _bytesDown = _bytesUp = 0;
    _nextPing = 1;
    _dataRateCalcTimer.start(dataRateCalcInterval);
    _pingTimer.start(pingInterval);

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(brokerAddress, brokerPort, this);
    _mqtt->setClientId("master_connection_status_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    connect(_mqtt, &QMQTT::Client::connected, this, [this]()
    {
        _mqtt->subscribe("ping", 0);
        Q_EMIT mqttConnected();
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

    connect(&_dataRateCalcTimer, &QTimer::timerId, this, [this]()
    {
        quint64 rateUp = (float)_bytesUp / ((double)_dataRateCalcInterval / 1000.0f);
        quint64 rateDown = (double)_bytesDown / ((double)_dataRateCalcInterval / 1000.0f);
        _bytesDown = _bytesUp = 0;

        // Send data_rate message on data_rate topic
        DataRateMessage msg;
        msg.dataRateDown = rateDown;
        msg.dataRateUp = rateUp;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "data_rate", msg, 0));

        Q_EMIT dataRateUpdate(rateUp, rateDown);
    });
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
    return _mqtt->isConnectedToHost();
}

} // namespace Soro
