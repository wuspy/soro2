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

#include "masteraudiocontroller.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/addmediabouncemessage.h"

#include "maincontroller.h"

#define LogTag "MasterAudioController"

namespace Soro {

MasterAudioController::MasterAudioController(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;

    _audioSocket = new QUdpSocket(this);
    if (!_audioSocket->bind())
    {
        MainController::panic(LogTag, "Cannot bind UDP socket");
    }
    if (!_audioSocket->open(QIODevice::ReadWrite))
    {
        MainController::panic(LogTag, "Cannot open UDP socket");
    }

    connect(_audioSocket, &QUdpSocket::readyRead, this, [this]()
    {
        this->onSocketReadyRead(_audioSocket, SORO_NET_MC_AUDIO_PORT);
    });
    LOG_I(LogTag, "Bound UDP audio socket");

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &MasterAudioController::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &MasterAudioController::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &MasterAudioController::onMqttDisconnected);
    _mqtt->setClientId("master_audio_controller");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _announceTimerId = startTimer(1000);
}

void MasterAudioController::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "audio_bounce")
    {
        AddMediaBounceMessage bounceMsg(msg.payload());
        if (bounceMsg.address == QHostAddress::Null)
        {
            LOG_W(LogTag, "Got audio_bounce message with invalid address");
        }
        else
        {
            if (!_bounceMap.contains(bounceMsg.clientID))
            {
                LOG_I(LogTag, "Adding client " + bounceMsg.clientID + " at " + bounceMsg.address.toString() + " to audio bounce list");
                _bounceMap.insert(bounceMsg.clientID, bounceMsg.address);
                _bounceAddresses = _bounceMap.values();
                Q_EMIT bounceAddressesChanged(_bounceMap);
            }
        }
    }
    else if (msg.topic() == "system_down")
    {
        QString clientID(msg.payload());

        if (_bounceMap.contains(clientID))
        {
            LOG_I(LogTag, "Removing client " + clientID + " at " + _bounceMap.value(clientID).toString() + " from audio bounce list");
            _bounceMap.remove(clientID);
            _bounceAddresses = _bounceMap.values();
            Q_EMIT bounceAddressesChanged(_bounceMap);
        }
    }
}

void MasterAudioController::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("audio_bounce", 0);
    _mqtt->subscribe("system_down", 2);
}

void MasterAudioController::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
}

void MasterAudioController::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _announceTimerId)
    {
        _audioSocket->writeDatagram("audio", 6, _settings->getMqttBrokerAddress(), SORO_NET_AUDIO_PORT);
    }
}

void MasterAudioController::onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort)
{
    quint32 totalLen = 0;
    while (socket->hasPendingDatagrams())
    {
        qint64 len = socket->readDatagram(_buffer, 65536);
        for (QHostAddress bounceAddress : _bounceAddresses)
        {
            socket->writeDatagram(_buffer, len, bounceAddress, bouncePort);
        }
        totalLen += len;
    }
    Q_EMIT bytesDown(totalLen);
}

} // namespace Soro
