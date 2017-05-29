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

#include "mastervideoclient.h"
#include "soro_core/logger.h"
#include "soro_core/constants.h"
#include "soro_core/addmediabouncemessage.h"

#include "maincontroller.h"

#define LogTag "MasterVideoClient"

namespace Soro {

MasterVideoClient::MasterVideoClient(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QObject *parent) : QObject(parent)
{
    _cameraSettings = cameraSettings;
    _settings = settings;
    _nextMqttMsgId = 1;

    for (int i = 0; i < cameraSettings->getCameraCount(); i++)
    {
        QUdpSocket *socket = new QUdpSocket(this);

        if (!socket->bind())
        {
            MainController::panic(LogTag, "Cannot bind UDP socket");
        }
        if (!socket->open(QIODevice::ReadWrite))
        {
            MainController::panic(LogTag, "Cannot open UDP socket");
        }
        connect(socket, &QUdpSocket::readyRead, this, [this, socket, i]()
        {
            quint32 totalLen = 0;
            while (socket->hasPendingDatagrams())
            {
                qint64 len = socket->readDatagram(_buffer, 65536);
                if (len > 0)
                {
                    for (QHostAddress bounceAddress : _bounceAddresses)
                    {
                        socket->writeDatagram(_buffer, len, bounceAddress, SORO_NET_MC_FIRST_VIDEO_PORT + i);
                    }
                    totalLen += len;
                }
            }
            Q_EMIT bytesDown(totalLen);
        });
        _videoSockets.append(socket);
        LOG_I(LogTag, "Bound UDP video socket");
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &MasterVideoClient::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &MasterVideoClient::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &MasterVideoClient::onMqttDisconnected);
    _mqtt->setClientId("master_video_client");
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _announceTimerId = startTimer(1000);
}

void MasterVideoClient::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "video_bounce")
    {
        //
        // A video client is requesting video forwarding
        //
        AddMediaBounceMessage bounceMsg(msg.payload());
        if (bounceMsg.address == QHostAddress::Null)
        {
            LOG_W(LogTag, "Got video_bounce message with invalid address");
        }
        else
        {
            if (!_bounceMap.contains(bounceMsg.clientID))
            {
                LOG_I(LogTag, "Adding client " + bounceMsg.clientID + " at " + bounceMsg.address.toString() + " to video bounce list");
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
            //
            // A video client has exited, remove it from our forwarding list
            //
            LOG_I(LogTag, "Removing client " + clientID + " at " + _bounceMap.value(clientID).toString() + " from video bounce list");
            _bounceMap.remove(clientID);
            _bounceAddresses = _bounceMap.values();
            Q_EMIT bounceAddressesChanged(_bounceMap);
        }
        else if (clientID.startsWith("video_server_"))
        {
            bool ok;
            int serverIndex = clientID.mid(clientID.lastIndexOf("_") + 1).toInt(&ok);
            if (ok && serverIndex >= 0)
            {
                //
                // One of the video servers has gone down
                //
                LOG_W(LogTag, "Video server " + QString::number(serverIndex) + " has disconnected");
                for (int cameraIndex : _videoStateMessages.keys())
                {
                    if (_videoStateMessages[cameraIndex].camera_computerIndex == serverIndex)
                    {
                        _videoStateMessages[cameraIndex].profile.codec = GStreamerUtil::CODEC_NULL;
                        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++,
                                                      "video_state_" + QString::number(cameraIndex),
                                                      _videoStateMessages[cameraIndex],
                                                      2,
                                                      true)); // <-- Retain message
                    }
                }
            }
        }
    }
    else if (msg.topic().startsWith("video_state_"))
    {
        VideoMessage videoMsg(msg.payload());
        _videoStateMessages[videoMsg.camera_index] = videoMsg;
    }
}

void MasterVideoClient::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("video_bounce", 0);
    _mqtt->subscribe("system_down", 1);
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        _mqtt->subscribe("video_state_" + QString::number(i), 2);
    }
}

void MasterVideoClient::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
}

void MasterVideoClient::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _announceTimerId)
    {
        for (int i = 0; i < _videoSockets.size(); ++i)
        {
            _videoSockets[i]->writeDatagram("video", 6, _settings->getMqttBrokerAddress(), SORO_NET_FIRST_VIDEO_PORT + i);
        }
    }
}


} // namespace Soro
