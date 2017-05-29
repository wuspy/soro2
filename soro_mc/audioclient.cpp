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

#include "audioclient.h"
#include "maincontroller.h"
#include "soro_core/gstreamerutil.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/audiomessage.h"
#include "soro_core/addmediabouncemessage.h"

#include <Qt5GStreamer/QGst/Bus>
#include <QNetworkInterface>

#define LogTag "AudioController"

namespace Soro {

AudioClient::AudioClient(const SettingsModel *settings, QObject *parent) : QObject(parent)
{
    _settings = settings;

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &AudioClient::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &AudioClient::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &AudioClient::onMqttDisconnected);
    _mqtt->setClientId("audio_client_" + MainController::getId());
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(2);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _pipelineWatch = nullptr;

    _announceTimerId = startTimer(1000);
}

AudioClient::~AudioClient()
{
    clearPipeline();
}

void AudioClient::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    _mqtt->subscribe("audio_state", 0);
    _mqtt->subscribe("system_down", 2);
    Q_EMIT mqttConnected();
}

void AudioClient::onMqttDisconnected()
{
    LOG_I(LogTag, "Disconnected from MQTT broker");
    Q_EMIT mqttDisconnected();
}

void AudioClient::onMqttMessage(const QMQTT::Message &msg)
{
    if (msg.topic() == "audio_state")
    {
        // Rover is notifying us of a change in its audio stream
        clearPipeline();
        AudioMessage audioMsg(msg.payload());

        _profile = audioMsg.profile;

        if (_profile.codec != GStreamerUtil::CODEC_NULL)
        {
            // Audio is streaming, create gstreamer pipeline
            _pipeline = QGst::Pipeline::create("audioPipeline");
            _bin = QGst::Bin::fromDescription(GStreamerUtil::createRtpAudioPlayString(QHostAddress::Any, SORO_NET_MC_AUDIO_PORT, _profile.codec));
            _pipeline->add(_bin);

            // Add signal watch to subscribe to bus events, like errors
            _pipelineWatch = new GStreamerPipelineWatch(0, _pipeline, this);
            connect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioClient::gstError);

            // Play
            _pipeline->setState(QGst::StatePlaying);
            Q_EMIT playing(_profile);
        }
        else
        {
            // Audio is not streaming
            Q_EMIT stopped();
        }
    }
    else if (msg.topic() == "system_down")
    {
        QString client = QString(msg.payload());
        if (client == "audio_server")
        {
            LOG_W(LogTag, "Audio server has disconnected");
            Q_EMIT audioServerDisconnected();
        }
        else if (client == "master_audio_client")
        {
            LOG_W(LogTag, "Master audio client has disconnected");
            Q_EMIT masterAudioClientDisconnected();
        }
    }
}

bool AudioClient::isPlaying() const
{
    return _profile.codec != GStreamerUtil::CODEC_NULL;
}

GStreamerUtil::AudioProfile AudioClient::getProfile() const
{
    return _profile;
}

void AudioClient::clearPipeline()
{
    if (!_pipeline.isNull())
    {
        _pipeline->bus()->removeSignalWatch();
        _pipeline->setState(QGst::StateNull);
        _pipeline.clear();
        _bin.clear();
    }
    if (_pipelineWatch)
    {
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioClient::gstError);
        delete _pipelineWatch;
        _pipelineWatch = nullptr;
    }
}

void AudioClient::play(GStreamerUtil::AudioProfile profile)
{
    // Send a request to the rover to start/change the audio stream

    if (_mqtt->isConnectedToHost())
    {
        LOG_I(LogTag, QString("Sending audio ON requet to rover with codec %1").arg(GStreamerUtil::getCodecName(profile.codec)));
        _profile = profile;

        AudioMessage msg;
        msg.profile = _profile;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "audio_request", msg, 2));
    }
    else
    {
        LOG_E(LogTag, "Failed to send audio ON request to rover - mqtt not connected");
    }
}

void AudioClient::stop()
{
    // Send a request to the rover to STOP the audio stream
    if (_mqtt->isConnectedToHost())
    {
        LOG_I(LogTag, "Sending audio OFF requet to rover");
        _profile = GStreamerUtil::AudioProfile();

        AudioMessage msg;
        msg.profile = _profile;
        _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "audio_request", msg, 2));
    }
    else
    {
        LOG_E(LogTag, "Failed to send audio OFF request to rover - mqtt not connected");
    }
}

void AudioClient::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _announceTimerId)
    {
        // Publish our address so we get an audio stream forwarded to us
        for (const QHostAddress &address : QNetworkInterface::allAddresses())
        {
            if ((address.protocol() == QAbstractSocket::IPv4Protocol) && (address != QHostAddress::LocalHost))
            {
                AddMediaBounceMessage msg;
                msg.address = address;
                msg.clientID = _mqtt->clientId();

                _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "audio_bounce", msg.serialize(), 0));
                break;
            }
        }
    }
}

} // namespace Soro
