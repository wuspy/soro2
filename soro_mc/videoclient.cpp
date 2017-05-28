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

#include "videoclient.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"
#include "soro_core/videomessage.h"
#include "soro_core/videostatemessage.h"
#include "soro_core/addmediabouncemessage.h"

#include <Qt5GStreamer/QGst/Bus>

#include <QNetworkInterface>

#include "maincontroller.h"

#define LogTag "VideoClient"

namespace Soro {

VideoClient::VideoClient(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QVector<QGst::ElementPtr> sinks, QObject *parent) : QObject(parent)
{
    _settings = settings;
    _cameraSettings = cameraSettings;
    _sinks = sinks;

    for (int i = 0; i < cameraSettings->getCameraCount(); ++i)
    {
        _pipelines.append(QGst::PipelinePtr());
        _bins.append(QGst::BinPtr());
        _pipelineWatches.append(nullptr);
        _videoStates.append(GStreamerUtil::VideoProfile());

        stopVideoOnSink(i);
    }

    LOG_I(LogTag, "Creating MQTT client...");
    _mqtt = new QMQTT::Client(settings->getMqttBrokerAddress(), SORO_NET_MQTT_BROKER_PORT, this);
    connect(_mqtt, &QMQTT::Client::received, this, &VideoClient::onMqttMessage);
    connect(_mqtt, &QMQTT::Client::connected, this, &VideoClient::onMqttConnected);
    connect(_mqtt, &QMQTT::Client::disconnected, this, &VideoClient::onMqttDisconnected);
    _mqtt->setClientId("video_client_" + MainController::getId());
    _mqtt->setAutoReconnect(true);
    _mqtt->setAutoReconnectInterval(1000);
    _mqtt->setWillMessage(_mqtt->clientId());
    _mqtt->setWillQos(1);
    _mqtt->setWillTopic("system_down");
    _mqtt->setWillRetain(false);
    _mqtt->connectToHost();

    _announceTimerId = startTimer(1000);
}

VideoClient::~VideoClient()
{
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        clearPipeline(i);
    }
}

void VideoClient::onMqttConnected()
{
    LOG_I(LogTag, "Connected to MQTT broker");
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        _mqtt->subscribe("video_state_" + QString::number(i), 1);
    }
    _mqtt->subscribe("system_down", 2);
}

void VideoClient::onMqttDisconnected()
{
    LOG_W(LogTag, "Disconnected from MQTT broker");
    // Fill video state lists with default values
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        _videoStates.append(GStreamerUtil::VideoProfile());
        stopVideoOnSink(i);
    }
}

void VideoClient::onMqttMessage(const QMQTT::Message &msg)
{
    LOG_I(LogTag, "Receiving MQTT message");
    if (msg.topic().startsWith("video_state_"))
    {
        VideoMessage videoMsg(msg.payload());

        LOG_I(LogTag, "Received video state message for camera " + QString::number(videoMsg.camera_index));
        if (videoMsg.camera_index < _videoStates.length())
        {
            if (videoMsg.profile != _videoStates.value(videoMsg.camera_index))
            {
                // Video state has changed
                _videoStates[videoMsg.camera_index] = videoMsg.profile;
                if (videoMsg.profile.codec != GStreamerUtil::CODEC_NULL)
                {
                    // Video is streaming
                    playVideoOnSink(videoMsg.camera_index, videoMsg.profile);
                }
                else
                {
                    // Video is NOT streaming
                    stopVideoOnSink(videoMsg.camera_index);
                }
            }
        }
        else
        {
            LOG_E(LogTag, "Received video_state message for camera " + QString::number(videoMsg.camera_index) + ", but this number is too big to be a camera index that we know of");
        }
    }
    else if (msg.topic() == "system_down")
    {
        QString client = QString(msg.payload());
        if (client.startsWith("video_server_"))
        {
            bool ok;
            int serverIndex = client.mid(client.lastIndexOf("_") + 1).toInt(&ok);
            if (ok && serverIndex >= 0)
            {
                LOG_W(LogTag, "Video server " + QString::number(serverIndex) + " has disconnected");
                for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
                {
                    if (_cameraSettings->getCamera(i).computerIndex == serverIndex)
                    {
                        stopVideoOnSink(i);
                    }
                }
                Q_EMIT videoServerDisconnected(serverIndex);
            }
        }
        else if (client == "master_video_client")
        {
            LOG_W(LogTag, "Master video client has disconnected");
            Q_EMIT masterVideoClientDisconnected();
        }
    }
}

void VideoClient::play(uint cameraIndex, GStreamerUtil::VideoProfile profile)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        if (_mqtt->isConnectedToHost())
        {
            // Send a request to the rover to start/change a video stream
            VideoMessage msg(cameraIndex, _cameraSettings->getCamera(cameraIndex));
            msg.profile = profile;

            LOG_I(LogTag, QString("Sending video ON request to the rover for camera %1: [Codec %2, %3x%4, %5fps, %6bps, %7q]").arg(
                                QString::number(cameraIndex), QString::number(profile.codec), QString::number(profile.width), QString::number(profile.height),
                                QString::number(profile.framerate), QString::number(profile.bitrate), QString::number(profile.mjpeg_quality)));

            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "video_request", msg, 0));
        }
        else
        {
            LOG_E(LogTag, "Failed to send video ON request to rover - mqtt not connected");
        }
    }
}

void VideoClient::constructPipelineOnSink(uint cameraIndex, QString sourceBinString)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        clearPipeline(cameraIndex);

        LOG_I(LogTag, QString("Starting pipeline '%1' on sink %2").arg(sourceBinString, QString::number(cameraIndex)));
        _pipelines[cameraIndex] = QGst::Pipeline::create(QString("video%1Pipeline").arg(cameraIndex).toLatin1().constData());
        _bins[cameraIndex] = QGst::Bin::fromDescription(sourceBinString);

        if (_sinks[cameraIndex].isNull())
        {
            LOG_E(LogTag, QString("Supplied sink for video %1 is NULL").arg(cameraIndex));
            Q_EMIT gstError("Supplied sink is null", cameraIndex);
            return;
        }
        if (_pipelines[cameraIndex].isNull())
        {
            LOG_E(LogTag, QString("Failed to create pipeline video%1Pipeline").arg(cameraIndex));
            Q_EMIT gstError("Failed to create pipeline", cameraIndex);
            return;
        }
        if (_bins[cameraIndex].isNull())
        {
            LOG_E(LogTag, QString("Failed to create binary '%1' for video%2Pipeline").arg(sourceBinString, QString::number(cameraIndex)));
            Q_EMIT gstError(QString("Failed to create binary '%1'").arg(sourceBinString), cameraIndex);
            return;
        }

        _pipelines[cameraIndex]->add(_bins[cameraIndex], _sinks[cameraIndex]);
        _bins[cameraIndex]->link(_sinks[cameraIndex]);

        _pipelineWatches[cameraIndex] = new GStreamerPipelineWatch(cameraIndex, _pipelines[cameraIndex], this);
        connect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoClient::gstError);

        _pipelines[cameraIndex]->setState(QGst::StatePlaying);
    }
}

void VideoClient::stopVideoOnSink(uint cameraIndex)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Stop the video on the specified sink, and play a placeholder animation
        LOG_I(LogTag, "Stopping video " + QString::number(cameraIndex));
        constructPipelineOnSink(cameraIndex, GStreamerUtil::createVideoTestSrcString("smpte", true, 800, 600, 10));
        _videoStates[cameraIndex] = GStreamerUtil::VideoProfile();
        Q_EMIT stopped(cameraIndex);
    }
}

void VideoClient::playVideoOnSink(uint cameraIndex, GStreamerUtil::VideoProfile profile)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Play the video on the specified sink
        LOG_I(LogTag, "Playing video " + QString::number(cameraIndex) + " with codec " + GStreamerUtil::getCodecName(profile.codec));
        constructPipelineOnSink(cameraIndex, GStreamerUtil::createRtpVideoDecodeString(
                                    QHostAddress::Any,
                                    SORO_NET_MC_FIRST_VIDEO_PORT + cameraIndex,
                                    profile.codec,
                                    _settings->getEnableHwDecoding()));
        _videoStates[cameraIndex] = profile;
        Q_EMIT playing(cameraIndex, profile);
    }
}

void VideoClient::stop(uint cameraIndex)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        if (_mqtt->isConnectedToHost())
        {
            // Send a request to the rover to stop a video stream
            VideoMessage msg(cameraIndex, _cameraSettings->getCamera(cameraIndex));
            msg.profile = GStreamerUtil::VideoProfile();

            LOG_I(LogTag, QString("Sending video OFF request to the rover for camera %1").arg(cameraIndex));

            _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "video_request", msg, 0));
        }
        else
        {
            LOG_E(LogTag, "Failed to send video OFF request to rover - mqtt not connected");
        }
        stopVideoOnSink(cameraIndex);
    }
}

void VideoClient::stopAll()
{
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        stop(i);
    }
}

void VideoClient::clearPipeline(uint cameraIndex)
{
    if (!_pipelines.value(cameraIndex).isNull())
    {
        _pipelines[cameraIndex]->setState(QGst::StateNull);
        _pipelines[cameraIndex].clear();
        _bins[cameraIndex].clear();
    }
    if (_pipelineWatches.value(cameraIndex) != nullptr)
    {
        disconnect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoClient::gstError);
        delete _pipelineWatches[cameraIndex];
        _pipelineWatches[cameraIndex] = nullptr;
    }
}

void VideoClient::timerEvent(QTimerEvent *e)
{
    if (e->timerId() == _announceTimerId)
    {
        // Publish our address so we get a video stream forwarded to us
        for (const QHostAddress &address : QNetworkInterface::allAddresses())
        {
            if ((address.protocol() == QAbstractSocket::IPv4Protocol) && (address != QHostAddress::LocalHost))
            {
                AddMediaBounceMessage msg;
                msg.address = address;
                msg.clientID = _mqtt->clientId();

                _mqtt->publish(QMQTT::Message(_nextMqttMsgId++, "video_bounce", msg.serialize(), 0));
                break;
            }
        }
    }
}

bool VideoClient::isPlaying(uint cameraIndex) const
{
    return _videoStates.value(cameraIndex).codec != GStreamerUtil::CODEC_NULL;
}

GStreamerUtil::VideoProfile VideoClient::getVideoProfile(uint cameraIndex) const
{
    return _videoStates.value(cameraIndex);
}

} // namespace Soro
