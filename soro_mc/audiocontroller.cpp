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

#include "audiocontroller.h"
#include "maincontroller.h"
#include "soro_core/gstreamerutil.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "ros_generated/audio.h"

#include <Qt5GStreamer/QGst/Bus>

#define LogTag "AudioController"

namespace Soro {

AudioController::AudioController(QObject *parent) : QObject(parent)
{
    //
    // Setup audio_ack topic subscriber
    //
    Logger::logInfo(LogTag, "Creating ROS subscriber for audio_state topic...");
    _audioStateSubscriber = _nh.subscribe
            <ros_generated::audio, Soro::AudioController>
            ("audio_state", 1, &AudioController::onAudioResponse, this);
    if (!_audioStateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for audio_state topic");

    //
    // Setup audio topic publisher
    //
    Logger::logInfo(LogTag, "Creating ROS publisher for audio_request topic...");
    _audioStatePublisher = _nh.advertise<ros_generated::audio>("audio_request", 1);
    if (!_audioStatePublisher) MainController::panic(LogTag, "Failed to create ROS publisher for audio_request topic");

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");

    _codec = GStreamerUtil::CODEC_NULL;
    _pipelineWatch = nullptr;
}

AudioController::~AudioController()
{
    clearPipeline();
}

void AudioController::onAudioResponse(ros_generated::audio msg)
{
    // Rover is notifying us of a change in its audio stream
    clearPipeline();

    if (msg.on)
    {
        // Audio is streaming, create gstreamer pipeline
        _pipeline = QGst::Pipeline::create("audioPipeline");
        _bin = QGst::Bin::fromDescription(GStreamerUtil::createRtpAudioPlayString(QHostAddress::Any, SORO_NET_AUDIO_PORT, msg.encoding));
        _pipeline->add(_bin);

        // Add signal watch to subscribe to bus events, like errors
        _pipelineWatch = new GStreamerPipelineWatch(0, _pipeline, this);
        connect(_pipelineWatch, &GStreamerPipelineWatch::eos, this, &AudioController::gstEos);
        connect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::gstError);

        // Play
        _pipeline->setState(QGst::StatePlaying);
        _codec = msg.encoding;
        Q_EMIT playing(_codec);
    }
    else
    {
        // Audio is not streaming
        _codec = GStreamerUtil::CODEC_NULL;
        Q_EMIT stopped();
    }
}

bool AudioController::isPlaying() const
{
    return _codec != GStreamerUtil::CODEC_NULL;
}

quint8 AudioController::getCodec() const
{
    return _codec;
}

void AudioController::clearPipeline()
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
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::eos, this, &AudioController::gstEos);
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::gstError);
        delete _pipelineWatch;
        _pipelineWatch = nullptr;
    }
}

void AudioController::play(quint8 codec)
{
    // Send a request to the rover to start/change the audio stream
    ros_generated::audio msg;
    msg.encoding = codec;
    msg.on = true;

    Logger::logInfo(LogTag, QString("Sending audio ON requet to rover with codec %1").arg(GStreamerUtil::getCodecName(codec)));
    _audioStatePublisher.publish(msg);
}

void AudioController::stop()
{
    // Send a request to the rover to STOP the audio stream
    ros_generated::audio msg;
    msg.encoding = GStreamerUtil::CODEC_NULL;
    msg.on = false;

    Logger::logInfo(LogTag, "Sending audio OFF requet to rover");
    _audioStatePublisher.publish(msg);
}

} // namespace Soro
