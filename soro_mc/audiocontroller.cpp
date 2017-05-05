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
    _profile = GStreamerUtil::AudioProfile(msg);

    if (_profile.codec != GStreamerUtil::CODEC_NULL)
    {
        // Audio is streaming, create gstreamer pipeline
        _pipeline = QGst::Pipeline::create("audioPipeline");
        _bin = QGst::Bin::fromDescription(GStreamerUtil::createRtpAudioPlayString(QHostAddress::Any, SORO_NET_MC_AUDIO_PORT, _profile.codec));
        _pipeline->add(_bin);

        // Add signal watch to subscribe to bus events, like errors
        _pipelineWatch = new GStreamerPipelineWatch(0, _pipeline, this);
        connect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::gstError);

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

bool AudioController::isPlaying() const
{
    return _profile.codec != GStreamerUtil::CODEC_NULL;
}

GStreamerUtil::AudioProfile AudioController::getProfile() const
{
    return _profile;
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
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::gstError);
        delete _pipelineWatch;
        _pipelineWatch = nullptr;
    }
}

void AudioController::play(GStreamerUtil::AudioProfile profile)
{
    // Send a request to the rover to start/change the audio stream

    Logger::logInfo(LogTag, QString("Sending audio ON requet to rover with codec %1").arg(GStreamerUtil::getCodecName(profile.codec)));
    _profile = profile;
    _audioStatePublisher.publish(_profile.toRosMessage());
}

void AudioController::stop()
{
    // Send a request to the rover to STOP the audio stream

    Logger::logInfo(LogTag, "Sending audio OFF requet to rover");
    _profile = GStreamerUtil::AudioProfile();
    _audioStatePublisher.publish(_profile.toRosMessage());
}

} // namespace Soro
