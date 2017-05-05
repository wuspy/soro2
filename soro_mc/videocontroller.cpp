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

#include "videocontroller.h"
#include "soro_core/constants.h"
#include "soro_core/logger.h"
#include "soro_core/gstreamerutil.h"

#include <Qt5GStreamer/QGst/Bus>

#include "maincontroller.h"

#define LogTag "VideoController"

namespace Soro {

VideoController::VideoController(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QVector<QGst::ElementPtr> sinks, QObject *parent) : QObject(parent)
{
    _settings = settings;
    _cameraSettings = cameraSettings;
    _sinks = sinks;

    // Fill video state lists with default values
    for (int i = 0; i < cameraSettings->getCameraCount(); ++i)
    {
        _pipelines.append(QGst::PipelinePtr());
        _bins.append(QGst::BinPtr());
        _pipelineWatches.append(nullptr);
        _videoStates.append(GStreamerUtil::VideoProfile());

        stopVideoOnSink(i);
    }

    Logger::logInfo(LogTag, "Creating ROS subscriber for video_state topic...");
    _videoStateSubscriber = _nh.subscribe
            <ros_generated::video_state, Soro::VideoController>
            ("video_state", 1, &VideoController::onVideoResponse, this);
    if (!_videoStateSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_state topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for video_request topic...");
    _videoRequestPublisher = _nh.advertise<ros_generated::video>("video_request", 100);
    if (!_videoRequestPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for video_request topic");

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");
}

VideoController::~VideoController()
{
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        clearPipeline(i);
    }
}

void VideoController::onVideoResponse(ros_generated::video_state msg)
{
    Logger::logInfo(LogTag, "Received video_state message");
    for (int i = 0; i < _videoStates.size(); ++i)
    {
        GStreamerUtil::VideoProfile newState = GStreamerUtil::VideoProfile();
        GStreamerUtil::VideoProfile oldState = _videoStates.value(i);

        for (ros_generated::video videoMsg : msg.videoStates)
        {
            if (videoMsg.camera_index == i)
            {
                newState = GStreamerUtil::VideoProfile(videoMsg);
                break;
            }
        }

        if (newState != oldState)
        {
            // Video state has changed
            _videoStates[i] = newState;
            if (newState.codec != GStreamerUtil::CODEC_NULL)
            {
                // Video is streaming
                playVideoOnSink(i, newState);
            }
            else
            {
                // Video is NOT streaming
                stopVideoOnSink(i);
            }
        }
    }
}

void VideoController::play(uint cameraIndex, GStreamerUtil::VideoProfile profile)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Send a request to the rover to start/change a video stream
        ros_generated::video msg = profile.toRosMessage();
        msg.camera_computerIndex = _cameraSettings->getCamera(cameraIndex).computerIndex;
        msg.camera_offset = _cameraSettings->getCamera(cameraIndex).offset;
        msg.camera_matchSerial = _cameraSettings->getCamera(cameraIndex).serial.toStdString();
        msg.camera_matchProductId = _cameraSettings->getCamera(cameraIndex).productId.toStdString();
        msg.camera_matchVendorId = _cameraSettings->getCamera(cameraIndex).vendorId.toStdString();
        msg.camera_name = _cameraSettings->getCamera(cameraIndex).name.toStdString();
        msg.camera_index = cameraIndex;

        Logger::logInfo(LogTag, QString("Sending video ON request to the rover for camera %1: [Codec %2, %3x%4, %5fps, %6bps, %7q]").arg(
                            QString::number(cameraIndex), QString::number(profile.codec), QString::number(profile.width), QString::number(profile.height),
                            QString::number(profile.framerate), QString::number(profile.bitrate), QString::number(profile.mjpeg_quality)));
        _videoRequestPublisher.publish(msg);
    }
}

void VideoController::constructPipelineOnSink(uint cameraIndex, QString sourceBinString)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        clearPipeline(cameraIndex);

        Logger::logInfo(LogTag, QString("Starting pipeline '%1' on sink %2").arg(sourceBinString, QString::number(cameraIndex)));
        _pipelines[cameraIndex] = QGst::Pipeline::create(QString("video%1Pipeline").arg(cameraIndex).toLatin1().constData());
        _bins[cameraIndex] = QGst::Bin::fromDescription(sourceBinString);

        if (_sinks[cameraIndex].isNull())
        {
            Logger::logError(LogTag, QString("Supplied sink for video %1 is NULL").arg(cameraIndex));
            Q_EMIT gstError("Supplied sink is null", cameraIndex);
            return;
        }
        if (_pipelines[cameraIndex].isNull())
        {
            Logger::logError(LogTag, QString("Failed to create pipeline video%1Pipeline").arg(cameraIndex));
            Q_EMIT gstError("Failed to create pipeline", cameraIndex);
            return;
        }
        if (_bins[cameraIndex].isNull())
        {
            Logger::logError(LogTag, QString("Failed to create binary '%1' for video%2Pipeline").arg(sourceBinString, QString::number(cameraIndex)));
            Q_EMIT gstError(QString("Failed to create binary '%1'").arg(sourceBinString), cameraIndex);
            return;
        }

        _pipelines[cameraIndex]->add(_bins[cameraIndex], _sinks[cameraIndex]);
        _bins[cameraIndex]->link(_sinks[cameraIndex]);

        _pipelineWatches[cameraIndex] = new GStreamerPipelineWatch(cameraIndex, _pipelines[cameraIndex], this);
        connect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoController::gstError);

        _pipelines[cameraIndex]->setState(QGst::StatePlaying);
    }
}

void VideoController::stopVideoOnSink(uint cameraIndex)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Stop the video on the specified sink, and play a placeholder animation
        Logger::logInfo(LogTag, "Stopping video " + QString::number(cameraIndex));
        constructPipelineOnSink(cameraIndex, GStreamerUtil::createVideoTestSrcString("smpte", true, 800, 600, 10));
        _videoStates[cameraIndex] = GStreamerUtil::VideoProfile();
        Q_EMIT stopped(cameraIndex);
    }
}

void VideoController::playVideoOnSink(uint cameraIndex, GStreamerUtil::VideoProfile profile)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Play the video on the specified sink
        Logger::logInfo(LogTag, "Playing video " + QString::number(cameraIndex) + " with codec " + GStreamerUtil::getCodecName(profile.codec));
        constructPipelineOnSink(cameraIndex, GStreamerUtil::createRtpVideoDecodeString(
                                    QHostAddress::Any,
                                    SORO_NET_MC_FIRST_VIDEO_PORT + cameraIndex,
                                    profile.codec,
                                    _settings->getEnableHwDecoding()));
        _videoStates[cameraIndex] = profile;
        Q_EMIT playing(cameraIndex, profile);
    }
}

void VideoController::stop(uint cameraIndex)
{
    if (cameraIndex < (uint)_cameraSettings->getCameraCount())
    {
        // Send a request to the rover to stop a video stream
        ros_generated::video msg = GStreamerUtil::VideoProfile().toRosMessage();
        msg.camera_computerIndex = _cameraSettings->getCamera(cameraIndex).computerIndex;
        msg.camera_offset = _cameraSettings->getCamera(cameraIndex).offset;
        msg.camera_matchSerial = _cameraSettings->getCamera(cameraIndex).serial.toStdString();
        msg.camera_matchProductId = _cameraSettings->getCamera(cameraIndex).productId.toStdString();
        msg.camera_matchVendorId = _cameraSettings->getCamera(cameraIndex).vendorId.toStdString();

        Logger::logInfo(LogTag, QString("Sending video OFF request to the rover for camera %1").arg(cameraIndex));
        _videoRequestPublisher.publish(msg);
    }
}

void VideoController::stopAll()
{
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        stop(i);
    }
}

void VideoController::clearPipeline(uint cameraIndex)
{
    if (!_pipelines.value(cameraIndex).isNull())
    {
        _pipelines[cameraIndex]->setState(QGst::StateNull);
        _pipelines[cameraIndex].clear();
        _bins[cameraIndex].clear();
    }
    if (_pipelineWatches.value(cameraIndex) != nullptr)
    {
        disconnect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoController::gstError);
        delete _pipelineWatches[cameraIndex];
        _pipelineWatches[cameraIndex] = nullptr;
    }
}

bool VideoController::isPlaying(uint cameraIndex) const
{
    return _videoStates.value(cameraIndex).codec != GStreamerUtil::CODEC_NULL;
}

GStreamerUtil::VideoProfile VideoController::getVideoProfile(uint cameraIndex) const
{
    return _videoStates.value(cameraIndex);
}

} // namespace Soro
