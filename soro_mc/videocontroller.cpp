#include "videocontroller.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"
#include "libsoromc/gstreamerutil.h"

#include <Qt5GStreamer/QGst/Bus>

#include "maincontroller.h"

#define LogTag "VideoController"

namespace Soro {

VideoController::VideoController(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QList<QGst::ElementPtr> sinks, QObject *parent) : QObject(parent)
{
    _settings = settings;
    _cameraSettings = cameraSettings;
    _sinks = sinks;

    // Fill video state lists with default values
    for (int i = 0; i < cameraSettings->getCameraCount(); ++i)
    {
        _pipelines.append(QGst::PipelinePtr());
        _bins.append(QGst::BinPtr());
        _codecs.append(CODEC_NULL);
        _pipelineWatches.append(nullptr);

        stopVideoOnSink(i);
    }

    Logger::logInfo(LogTag, "Creating ROS subscriber for video_ack topic...");
    _videoSubscriber = _nh.subscribe
            <ros_generated::video, Soro::VideoController>
            ("video_ack", 1, &VideoController::onVideoResponse, this);
    if (!_videoSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for video_ack topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for video topic...");
    _videoPublisher = _nh.advertise<ros_generated::video>("video", 1);
    if (!_videoPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for video topic");

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");
}

VideoController::~VideoController()
{
    for (int i = 0; i < _cameraSettings->getCameraCount(); ++i)
    {
        clearPipeline(i);
    }
}

void VideoController::onVideoResponse(ros_generated::video msg)
{
    if (msg.on)
    {
        // Video is streaming
        playVideoOnSink(msg.cameraId, msg.encoding);
    }
    else
    {
        // Video is NOT streaming
        stopVideoOnSink(msg.cameraId);
    }
}

void VideoController::play(uint cameraIndex, quint8 codec, uint width, uint height, uint framerate, uint bitrate, uint quality)
{
    // Send a request to the rover to start/change a video stream
    ros_generated::video msg;
    msg.cameraId = cameraIndex;
    msg.on = true;
    msg.encoding = codec;
    msg.bitrate = bitrate;
    msg.fps = framerate;
    msg.width = width;
    msg.height = height;
    msg.quality = quality;

    Logger::logInfo(LogTag, QString("Sending video ON request to the rover for camera %1: [Codec %2, %3x%4, %5fps, %6bps, %7q]").arg(
                        QString::number(cameraIndex), QString::number(codec), QString::number(width), QString::number(height),
                        QString::number(framerate), QString::number(bitrate), QString::number(quality)));
    _videoPublisher.publish(msg);
}

void VideoController::constructPipelineOnSink(uint cameraIndex, QString sourceBinString)
{
    clearPipeline(cameraIndex);

    Logger::logInfo(LogTag, QString("Starting pipeline '%1' on sink %2").arg(sourceBinString, QString::number(cameraIndex)));
    _pipelines[cameraIndex] = QGst::Pipeline::create(QString("video%1Pipeline").arg(cameraIndex).toLatin1().constData());
    _bins[cameraIndex] = QGst::Bin::fromDescription(sourceBinString);

    if (_sinks[cameraIndex].isNull())
    {
        Logger::logError(LogTag, QString("Supplied sink for video %1 is NULL").arg(cameraIndex));
        Q_EMIT error("Supplied sink is null", cameraIndex);
        return;
    }
    if (_pipelines[cameraIndex].isNull())
    {
        Logger::logError(LogTag, QString("Failed to create pipeline video%1Pipeline").arg(cameraIndex));
        Q_EMIT error("Failed to create pipeline", cameraIndex);
        return;
    }
    if (_bins[cameraIndex].isNull())
    {
        Logger::logError(LogTag, QString("Failed to create binary '%1' for video%2Pipeline").arg(sourceBinString, QString::number(cameraIndex)));
        Q_EMIT error(QString("Failed to create binary '%1'").arg(sourceBinString), cameraIndex);
        return;
    }

    _pipelines[cameraIndex]->add(_bins[cameraIndex], _sinks[cameraIndex]);
    _bins[cameraIndex]->link(_sinks[cameraIndex]);

    _pipelineWatches[cameraIndex] = new GStreamerPipelineWatch(cameraIndex, _pipelines[cameraIndex], this);
    connect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::eos, this, &VideoController::eos);
    connect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoController::error);

    _pipelines[cameraIndex]->setState(QGst::StatePlaying);
}

void VideoController::stopVideoOnSink(uint cameraIndex)
{
    // Stop the video on the specified sink, and play a placeholder animation
    constructPipelineOnSink(cameraIndex, GStreamerUtil::createVideoTestSrcBinString("smpte75", 800, 600, 24));
    _codecs[cameraIndex] = CODEC_NULL;
    Q_EMIT stopped(cameraIndex);
}

void VideoController::playVideoOnSink(uint cameraIndex, quint8 codec)
{
    // Play the video on the specified sink
    constructPipelineOnSink(cameraIndex, GStreamerUtil::createRtpVideoDecodeBinString(
                                QHostAddress::Any,
                                SORO_NET_FIRST_VIDEO_PORT + cameraIndex,
                                codec,
                                _settings->getEnableHwDecoding()));
    _codecs[cameraIndex] = codec;
    Q_EMIT playing(cameraIndex, codec);
}

void VideoController::stop(uint cameraIndex)
{
    // Send a request to the rover to stop a video stream
    ros_generated::video msg;
    msg.cameraId = cameraIndex;
    msg.on = false;
    msg.encoding = CODEC_NULL;

    Logger::logInfo(LogTag, QString("Sending video OFF request to the rover for camera %1").arg(cameraIndex));
    _videoPublisher.publish(msg);
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
        disconnect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::eos, this, &VideoController::eos);
        disconnect(_pipelineWatches[cameraIndex], &GStreamerPipelineWatch::error, this, &VideoController::error);
        delete _pipelineWatches[cameraIndex];
        _pipelineWatches[cameraIndex] = nullptr;
    }
}

bool VideoController::isPlaying(uint cameraIndex) const
{
    return _codecs.value(cameraIndex) != CODEC_NULL;
}

quint8 VideoController::getCodec(uint cameraIndex) const
{
    return _codecs.value(cameraIndex);
}

} // namespace Soro
