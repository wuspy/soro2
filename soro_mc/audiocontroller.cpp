#include "audiocontroller.h"
#include "libsoromc/gstreamerutil.h"
#include "libsoromc/constants.h"
#include "libsoromc/logger.h"
#include <Qt5GStreamer/QGst/Bus>

#include "maincontroller.h"

#define LogTag "AudioController"

namespace Soro {

AudioController::AudioController(QObject *parent) : QObject(parent)
{
    Logger::logInfo(LogTag, "Creating ROS subscriber for audio_ack topic...");
    _audioSubscriber = _nh.subscribe
            <ros_generated::audio, Soro::AudioController>
            ("audio_ack", 1, &AudioController::onAudioResponse, this);
    if (!_audioSubscriber) MainController::panic(LogTag, "Failed to create ROS subscriber for audio_ack topic");

    Logger::logInfo(LogTag, "Creating ROS publisher for audio topic...");
    _audioPublisher = _nh.advertise<ros_generated::audio>("audio", 1);
    if (!_audioPublisher) MainController::panic(LogTag, "Failed to create ROS publisher for audio topic");

    Logger::logInfo(LogTag, "All ROS publishers and subscribers created");

    _codec = CODEC_NULL;
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
        _bin = QGst::Bin::fromDescription(GStreamerUtil::createRtpAudioPlayBinString(QHostAddress::Any, SORO_NET_AUDIO_PORT, msg.encoding));
        _pipeline->add(_bin);

        // Add signal watch to subscribe to bus events, like errors
        _pipelineWatch = new GStreamerPipelineWatch(0, _pipeline, this);
        connect(_pipelineWatch, &GStreamerPipelineWatch::eos, this, &AudioController::eos);
        connect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::error);

        // Play
        _pipeline->setState(QGst::StatePlaying);
        _codec = msg.encoding;
        Q_EMIT playing(_codec);
    }
    else
    {
        // Audio is not streaming
        _codec = CODEC_NULL;
        Q_EMIT stopped();
    }
}

bool AudioController::isPlaying() const
{
    return _codec != CODEC_NULL;
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
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::eos, this, &AudioController::eos);
        disconnect(_pipelineWatch, &GStreamerPipelineWatch::error, this, &AudioController::error);
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
    _audioPublisher.publish(msg);
}

void AudioController::stop()
{
    // Send a request to the rover to STOP the audio stream
    ros_generated::audio msg;
    msg.encoding = CODEC_NULL;
    msg.on = false;

    Logger::logInfo(LogTag, "Sending audio OFF requet to rover");
    _audioPublisher.publish(msg);
}

} // namespace Soro
