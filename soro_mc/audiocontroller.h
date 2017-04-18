#ifndef AUDIOCONTROLLER_H
#define AUDIOCONTROLLER_H

#include <QObject>
#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>

#include <ros/ros.h>
#include "ros_generated/audio.h"
#include "gstreamerpipelinewatch.h"

namespace Soro {

class AudioController : public QObject
{
    Q_OBJECT
public:
    explicit AudioController(QObject *parent = 0);

    bool isPlaying() const;
    quint8 getCodec() const;

Q_SIGNALS:
    /* Emitted when the rover confirms audio is playing with the specified codec
     */
    void playing(quint8 codec);
    /* Emitted when the rover confirms audio is no longer streaming
     */
    void stopped();
    /* Emitted when an audio decoding error is encountered. This does not mean the
     * rover is no longer streaming audio, only that we could not play it
     */
    void error(QString message);
    /* Emitted when an EOS message is encountered while playing the stream. This should
     * really never happen, and can be considered a decoding error.
     */
    void eos();

public Q_SLOTS:
    void play(quint8 codec);
    void stop();

private:
    void clearPipeline();
    void onAudioResponse(ros_generated::audio msg);

    QGst::PipelinePtr _pipeline;
    QGst::BinPtr _bin;
    GStreamerPipelineWatch *_pipelineWatch;

    ros::NodeHandle _nh;
    ros::Subscriber _audioSubscriber;
    ros::Publisher _audioPublisher;

    quint8 _codec;
};

} // namespace Soro

#endif // AUDIOCONTROLLER_H
