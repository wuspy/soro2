#ifndef AUDIOCONTROLLER_H
#define AUDIOCONTROLLER_H

#include <QObject>
#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>

#include <ros/ros.h>
#include "gstreamerpipelinewatch.h"
#include "ros_generated/audio.h"

namespace Soro {

/* Controls the rover's audio system.
 *
 * You can play or stop the rover's audio stream with the play() and stop() functions. Once
 * the rover responds to this request, the playing() and stopped() signals will be emitted.
 *
 * Additionally, the gstError() and gstEsos() signals may be emitted if there is an error decoding
 * the audio stream.
 */
class AudioController : public QObject
{
    Q_OBJECT
public:
    explicit AudioController(QObject *parent = 0);
    ~AudioController();

    bool isPlaying() const;
    /* Gets the codec of the audio currently playing
     */
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
    void gstError(QString message);
    /* Emitted when an EOS message is encountered while playing the stream. This should
     * really never happen, and can be considered a decoding error.
     */
    void gstEos();

public Q_SLOTS:
    void play(quint8 codec);
    void stop();

private:
    void onAudioResponse(ros_generated::audio msg);
    void clearPipeline();

    QGst::PipelinePtr _pipeline;
    QGst::BinPtr _bin;
    GStreamerPipelineWatch *_pipelineWatch;

    ros::NodeHandle _nh;
    ros::Subscriber _audioStateSubscriber;
    ros::Publisher _audioStatePublisher;

    quint8 _codec;
};

} // namespace Soro

#endif // AUDIOCONTROLLER_H
