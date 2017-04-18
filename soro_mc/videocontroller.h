#ifndef VIDEOCONTROLLER_H
#define VIDEOCONTROLLER_H

#include <QObject>

#include "libsoromc/camerasettingsmodel.h"
#include "settingsmodel.h"
#include "gstreamerpipelinewatch.h"

#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>
#include <Qt5GStreamer/QGst/Element>

#include "ros_generated/video.h"

#include <ros/ros.h>

namespace Soro {

class VideoController : public QObject
{
    Q_OBJECT
public:
    explicit VideoController(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QList<QGst::ElementPtr> sinks, QObject *parent = 0);
    ~VideoController();

    bool isPlaying(uint cameraIndex) const;
    quint8 getCodec(uint cameraIndex) const;

    void play(uint cameraIndex, quint8 codec, uint width, uint height, uint framerate, uint bitrate, uint quality);
    void stop(uint cameraIndex);

Q_SIGNALS:
    void playing(uint cameraIndex, quint8 codec);
    void stopped(uint cameraIndex);
    /* Emitted when an EOS message is encountered while playing the stream. This should
     * really never happen, and can be considered a decoding error.
     */
    void eos(uint cameraIndex);
    /* Emitted when an audio decoding error is encountered. This does not mean the
     * rover is no longer streaming this camera, only that we could not play it
     */
    void error(QString message, uint cameraIndex);

private:
    void onVideoResponse(ros_generated::video msg);
    void clearPipeline(uint cameraIndex);
    void playVideoOnSink(uint cameraIndex, quint8 codec);
    void constructPipelineOnSink(uint cameraIndex, QString sourceBinString);
    void stopVideoOnSink(uint cameraIndex);

    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;

    QList<QGst::PipelinePtr> _pipelines;
    QList<QGst::BinPtr> _bins;
    QList<QGst::ElementPtr> _sinks;
    QList<uint> _codecs;
    QList<GStreamerPipelineWatch*> _pipelineWatches;

    ros::NodeHandle _nh;
    ros::Publisher _videoPublisher;
    ros::Subscriber _videoSubscriber;
};

} // namespace Soro

#endif // VIDEOCONTROLLER_H
