#ifndef VIDEOCONTROLLER_H
#define VIDEOCONTROLLER_H

#include <QObject>

#include "soro_core/camerasettingsmodel.h"
#include "settingsmodel.h"
#include "gstreamerpipelinewatch.h"

#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>
#include <Qt5GStreamer/QGst/Element>

#include "ros_generated/video.h"
#include "ros_generated/video_state.h"

#include "soro_core/gstreamerutil.h"

#include <ros/ros.h>

namespace Soro {

/* Controls the rover's video system.
 *
 * You can play, stop, or change the streaming parameters of a camera with the play() and stop() functions().
 * Once the rover responds to such a request, the playing() and stopped() signals will be emitted. Keep in mind
 * that in a situation where multiple mission controls are controlling the rover, the playing() and stopped() signals
 * may still be emitted if another mission control requests such a change, without any action from us.
 *
 * When this class is first created, it requires a list of QGst::ElementPtr that serve as the sinks for each of the
 * videos we may receive. These cannot be changed without destroying and recreating the VideoController instance.
 * Videos are assigned to sinks based on their index, so video 0 will play on sink index 0, etc.
 *
 * When no video is being streamed on any particular sink, a placeholder animation will be shown using a
 * videotestsrc animation.
 *
 * Additionally, the signals gstError() and gstEos() may be emitted if there is an error decoding the video streamed
 * by the rover.
 */
class VideoController : public QObject
{
    Q_OBJECT
public:
    explicit VideoController(const SettingsModel *settings, const CameraSettingsModel *cameraSettings, QVector<QGst::ElementPtr> sinks, QObject *parent = 0);
    ~VideoController();

    bool isPlaying(uint cameraIndex) const;
    GStreamerUtil::VideoProfile getVideoProfile(uint cameraIndex) const;

    void play(uint cameraIndex, GStreamerUtil::VideoProfile profile);
    void stop(uint cameraIndex);

Q_SIGNALS:
    void playing(uint cameraIndex, GStreamerUtil::VideoProfile profile);
    void stopped(uint cameraIndex);

    /* Emitted when an audio decoding error is encountered. This does not mean the
     * rover is no longer streaming this camera, only that we could not play it
     */
    void gstError(QString message, uint cameraIndex);

private:
    void onVideoResponse(ros_generated::video_state msg);
    void clearPipeline(uint cameraIndex);
    void playVideoOnSink(uint cameraIndex, GStreamerUtil::VideoProfile profile);
    void constructPipelineOnSink(uint cameraIndex, QString sourceBinString);
    void stopVideoOnSink(uint cameraIndex);

    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;

    QVector<QGst::PipelinePtr> _pipelines;
    QVector<QGst::BinPtr> _bins;
    QVector<QGst::ElementPtr> _sinks;
    QVector<GStreamerPipelineWatch*> _pipelineWatches;
    QVector<GStreamerUtil::VideoProfile> _videoStates;

    ros::NodeHandle _nh;
    ros::Publisher _videoRequestPublisher;
    ros::Subscriber _videoStateSubscriber;
};

} // namespace Soro

#endif // VIDEOCONTROLLER_H
