#ifndef AUDIOCONTROLLER_H
#define AUDIOCONTROLLER_H

#include <QObject>
#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Bin>
#include <QTimerEvent>

#include "qmqtt/qmqtt.h"

#include "soro_core/gstreamerutil.h"
#include "settingsmodel.h"
#include "gstreamerpipelinewatch.h"

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
    explicit AudioController(const SettingsModel *settings, QObject *parent = 0);
    ~AudioController();

    bool isPlaying() const;
    /* Gets the profile of the audio currently playing
     */
    GStreamerUtil::AudioProfile getProfile() const;

Q_SIGNALS:
    /* Emitted when the rover confirms audio is playing with the specified codec
     */
    void playing(GStreamerUtil::AudioProfile profile);
    /* Emitted when the rover confirms audio is no longer streaming
     */
    void stopped();
    /* Emitted when an audio decoding error is encountered. This does not mean the
     * rover is no longer streaming audio, only that we could not play it
     */
    void gstError(QString message);

    void mqttConnected();
    void mqttDisconnected();

public Q_SLOTS:
    void play(GStreamerUtil::AudioProfile profile);
    void stop();

protected:
    void timerEvent(QTimerEvent *e);

private Q_SLOTS:
    void onMqttConnected();
    void onMqttDisconnected();
    void onMqttMessage(const QMQTT::Message &msg);

private:
    void clearPipeline();

    const SettingsModel *_settings;
    int _announceTimerId;
    quint16 _nextMqttMsgId;
    QGst::PipelinePtr _pipeline;
    QGst::BinPtr _bin;
    GStreamerPipelineWatch *_pipelineWatch;

    QMQTT::Client *_mqtt;

    GStreamerUtil::AudioProfile _profile;
};

} // namespace Soro

#endif // AUDIOCONTROLLER_H
