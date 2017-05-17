#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>
#include <Qt5GStreamer/QGst/Element>
#include <SDL2/SDL.h>

#include "qmqtt/qmqtt.h"

#include "settingsmodel.h"
#include "soro_core/camerasettingsmodel.h"
#include "soro_core/notificationmessage.h"
#include "soro_core/gstreamerutil.h"

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:

    explicit MainWindowController(QQmlEngine *engine, const SettingsModel *settings,
                                  const CameraSettingsModel *cameraSettings, QObject *parent = 0);

    void notify(NotificationMessage::Level level, QString title, QString message);
    void notifyAll(NotificationMessage::Level level, QString title, QString message);

    QVector<QGst::ElementPtr> getVideoSinks();

Q_SIGNALS:
    void keyPressed(int key);
    void mqttConnected();
    void mqttDisconnected();

public Q_SLOTS:
    void onAudioProfileChanged(GStreamerUtil::AudioProfile profile);
    void onVideoProfileChanged(uint cameraIndex, GStreamerUtil::VideoProfile profile);
    void toggleSidebar();
    void dismissNotifications();
    void selectViewAbove();
    void selectViewBelow();
    void onConnectedChanged(bool connected);
    void onLatencyUpdated(quint32 latency);
    void onDataRateUpdated(quint64 rateUp, quint64 rateDown);

private Q_SLOTS:
    void onMqttConnected();
    void onMqttDisconnected();
    void onMqttMessage(const QMQTT::Message &msg);

private:
    QQuickWindow *_window;

    quint16 _notificationMsgId;
    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;

    QMQTT::Client *_mqtt;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
