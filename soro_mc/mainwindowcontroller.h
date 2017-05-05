#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>
#include <Qt5GStreamer/QGst/Element>
#include <SDL2/SDL.h>

#include <ros/ros.h>

#include "settingsmodel.h"
#include "soro_core/camerasettingsmodel.h"
#include "ros_generated/notification.h"
#include "soro_core/gstreamerutil.h"

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:

    explicit MainWindowController(QQmlEngine *engine, const SettingsModel *settings,
                                  const CameraSettingsModel *cameraSettings, QObject *parent = 0);

    void notify(int type, QString title, QString message);
    void notifyAll(int type, QString title, QString message);

    QVector<QGst::ElementPtr> getVideoSinks();

Q_SIGNALS:
    void keyPressed(int key);

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

private:
    void onNewNotification(ros_generated::notification msg);

    QQuickWindow *_window;

    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;

    ros::NodeHandle _nh;
    ros::Publisher _notifyPublisher;
    ros::Subscriber _notifySubscriber;
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
