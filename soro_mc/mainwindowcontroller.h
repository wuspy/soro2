#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>
#include <Qt5GStreamer/QGst/Element>
#include <SDL2/SDL.h>

#include <ros/ros.h>

#include "settingsmodel.h"
#include "libsoromc/camerasettingsmodel.h"
#include "ros_generated/notification.h"

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:

    explicit MainWindowController(QQmlEngine *engine, const SettingsModel *settings,
                                  const CameraSettingsModel *cameraSettings, QObject *parent = 0);

    void notify(int type, QString title, QString message);
    void notifyAll(int type, QString title, QString message);

    QList<QGst::ElementPtr> getVideoSinks();

private:
    void onNewNotification(ros_generated::notification msg);

    QQuickWindow *_window;

    const SettingsModel *_settings;
    const CameraSettingsModel *_cameraSettings;

    ros::NodeHandle _nh;
    ros::Publisher _notifyPublisher;
    ros::Subscriber _notifySubscriber;

public Q_SLOTS:
    void onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed);
    void onConnectedChanged(bool connected);
    void onLatencyUpdated(quint32 latency);
    void onBitrateUpdated(quint64 bitsUp, quint64 bitsDown);
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
