#ifndef MAINWINDOWCONTROLLER_H
#define MAINWINDOWCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QQuickWindow>
#include <Qt5GStreamer/QGst/Pipeline>
#include <Qt5GStreamer/QGst/Element>
#include <Qt5GStreamer/QGst/Bin>
#include <SDL2/SDL.h>

#include <ros/ros.h>

#include "libsoromc/videoformat.h"
#include "libsoromc/notificationmessage.h"

namespace Soro {

class MainWindowController : public QObject
{
    Q_OBJECT
public:

    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

    void stopVideo(int cameraId, QString pattern="snow");
    void playVideo(int cameraId, VideoFormat format);
    void notify(int type, QString title, QString message);
    void notifyAll(int type, QString title, QString message);

private:
    void clearVideo(int cameraId);

    void onNewNotification(message_gen::notification msg);

    QQuickWindow *_window;
    QGst::PipelinePtr _videoPipelines[8];
    QGst::BinPtr _videoBins[8];

    ros::Publisher _notifyPublisher;
    ros::Subscriber _notifySubscriber;


private Q_SLOTS:
    void onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed);
    void onConnectedChanged(bool connected);
    void onLatencyUpdated(quint32 latency);
    void onBitrateUpdated(quint64 bitsUp, quint64 bitsDown);
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
