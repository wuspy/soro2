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
    enum MessageType {
        MessageType_Info,
        MessageType_Warning,
        MessageType_Error
    };

    explicit MainWindowController(QQmlEngine *engine, QObject *parent = 0);

    void stopVideo(int cameraId, QString pattern="snow");
    void playVideo(int cameraId, VideoFormat format);
    void notify(MessageType type, QString title, QString message);
    void notifyAll(MessageType type, QString title, QString message);

private:
    void clearVideo(int cameraId);

    void onNewNotification(const Soro::Messages::notification msg);

    int messageTypeToInt(MessageType type);
    MessageType intToMessageType(int type);

    QQuickWindow *_window;
    QGst::PipelinePtr _videoPipelines[8];
    QGst::ElementPtr _videoSinks[8];
    QGst::BinPtr _videoBins[8];

    ros::Publisher _notifyPublisher;
    ros::Subscriber _notifySubscriber;


private slots:
    void onGamepadButtonPressed(SDL_GameControllerButton button, bool pressed);
};

} // namespace Soro

#endif // MAINWINDOWCONTROLLER_H
