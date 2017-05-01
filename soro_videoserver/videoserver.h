#ifndef MEDIASERVER_H
#define MEDIASERVER_H

#include <QObject>
#include <QTimerEvent>
#include <QtDBus>
#include <QHostAddress>

#include <ros/ros.h>

#include "soro_core/gstreamerutil.h"

#include "ros_generated/video.h"
#include "ros_generated/video_state.h"
#include "ros_generated/notification.h"

namespace Soro {

class VideoServer : public QObject
{
    Q_OBJECT
public:
    explicit VideoServer(int computerIndex, QObject *parent = 0);
    ~VideoServer();
    void setShouldUseVaapiForCodec(quint8 codec, bool vaapi);

public Q_SLOTS:
    void onChildError(qint64 childPid, QString message);
    void onChildReady(qint64 childPid);
    void onChildStreaming(qint64 childPid);

protected:
    void timerEvent(QTimerEvent *e);

private:
    struct Assignment
    {
        QString device;
        QString cameraName;
        QHostAddress address;
        quint16 port;
        GStreamerUtil::VideoProfile profile;
        bool vaapi;
    };

    void spawnChildForAssignment(Assignment assignment);
    void stopChild(qint64 pid);
    void killChild(qint64 pid);
    void reportVideoState();

    void onVideoStateMessage(ros_generated::video_state msg);
    void onVideoRequestMessage(ros_generated::video msg);

    QString findUsbCamera(QString serial, QString productId, QString vendorId, int offset);
    QHostAddress getPeerAddress();

    int _computerIndex;
    QVector<ros_generated::video> _videoStates;
    ros::NodeHandle _nh;
    ros::Publisher _videoStatePublisher;
    ros::Publisher _notificationPublisher;
    ros::Subscriber _videoStateSubscriber;
    ros::Subscriber _videoRequestSubscriber;

    QHash<qint64, QDBusInterface*> _childInterfaces;
    QHash<qint64, Assignment> _waitingVideoStreams;
    QHash<qint64, Assignment> _assignedVideoStreams;
    QVector<QProcess*> _children;
    QHash<quint8, bool> _useVaapi;
};

} // namespace Soro

#endif // MEDIASERVER_H
