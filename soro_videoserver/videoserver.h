#ifndef MEDIASERVER_H
#define MEDIASERVER_H

#include <QObject>
#include <QTimerEvent>
#include <QtDBus>
#include <QHostAddress>

#include <ros/ros.h>

#include "soro_core/gstreamerutil.h"
#include "soro_core/rosnodelist.h"

#include "ros_generated/video.h"
#include "ros_generated/video_state.h"
#include "ros_generated/notification.h"

namespace Soro {

class VideoServer : public QObject
{
    Q_OBJECT
public:
    explicit VideoServer(int computerIndex, const RosNodeList *rosNodeList, QObject *parent = 0);
    ~VideoServer();
    void setShouldUseVaapiForCodec(quint8 codec, bool vaapi);

public Q_SLOTS:
    void onChildError(QString childName, QString message);
    void onChildReady(QString childName);
    void onChildStreaming(QString childName);
    void onChildLogInfo(QString childName, const QString &tag, const QString &message);

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
        ros_generated::video originalMessage;
        bool vaapi;

        Assignment();
    };

    void giveChildAssignment(Assignment assignment);
    void terminateChild(QString childName);
    void reportVideoState();

    void onVideoStateMessage(ros_generated::video_state msg);
    void onVideoRequestMessage(ros_generated::video msg);

    QString findUsbCamera(QString serial, QString productId, QString vendorId, int offset);

    int _computerIndex;
    int _heartbeatTimerId;
    const RosNodeList *_rosNodeList;
    ros_generated::video_state _lastVideoStateMsg;
    ros::NodeHandle _nh;
    ros::Publisher _videoStatePublisher;
    ros::Publisher _notificationPublisher;
    ros::Subscriber _videoStateSubscriber;
    ros::Subscriber _videoRequestSubscriber;

    // These key to these hash sets is the device the child is assigned
    // to stream. Each child is spawned to stream a single device (usually
    // a /dev/video* deivce), and it will only ever serve streams for that
    // single device. This system is done to prevent multiple children from
    // attempting to stream the same device, which would fail.
    QHash<QString, QProcess*> _children;
    QHash<QString, QDBusInterface*> _childInterfaces;
    QHash<QString, Assignment> _waitingAssignments;
    QHash<QString, Assignment> _currentAssignments;
    QHash<quint8, bool> _useVaapi;

};

} // namespace Soro

#endif // MEDIASERVER_H
