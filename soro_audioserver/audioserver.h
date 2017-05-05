#ifndef MEDIASERVER_H
#define MEDIASERVER_H

#include <QObject>
#include <QTimerEvent>
#include <QtDBus>
#include <QHostAddress>

#include <ros/ros.h>

#include "soro_core/gstreamerutil.h"
#include "soro_core/rosnodelist.h"

#include "ros_generated/audio.h"
#include "ros_generated/notification.h"

namespace Soro {

class AudioServer : public QObject
{
    Q_OBJECT
public:
    explicit AudioServer(const RosNodeList *rosNodeList, QObject *parent = 0);
    ~AudioServer();

public Q_SLOTS:
    void onChildError(QString message);
    void onChildReady();
    void onChildStreaming();
    void onChildLogInfo(const QString &tag, const QString &message);

protected:
    void timerEvent(QTimerEvent *e);

private:
    struct Assignment
    {
        QHostAddress address;
        quint16 port;
        GStreamerUtil::AudioProfile profile;
        ros_generated::audio originalMessage;

        Assignment();
    };

    void giveChildAssignment(Assignment assignment);
    void terminateChild();
    void reportAudioState();

    void onAudioRequestMessage(ros_generated::audio msg);

    int _heartbeatTimerId;
    const RosNodeList *_rosNodeList;
    ros::NodeHandle _nh;
    ros::Publisher _audioStatePublisher;
    ros::Publisher _notificationPublisher;
    ros::Subscriber _audioRequestSubscriber;

    QProcess* _child;
    QDBusInterface* _childInterface;
    bool _hasWaitingAssignment;
    Assignment _waitingAssignment;
    bool _hasCurrentAssignment;
    Assignment _currentAssignment;

};

} // namespace Soro

#endif // MEDIASERVER_H
