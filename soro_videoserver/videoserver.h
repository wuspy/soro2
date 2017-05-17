#ifndef VIDEOSERVER_H
#define VIDEOSERVER_H

#include <QObject>
#include <QTimerEvent>
#include <QtDBus>
#include <QHostAddress>
#include <QUdpSocket>

#include "qmqtt/qmqtt.h"

#include "settingsmodel.h"
#include "soro_core/videomessage.h"
#include "soro_core/videostatemessage.h"
#include "soro_core/gstreamerutil.h"

namespace Soro {

class VideoServer : public QObject
{
    Q_OBJECT
public:
    explicit VideoServer(const SettingsModel *settings, QObject *parent = 0);
    ~VideoServer();

public Q_SLOTS:
    void onChildError(QString childName, QString message);
    void onChildReady(QString childName);
    void onChildStreaming(QString childName);
    void onChildLogInfo(QString childName, const QString &tag, const QString &message);

Q_SIGNALS:
    void mqttConnected();
    void mqttDisconnected();

protected:
    void timerEvent(QTimerEvent *e);

private Q_SLOTS:
    void onMqttConnected();
    void onMqttDisconnected();
    void onMqttMessage(const QMQTT::Message &msg);

private:
    struct Assignment
    {
        QString device;
        QString device2;
        bool isStereo;
        QString cameraName;
        QHostAddress address;
        quint16 port;
        GStreamerUtil::VideoProfile profile;
        VideoMessage originalMessage;
        bool vaapi;

        Assignment();
    };

    void giveChildAssignment(Assignment assignment);
    void terminateChild(QString childName);
    void reportVideoState();

    QString findUsbCamera(QString serial, QString productId, QString vendorId, int offset);

    const SettingsModel *_settings;
    int _heartbeatTimerId;
    quint16 _nextMqttMsgId;
    quint16 _nextVideoStateMsgId;
    VideoStateMessage _lastVideoStateMsg;

    // These key to these hash sets is the device the child is assigned
    // to stream. Each child is spawned to stream a single device (usually
    // a /dev/video* deivce), and it will only ever serve streams for that
    // single device. This system is done to prevent multiple children from
    // attempting to stream the same device, which would fail.
    QHash<QString, QProcess*> _children;
    QHash<QString, QDBusInterface*> _childInterfaces;
    QHash<QString, Assignment> _waitingAssignments;
    QHash<QString, Assignment> _currentAssignments;
    QHash<quint16, QUdpSocket*> _videoSockets;
    QHash<quint16, QHostAddress> _clientAddresses;
    QHash<quint16, quint16> _clientPorts;
    QHash<quint8, bool> _useVaapi;

    QMQTT::Client *_mqtt;

};

} // namespace Soro

#endif // VIDEOSERVER_H
