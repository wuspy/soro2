#ifndef AUDIOSERVER_H
#define AUDIOSERVER_H

#include <QObject>
#include <QTimerEvent>
#include <QtDBus>
#include <QHostAddress>
#include <QUdpSocket>

#include "qmqtt/qmqtt.h"

#include "settingsmodel.h"
#include "soro_core/audiomessage.h"
#include "soro_core/gstreamerutil.h"

namespace Soro {

class AudioServer : public QObject
{
    Q_OBJECT
public:
    explicit AudioServer(const SettingsModel *settings, QObject *parent = 0);
    ~AudioServer();

public Q_SLOTS:
    void onChildError(QString message);
    void onChildReady();
    void onChildStreaming();
    void onChildLogInfo(const QString &tag, const QString &message);

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
        QHostAddress address;
        quint16 port;
        AudioMessage message;

        Assignment();
    };

    void giveChildAssignment(Assignment assignment);
    void terminateChild();
    void reportAudioState();

    const SettingsModel *_settings;
    int _heartbeatTimerId;
    quint16 _nextMqttMsgId;

    // These key to these hash sets is the device the child is assigned
    // to stream. Each child is spawned to stream a single device (usually
    // a /dev/video* deivce), and it will only ever serve streams for that
    // single device. This system is done to prevent multiple children from
    // attempting to stream the same device, which would fail.
    QProcess *_child;
    QDBusInterface* _childInterface;
    Assignment _waitingAssignment;
    bool _hasWaitingAssignment;
    Assignment _currentAssignment;
    bool _hasCurrentAssignment;
    QUdpSocket _audioSocket;
    QHostAddress _clientAddress;
    quint16 _clientPort;

    QMQTT::Client *_mqtt;

};

} // namespace Soro

#endif // AUDIOSERVER_H
