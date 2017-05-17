#ifndef MASTERAUDIOCONTROLLER_H
#define MASTERAUDIOCONTROLLER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimerEvent>

#include "qmqtt/qmqtt.h"

#include "soro_core/camerasettingsmodel.h"
#include "settingsmodel.h"

namespace Soro {

class MasterAudioController : public QObject
{
    Q_OBJECT
public:
    explicit MasterAudioController(const SettingsModel *settings, QObject *parent = 0);

Q_SIGNALS:
    void bytesDown(quint32 bytes);
    void bounceAddressesChanged(const QHash<QString, QHostAddress>& addresses);

protected:
    void timerEvent(QTimerEvent *e);

private Q_SLOTS:
    void onMqttMessage(const QMQTT::Message &msg);
    void onMqttConnected();
    void onMqttDisconnected();

private:
    void onSocketReadyRead(QUdpSocket *socket, quint16 bouncePort);

    char _buffer[65536];
    int _announceTimerId;
    QMQTT::Client *_mqtt;
    const SettingsModel *_settings;
    QUdpSocket *_audioSocket;
    QHash<QString, QHostAddress> _bounceMap;
    QList<QHostAddress> _bounceAddresses;
};

} // namespace Soro

#endif // MASTERAUDIOCONTROLLER_H
