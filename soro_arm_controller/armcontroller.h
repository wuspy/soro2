#ifndef SCIENCEPACKAGECONTROLLER_H
#define SCIENCEPACKAGECONTROLLER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

/* Class to control the rover's physical arm through a LAN UDP socket
 * from instructions sent over MQTT
 */
class ArmController: public QObject
{
    Q_OBJECT

public:
    explicit ArmController(const SettingsModel* settings, QObject *parent=0);

Q_SIGNALS:
    void armConnectedChanged(bool connected);

private:
    bool _armConnected;
    QTimer _watchdogTimer;
    quint16 _nextMqttMsgId;
    QUdpSocket _armUdpSocket;
    QMQTT::Client *_mqtt;
    char _buffer[USHRT_MAX];
};

}

#endif // SCIENCEPACKAGECONTROLLER_H
