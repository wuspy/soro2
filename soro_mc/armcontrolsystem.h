#ifndef ARMCONTROLSYSTEM_H
#define ARMCONTROLSYSTEM_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "qmqtt/qmqtt.h"

namespace Soro {

class ArmControlSystem: public QObject
{
    Q_OBJECT

public:
    explicit ArmControlSystem(QHostAddress brokerAddress, quint16 brokerPort, int masterArmConnectionTimeout, QObject *parent=0);

Q_SIGNALS:
    void dataWritten(uint bytes);
    void masterArmConnectedChanged(bool connected);

public Q_SLOTS:
    void enable();
    void disable();

private:
    QTimer _watchdogTimer;
    bool _enabled;
    bool _masterConnected;
    quint16 _nextMqttMsgId;
    QUdpSocket _armUdpSocket;
    QMQTT::Client *_mqtt;
    char _buffer[USHRT_MAX];
};

}

#endif // ARMCONTROLSYSTEM_H
