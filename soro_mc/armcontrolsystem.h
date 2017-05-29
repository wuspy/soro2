#ifndef ARMCONTROLSYSTEM_H
#define ARMCONTROLSYSTEM_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

/* Component to read instructions from the master arm over a LAN UDP socket,
 * and package them into MQTT messages
 */
class ArmControlSystem: public QObject
{
    Q_OBJECT

public:
    explicit ArmControlSystem(const SettingsModel* settings, QObject *parent=0);

Q_SIGNALS:
    void masterArmConnectedChanged(bool connected);
    void slaveArmDisconnected();
    void armControllerDisconnected();

public Q_SLOTS:
    void enable();
    void disable();

private:
    QTimer _watchdogTimer;
    bool _masterConnected;
    bool _enabled;
    quint16 _nextMqttMsgId;
    QUdpSocket _armUdpSocket;
    QMQTT::Client *_mqtt;
    char _buffer[USHRT_MAX];
};

} // namespace Soro

#endif // ARMCONTROLSYSTEM_H
