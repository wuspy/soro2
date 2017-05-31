#ifndef SCIENCEARMCONTROLSYSTEM_H
#define SCIENCEARMCONTROLSYSTEM_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

/* Component to read instructions from the science master arm over a LAN UDP socket, and
 * package them as messages over MQTT.
 */
class ScienceArmControlSystem: public QObject
{
    Q_OBJECT

public:
    explicit ScienceArmControlSystem(const SettingsModel* settings, QObject *parent=0);

Q_SIGNALS:
    void masterArmConnectedChanged(bool connected);
    void sciencePackageControllerDisconnected();
    void sciencePackageDisconnected();
    void flirDisconnected();
    void lidarDisconnected();

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

} // namespace Soro

#endif // SCIENCEARMCONTROLSYSTEM_H
