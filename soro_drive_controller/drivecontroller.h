#ifndef DRIVECONTROLLER_H
#define DRIVECONTROLLER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

/* Class to control the rover's drive system through a LAN UDP socket
 * from instructions sent over MQTT
 */
class DriveController: public QObject
{
    Q_OBJECT

public:
    explicit DriveController(const SettingsModel* settings, QObject *parent=0);

Q_SIGNALS:
    void driveMicrocontrollerConnectedChanged(bool connected);

private:
    bool _driveConnected;
    QTimer _watchdogTimer;
    quint16 _nextMqttMsgId;
    QUdpSocket _driveUdpSocket;
    QMQTT::Client *_mqtt;
    char _buffer[USHRT_MAX];
};

} // namespace Soro

#endif // DRIVECONTROLLER_H
