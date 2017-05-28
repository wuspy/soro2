#ifndef ARMCONTROLSYSTEM_H
#define ARMCONTROLSYSTEM_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

class SciencePackageController: public QObject
{
    Q_OBJECT

public:
    explicit SciencePackageController(const SettingsModel* settings, QObject *parent=0);

private:
    bool _packageConnected;
    QTimer _watchdogTimer;
    quint16 _nextMqttMsgId;
    QUdpSocket _packageUdpSocket;
    QMQTT::Client *_mqtt;
    float _lastCompassHeading = 0;
    char _buffer[USHRT_MAX];
};

}

#endif // ARMCONTROLSYSTEM_H
