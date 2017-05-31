#ifndef SCIENCEPACKAGECONTROLLER_H
#define SCIENCEPACKAGECONTROLLER_H

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

Q_SIGNALS:
    void sciencePackageConnectedChanged(bool connected);

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

#endif // SCIENCEPACKAGECONTROLLER_H
