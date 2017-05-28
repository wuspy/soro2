#ifndef CONNECTIONSTATUSCONTROLLER_H
#define CONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QTimer>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"

namespace Soro {

/* Handles connection state monitoring for the soro_mc application.
 */
class ConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit ConnectionStatusController(const SettingsModel *settings, QObject *parent = 0);

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdate(quint32 latency);
    void connectedChanged(bool connected);
    void dataRateUpdate(quint64 rateFromRover);

private:
    QMQTT::Client *_mqtt;
    QTimer _connectionWatchdog;
    bool _connected;
};

} // namespace Soro

#endif // CONNECTIONSTATUSCONTROLLER_H
