#ifndef MASTERCONNECTIONSTATUSCONTROLLER_H
#define MASTERCONNECTIONSTATUSCONTROLLER_H

#include <QObject>
#include <QQueue>
#include <QTimer>
#include <QThread>

#include "settingsmodel.h"
#include "qmqtt/qmqtt.h"


namespace Soro {

/* Master implementation of the connection status controller.
 */
class MasterConnectionStatusController : public QObject
{
    Q_OBJECT
public:
    explicit MasterConnectionStatusController(const SettingsModel *settings, QObject *parent = 0);

    bool isConnected() const;

Q_SIGNALS:
    void latencyUpdate(quint32 latency);
    void dataRateUpdate(quint64 rateFromRover);
    void connectedChanged(bool connected);

public Q_SLOTS:
    void logDataFromRover(quint32 bytes);

private:
    const SettingsModel *_settings;
    QTimer _pingTimer;
    QTimer _dataRateCalcTimer;
    QQueue<quint64> _pingIdQueue;
    QQueue<qint64> _pingTimeQueue;
    quint64 _nextPing;
    quint16 _nextMqttMsgId;
    QMQTT::Client *_mqtt;
    quint64 _bytesFromRover;
};

} // namespace Soro

#endif // MASTERCONNECTIONSTATUSCONTROLLER_H
